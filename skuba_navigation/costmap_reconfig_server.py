#!/usr/bin/env python3
"""
Costmap Dynamic Reconfiguration Server for SKUBA robot.
This node provides a simple service to dynamically modify costmap parameters
by updating YAML configuration and reloading to Nav2.

Also supports controller server parameter changes, including a dedicated
SetControllerMode service for switching the robot to backward-only motion.
"""

import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from skuba_interfaces.srv import DynamicReconfig
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from pathlib import Path
from std_srvs.srv import Trigger

NAMESPACE = 'skuba/cheesecake/nav'

# Controller server node name (adjust if your launch uses a different name)
CONTROLLER_SERVER_NODE = '/controller_server'


class CostmapReconfigServer(Node):
    """
    Simple costmap reconfiguration server that modifies YAML and applies to Nav2.
    Also supports controller server parameter changes for motion mode switching
    (e.g., backward-only mode via allow_reversing / min_vel_x / max_vel_x).
    """

    def __init__(self):
        super().__init__('costmap_reconfig_server')
        self.callback_group = ReentrantCallbackGroup()

        # File paths
        package_name = 'skuba_navigation'
        install_share_path = Path(get_package_share_directory(package_name))
        ws_root = install_share_path.parents[3]
        config_path = ws_root / 'src' / package_name / 'params' / 'costmap_dynamic_config.yaml'
        self.config_path = str(config_path)

        # Parameter mapping: simple name -> full Nav2 parameter path
        self.param_mapping = {
            'robot_radius': 'robot_radius',
            'resolution': 'resolution',
            'width': 'width',
            'height': 'height',
            'update_frequency': 'update_frequency',
            'publish_frequency': 'publish_frequency',
            'inflation_radius': 'inflation_layer.inflation_radius',
            'cost_scaling_factor': 'inflation_layer.cost_scaling_factor',
            'max_obstacle_height': 'voxel_layer.max_obstacle_height',
            'obstacle_max_range': 'voxel_layer.scan.obstacle_max_range',
            'obstacle_min_range': 'voxel_layer.scan.obstacle_min_range',
            'raytrace_max_range': 'voxel_layer.scan.raytrace_max_range'
        }

        # Controller parameter mapping: simple name -> full parameter path on /controller_server
        # These are FollowPath (DWB / RPP) parameters — adjust plugin prefix to match your config.
        # Common prefixes: "FollowPath" (RPP), "FollowPath.critics" (DWB).
        self.controller_param_mapping = {
            # Regulated Pure Pursuit (RPP) — used when you want backward motion
            'allow_reversing':         'FollowPath.allow_reversing',          # bool
            'min_vel_x':               'FollowPath.min_vel_x',                # double (negative = allow backward)
            'max_vel_x':               'FollowPath.max_vel_x',                # double
            'max_vel_x_backwards':     'FollowPath.max_vel_x_backwards',      # double (RPP only)
            'max_vel_theta':           'FollowPath.max_vel_theta',             # double
            'min_speed_xy':            'FollowPath.min_speed_xy',             # double
            'max_speed_xy':            'FollowPath.max_speed_xy',             # double
            # DWB extras (uncomment if you use DWB)
            # 'min_vel_x_dwb':         'FollowPath.min_vel_x',
            # 'max_vel_x_dwb':         'FollowPath.max_vel_x',
        }

        # Controller parameter types (needed to set the correct ParameterType)
        self.controller_param_types = {
            'allow_reversing':         ParameterType.PARAMETER_BOOL,
            'min_vel_x':               ParameterType.PARAMETER_DOUBLE,
            'max_vel_x':               ParameterType.PARAMETER_DOUBLE,
            'max_vel_x_backwards':     ParameterType.PARAMETER_DOUBLE,
            'max_vel_theta':           ParameterType.PARAMETER_DOUBLE,
            'min_speed_xy':            ParameterType.PARAMETER_DOUBLE,
            'max_speed_xy':            ParameterType.PARAMETER_DOUBLE,
        }

        # ── Costmap services ──────────────────────────────────────────────────
        self._reconfig_service = self.create_service(
            DynamicReconfig,
            f'{NAMESPACE}/costmap_reconfig',
            self.reconfig_callback,
            callback_group=self.callback_group
        )

        self._reset_service = self.create_service(
            Trigger,
            f'{NAMESPACE}/reset_costmap_config',
            self.reset_config_callback,
            callback_group=self.callback_group
        )

        # ── Controller services ───────────────────────────────────────────────
        # Generic controller parameter setter (mirrors DynamicReconfig interface)
        self._controller_reconfig_service = self.create_service(
            DynamicReconfig,
            f'{NAMESPACE}/controller_reconfig',
            self.controller_reconfig_callback,
            callback_group=self.callback_group
        )

        # Dedicated backward-only mode toggle  (Trigger: call → enable, /disable to disable)
        self._backward_mode_enable_service = self.create_service(
            Trigger,
            f'{NAMESPACE}/enable_backward_mode',
            self.enable_backward_mode_callback,
            callback_group=self.callback_group
        )

        self._backward_mode_disable_service = self.create_service(
            Trigger,
            f'{NAMESPACE}/disable_backward_mode',
            self.disable_backward_mode_callback,
            callback_group=self.callback_group
        )

        # ── ROS2 parameter clients ────────────────────────────────────────────
        self.param_clients = {
            'local':  self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters'),
            'global': self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters'),
        }

        self.controller_set_client  = self.create_client(
            SetParameters,
            f'{CONTROLLER_SERVER_NODE}/set_parameters'
        )
        self.controller_get_client  = self.create_client(
            GetParameters,
            f'{CONTROLLER_SERVER_NODE}/get_parameters'
        )

        # Snapshot of original controller params (populated lazily on first use)
        self._controller_original_snapshot: dict = {}

        # ── Misc setup ────────────────────────────────────────────────────────
        self.validation_rules = self.load_validation_rules()
        self.initialize_config_file()

        self.get_logger().info("Costmap Reconfig Server ready.")
        self.get_logger().info(f"Config file: {self.config_path}")
        self.get_logger().info(f"Costmap params:    {list(self.param_mapping.keys())}")
        self.get_logger().info(f"Controller params: {list(self.controller_param_mapping.keys())}")
        self.get_logger().info("Services available:")
        self.get_logger().info(f"  {NAMESPACE}/costmap_reconfig        (DynamicReconfig)")
        self.get_logger().info(f"  {NAMESPACE}/reset_costmap_config    (Trigger)")
        self.get_logger().info(f"  {NAMESPACE}/controller_reconfig     (DynamicReconfig)")
        self.get_logger().info(f"  {NAMESPACE}/enable_backward_mode    (Trigger)")
        self.get_logger().info(f"  {NAMESPACE}/disable_backward_mode   (Trigger)")

    # =========================================================================
    # Controller reconfig (generic)
    # =========================================================================

    def controller_reconfig_callback(self, request, response):
        """
        Generic controller parameter setter.
        request.costmap_type is ignored (pass anything, e.g. 'controller').
        request.parameter_name must be one of controller_param_mapping keys.
        request.value is the new value as string.
        """
        try:
            self.get_logger().info(
                f"Controller reconfig: {request.parameter_name} = {request.value}"
            )

            if request.parameter_name not in self.controller_param_mapping:
                response.success = False
                response.message = (
                    f"Parameter '{request.parameter_name}' not supported. "
                    f"Available: {list(self.controller_param_mapping.keys())}"
                )
                return response

            param_type = self.controller_param_types[request.parameter_name]
            converted = self._convert_value(request.value, param_type)
            if converted is None:
                response.success = False
                response.message = f"Cannot convert '{request.value}' for parameter '{request.parameter_name}'"
                return response

            full_name = self.controller_param_mapping[request.parameter_name]
            result = self._set_controller_param(full_name, converted, param_type)

            response.success = result['success']
            response.message = (
                f"Set controller {request.parameter_name} = {request.value}"
                if result['success']
                else result['error']
            )

        except Exception as e:
            self.get_logger().error(f"controller_reconfig_callback error: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"

        return response

    # =========================================================================
    # Backward-only mode
    # =========================================================================

    def enable_backward_mode_callback(self, request, response):
        """
        Enable backward-only motion:
          - allow_reversing  = True
          - max_vel_x        = 0.0   (no forward movement)
          - min_vel_x        = -0.3  (allow up to 0.3 m/s backward)
          - max_vel_x_backwards = 0.3

        Saves a snapshot of current values so disable can restore them.
        """
        try:
            self.get_logger().info("Enabling backward-only mode...")

            # Take snapshot before changing anything
            snapshot_ok = self._snapshot_controller_params(
                ['allow_reversing', 'max_vel_x', 'min_vel_x', 'max_vel_x_backwards']
            )
            if not snapshot_ok:
                self.get_logger().warn(
                    "Could not snapshot all controller params — disable_backward_mode "
                    "will use hardcoded defaults instead."
                )

            changes = [
                ('allow_reversing',     True,  ParameterType.PARAMETER_BOOL),
                ('max_vel_x',           0.0,   ParameterType.PARAMETER_DOUBLE),
                ('min_vel_x',          -0.7,   ParameterType.PARAMETER_DOUBLE),
                ('max_vel_x_backwards', 0.8,   ParameterType.PARAMETER_DOUBLE),
            ]

            failed = []
            for simple_name, value, ptype in changes:
                full_name = self.controller_param_mapping.get(simple_name)
                if full_name is None:
                    self.get_logger().warn(f"No mapping for '{simple_name}', skipping.")
                    continue
                result = self._set_controller_param(full_name, value, ptype)
                if not result['success']:
                    failed.append(f"{simple_name}: {result['error']}")

            if failed:
                response.success = False
                response.message = "Backward mode partially failed: " + "; ".join(failed)
            else:
                response.success = True
                response.message = (
                    "Backward-only mode ENABLED. "
                    "Robot will only move backward (max 0.3 m/s). "
                    "Call disable_backward_mode to restore."
                )
                self.get_logger().info("Backward-only mode ENABLED.")

        except Exception as e:
            self.get_logger().error(f"enable_backward_mode error: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"

        return response

    def disable_backward_mode_callback(self, request, response):
        """
        Restore normal (forward+backward) motion by reverting the snapshot
        taken when backward mode was enabled, or falling back to safe defaults.
        """
        try:
            self.get_logger().info("Disabling backward mode, restoring normal motion...")

            # Use snapshot if available, otherwise safe defaults
            defaults = {
                'allow_reversing':     (False, ParameterType.PARAMETER_BOOL),
                'max_vel_x':           (0.5,   ParameterType.PARAMETER_DOUBLE),
                'min_vel_x':           (0.0,   ParameterType.PARAMETER_DOUBLE),
                'max_vel_x_backwards': (0.3,   ParameterType.PARAMETER_DOUBLE),
            }

            failed = []
            for simple_name, (default_val, ptype) in defaults.items():
                full_name = self.controller_param_mapping.get(simple_name)
                if full_name is None:
                    continue

                # Prefer snapshot value
                value = self._controller_original_snapshot.get(simple_name, default_val)
                result = self._set_controller_param(full_name, value, ptype)
                if not result['success']:
                    failed.append(f"{simple_name}: {result['error']}")

            # Clear snapshot
            self._controller_original_snapshot.clear()

            if failed:
                response.success = False
                response.message = "Restore partially failed: " + "; ".join(failed)
            else:
                response.success = True
                response.message = "Normal motion mode RESTORED."
                self.get_logger().info("Backward mode DISABLED — normal motion restored.")

        except Exception as e:
            self.get_logger().error(f"disable_backward_mode error: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"

        return response

    # =========================================================================
    # Internal helpers — controller
    # =========================================================================

    def _convert_value(self, value_str, param_type):
        """Convert string value to the correct Python type for a ParameterType."""
        try:
            if param_type == ParameterType.PARAMETER_BOOL:
                if isinstance(value_str, bool):
                    return value_str
                return value_str.strip().lower() in ('true', '1', 'yes')
            elif param_type == ParameterType.PARAMETER_DOUBLE:
                return float(value_str)
            elif param_type == ParameterType.PARAMETER_INTEGER:
                return int(value_str)
            else:
                return str(value_str)
        except Exception:
            return None

    def _set_controller_param(self, full_param_name, value, param_type):
        """
        Send a single parameter to /controller_server via SetParameters.
        Returns dict with 'success' bool and optional 'error' string.
        """
        try:
            if not self.controller_set_client.wait_for_service(timeout_sec=2.0):
                return {
                    'success': False,
                    'error': f"{CONTROLLER_SERVER_NODE}/set_parameters not available"
                }

            param = Parameter()
            param.name = full_param_name
            param.value = ParameterValue()
            param.value.type = param_type

            if param_type == ParameterType.PARAMETER_BOOL:
                param.value.bool_value = bool(value)
            elif param_type == ParameterType.PARAMETER_DOUBLE:
                param.value.double_value = float(value)
            elif param_type == ParameterType.PARAMETER_INTEGER:
                param.value.integer_value = int(value)
            else:
                param.value.string_value = str(value)

            req = SetParameters.Request()
            req.parameters = [param]

            future = self.controller_set_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                resp = future.result()
                if resp and resp.results and resp.results[0].successful:
                    self.get_logger().info(f"Controller param set: {full_param_name} = {value}")
                    return {'success': True}
                else:
                    reason = (
                        resp.results[0].reason
                        if resp and resp.results
                        else "Unknown error"
                    )
                    return {'success': False, 'error': reason}
            else:
                return {'success': False, 'error': "SetParameters call timed out"}

        except Exception as e:
            return {'success': False, 'error': str(e)}

    def _snapshot_controller_params(self, simple_names: list) -> bool:
        """
        Fetch current values of the listed parameters from /controller_server
        and store them in self._controller_original_snapshot.
        Returns True if all params were fetched successfully.
        """
        try:
            if not self.controller_get_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("GetParameters service not available, skipping snapshot.")
                return False

            full_names = [
                self.controller_param_mapping[n]
                for n in simple_names
                if n in self.controller_param_mapping
            ]

            req = GetParameters.Request()
            req.names = full_names

            future = self.controller_get_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if not future.done():
                return False

            resp = future.result()
            if resp is None:
                return False

            for simple_name, pval in zip(simple_names, resp.values):
                ptype = self.controller_param_types.get(simple_name)
                if ptype == ParameterType.PARAMETER_BOOL:
                    self._controller_original_snapshot[simple_name] = pval.bool_value
                elif ptype == ParameterType.PARAMETER_DOUBLE:
                    self._controller_original_snapshot[simple_name] = pval.double_value
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    self._controller_original_snapshot[simple_name] = pval.integer_value
                else:
                    self._controller_original_snapshot[simple_name] = pval.string_value

            self.get_logger().info(f"Snapshot saved: {self._controller_original_snapshot}")
            return True

        except Exception as e:
            self.get_logger().error(f"Snapshot error: {e}")
            return False

    # =========================================================================
    # Original costmap logic (unchanged)
    # =========================================================================

    def load_validation_rules(self):
        rules = {
            'robot_radius':       {'min': 0.05, 'max': 0.5},
            'resolution':         {'min': 0.01, 'max': 0.1},
            'width':              {'min': 2,    'max': 10},
            'height':             {'min': 2,    'max': 10},
            'update_frequency':   {'min': 1.0,  'max': 20.0},
            'publish_frequency':  {'min': 0.5,  'max': 10.0},
            'inflation_radius':   {'min': 0.1,  'max': 1.0},
            'cost_scaling_factor':{'min': 1.0,  'max': 10.0},
            'max_obstacle_height':{'min': 0.5,  'max': 5.0},
            'obstacle_max_range': {'min': 1.0,  'max': 5.0},
            'obstacle_min_range': {'min': 0.1,  'max': 1.0},
            'raytrace_max_range': {'min': 1.0,  'max': 8.0},
        }
        self.get_logger().info(f"Loaded {len(rules)} validation rules")
        return rules

    def reconfig_callback(self, request, response):
        try:
            self.get_logger().info(
                f"Reconfig request: {request.costmap_type}.{request.parameter_name} = {request.value}"
            )

            if request.costmap_type not in ['local', 'global']:
                response.success = False
                response.message = f"Invalid costmap_type: {request.costmap_type}. Must be 'local' or 'global'"
                return response

            if request.parameter_name not in self.param_mapping:
                response.success = False
                response.message = (
                    f"Parameter '{request.parameter_name}' not supported. "
                    f"Available: {list(self.param_mapping.keys())}"
                )
                return response

            validation_result = self.validate_parameter(request.parameter_name, request.value)
            if not validation_result['valid']:
                response.success = False
                response.message = validation_result['error']
                return response

            update_result = self.update_yaml_config(
                request.costmap_type,
                request.parameter_name,
                validation_result['converted_value']
            )

            if not update_result['success']:
                response.success = False
                response.message = update_result['error']
                return response

            nav2_param_name = self.param_mapping[request.parameter_name]
            apply_result = self.apply_to_nav2(
                request.costmap_type,
                nav2_param_name,
                validation_result['converted_value']
            )

            if not apply_result['success']:
                self.update_yaml_config(
                    request.costmap_type,
                    request.parameter_name,
                    update_result['old_value']
                )
                response.success = False
                response.message = f"Failed to apply to Nav2: {apply_result['error']}"
                return response

            response.success = True
            response.message = (
                f"Successfully updated {request.costmap_type}_costmap."
                f"{request.parameter_name} = {request.value}"
            )

        except Exception as e:
            self.get_logger().error(f"Error in reconfig_callback: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"

        return response

    def validate_parameter(self, param_name, value_str):
        try:
            converted_value = float(value_str)
            if param_name in self.validation_rules:
                rule = self.validation_rules[param_name]
                if 'min' in rule and converted_value < rule['min']:
                    return {'valid': False, 'error': f"Value {converted_value} below minimum {rule['min']}"}
                if 'max' in rule and converted_value > rule['max']:
                    return {'valid': False, 'error': f"Value {converted_value} above maximum {rule['max']}"}
            return {'valid': True, 'converted_value': converted_value}
        except ValueError:
            return {'valid': False, 'error': f"Cannot convert '{value_str}' to number"}

    def update_yaml_config(self, costmap_type, param_name, new_value):
        try:
            with open(self.config_path, 'r') as f:
                config = yaml.safe_load(f)

            old_value = None
            costmap_key = f"{costmap_type}_costmap"

            if costmap_key in config:
                costmap_config = config[costmap_key][costmap_key]['ros__parameters']

                if param_name in costmap_config:
                    old_value = costmap_config[param_name]
                    costmap_config[param_name] = new_value
                else:
                    found = False
                    for section_name, section_config in costmap_config.items():
                        if isinstance(section_config, dict) and param_name in section_config:
                            old_value = section_config[param_name]
                            section_config[param_name] = new_value
                            found = True
                            break
                        if isinstance(section_config, dict):
                            for nested_name, nested_config in section_config.items():
                                if isinstance(nested_config, dict) and param_name in nested_config:
                                    old_value = nested_config[param_name]
                                    nested_config[param_name] = new_value
                                    found = True
                                    break
                            if found:
                                break
                    if not found:
                        return {'success': False, 'error': f"Parameter '{param_name}' not found in {costmap_type}_costmap"}
            else:
                return {'success': False, 'error': f"Costmap '{costmap_type}' not found in config"}

            with open(self.config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, indent=2)

            self.get_logger().info(f"Updated YAML: {param_name} from {old_value} to {new_value}")
            return {'success': True, 'old_value': old_value}

        except Exception as e:
            return {'success': False, 'error': f"Failed to update YAML: {str(e)}"}

    def apply_to_nav2(self, costmap_type, param_name, new_value):
        try:
            client = self.param_clients[costmap_type]
            if not client.wait_for_service(timeout_sec=2.0):
                return {'success': False, 'error': f"{costmap_type}_costmap parameter service not available"}

            param = Parameter()
            param.name = param_name
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = float(new_value)

            request = SetParameters.Request()
            request.parameters = [param]

            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if future.done():
                response = future.result()
                if response and len(response.results) > 0 and response.results[0].successful:
                    self.get_logger().info(f"Applied to Nav2: {costmap_type}.{param_name} = {new_value}")
                    return {'success': True}
                else:
                    reason = response.results[0].reason if response and response.results else "Unknown error"
                    return {'success': False, 'error': f"Nav2 parameter update failed: {reason}"}
            else:
                return {'success': False, 'error': "Nav2 parameter service call timeout"}

        except Exception as e:
            return {'success': False, 'error': f"Nav2 apply error: {str(e)}"}

    def reset_config_callback(self, request, response):
        try:
            backup_result = self.create_backup_config()
            if backup_result['success']:
                self.get_logger().info(f"Created backup: {backup_result['backup_file']}")

            original_config = self.get_original_config()
            with open(self.config_path, 'w') as f:
                yaml.dump(original_config, f, default_flow_style=False, indent=2)

            response.success = True
            response.message = "Config reset to original values. Restart nodes to apply changes."
            self.get_logger().info("Costmap config reset to original values")

        except Exception as e:
            response.success = False
            response.message = f"Failed to reset config: {str(e)}"
            self.get_logger().error(f"Error resetting config: {e}")

        return response

    def create_backup_config(self):
        try:
            import time
            timestamp = int(time.time())
            backup_file = self.config_path.replace('.yaml', f'_backup_{timestamp}.yaml')
            with open(self.config_path, 'r') as src:
                content = src.read()
            with open(backup_file, 'w') as dst:
                dst.write(content)
            return {'success': True, 'backup_file': backup_file}
        except Exception as e:
            return {'success': False, 'error': str(e)}

    def get_original_config(self):
        return {
            'local_costmap': {
                'local_costmap': {
                    'ros__parameters': {
                        'robot_radius': 0.1,
                        'resolution': 0.05,
                        'width': 3,
                        'height': 3,
                        'update_frequency': 5.0,
                        'publish_frequency': 2.0,
                        'inflation_layer': {
                            'plugin': "nav2_costmap_2d::InflationLayer",
                            'cost_scaling_factor': 2.0,
                            'inflation_radius': 0.3
                        },
                        'voxel_layer': {
                            'plugin': "nav2_costmap_2d::VoxelLayer",
                            'enabled': True,
                            'max_obstacle_height': 2.0,
                            'scan': {
                                'obstacle_max_range': 2.5,
                                'obstacle_min_range': 0.3,
                                'raytrace_max_range': 3.0
                            }
                        }
                    }
                }
            },
            'global_costmap': {
                'global_costmap': {
                    'ros__parameters': {
                        'robot_radius': 0.10,
                        'resolution': 0.05,
                        'update_frequency': 1.0,
                        'publish_frequency': 1.0,
                        'track_unknown_space': True,
                        'inflation_layer': {
                            'plugin': "nav2_costmap_2d::InflationLayer",
                            'cost_scaling_factor': 3.0,
                            'inflation_radius': 0.35
                        },
                        'obstacle_layer': {
                            'plugin': "nav2_costmap_2d::ObstacleLayer",
                            'enabled': True,
                            'scan': {
                                'obstacle_max_range': 2.5,
                                'obstacle_min_range': 0.3,
                                'raytrace_max_range': 3.0
                            }
                        }
                    }
                }
            }
        }

    def initialize_config_file(self):
        try:
            original_config = self.get_original_config()
            with open(self.config_path, 'w') as f:
                yaml.dump(original_config, f, default_flow_style=False, indent=2)
            self.get_logger().info("Config file initialized with original values")
        except Exception as e:
            self.get_logger().error(f"Error initializing config file: {str(e)}")


def main():
    rclpy.init()
    node = CostmapReconfigServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()