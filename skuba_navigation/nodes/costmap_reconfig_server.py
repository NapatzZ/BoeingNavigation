#!/usr/bin/env python3
"""
Costmap Dynamic Reconfiguration Server for SKUBA robot.

This node provides a simple service to dynamically modify costmap parameters
by updating YAML configuration and reloading to Nav2.

Also supports controller server parameter changes, including a dedicated
SetControllerMode service for switching the robot to backward-only motion.

Pure validation/YAML/dict logic lives in costmap_config.py; this file only
wires up services and the rclpy parameter-client calls that logic needs.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from ament_index_python.packages import get_package_share_directory
from skuba_interfaces.srv import DynamicReconfig
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from pathlib import Path
from std_srvs.srv import Trigger

from ..core import costmap_config as cc
from ..core.package_paths import find_package_source_dir

NAMESPACE = "skuba/navigation"

# Controller server node name (adjust if your launch uses a different name)
CONTROLLER_SERVER_NODE = "/controller_server"


class CostmapReconfigServer(Node):
    """
    Simple costmap reconfiguration server that modifies YAML and applies to Nav2.

    Also supports controller server parameter changes for motion mode switching
    (e.g., backward-only mode via allow_reversing / min_vel_x / max_vel_x).
    """

    def __init__(self):
        super().__init__("costmap_reconfig_server")
        self.callback_group = ReentrantCallbackGroup()

        # File paths
        package_name = "skuba_navigation"
        install_share_path = Path(get_package_share_directory(package_name))
        source_dir = find_package_source_dir(install_share_path, package_name)
        config_path = (
            (source_dir or install_share_path)
            / "params"
            / "costmap_dynamic_config.yaml"
        )
        self.config_path = str(config_path)

        # ── Costmap services ──────────────────────────────────────────────────
        self._reconfig_service = self.create_service(
            DynamicReconfig,
            f"/{NAMESPACE}/costmap_reconfig",
            self.reconfig_callback,
            callback_group=self.callback_group,
        )

        self._reset_service = self.create_service(
            Trigger,
            f"/{NAMESPACE}/reset_costmap_config",
            self.reset_config_callback,
            callback_group=self.callback_group,
        )

        # ── Controller services ───────────────────────────────────────────────
        self._controller_reconfig_service = self.create_service(
            DynamicReconfig,
            f"/{NAMESPACE}/controller_reconfig",
            self.controller_reconfig_callback,
            callback_group=self.callback_group,
        )

        self._backward_mode_enable_service = self.create_service(
            Trigger,
            f"/{NAMESPACE}/enable_backward_mode",
            self.enable_backward_mode_callback,
            callback_group=self.callback_group,
        )

        self._backward_mode_disable_service = self.create_service(
            Trigger,
            f"/{NAMESPACE}/disable_backward_mode",
            self.disable_backward_mode_callback,
            callback_group=self.callback_group,
        )

        # ── ROS2 parameter clients ────────────────────────────────────────────
        self.param_clients = {
            "local": self.create_client(
                SetParameters, "/local_costmap/local_costmap/set_parameters"
            ),
            "global": self.create_client(
                SetParameters, "/global_costmap/global_costmap/set_parameters"
            ),
        }

        self.controller_set_client = self.create_client(
            SetParameters, f"{CONTROLLER_SERVER_NODE}/set_parameters"
        )
        self.controller_get_client = self.create_client(
            GetParameters, f"{CONTROLLER_SERVER_NODE}/get_parameters"
        )

        # Snapshot of original controller params (populated lazily on first use)
        self._controller_original_snapshot: dict = {}

        # ── Misc setup ────────────────────────────────────────────────────────
        self.validation_rules = cc.load_validation_rules()
        self.get_logger().info(f"Loaded {len(self.validation_rules)} validation rules")
        self.initialize_config_file()

        self.get_logger().info("Costmap Reconfig Server ready.")
        self.get_logger().info(f"Config file: {self.config_path}")
        self.get_logger().info(f"Costmap params:    {list(cc.PARAM_MAPPING.keys())}")
        self.get_logger().info(
            f"Controller params: {list(cc.CONTROLLER_PARAM_MAPPING.keys())}"
        )
        self.get_logger().info("Services available:")
        self.get_logger().info(
            f"  /{NAMESPACE}/costmap_reconfig        (DynamicReconfig)"
        )
        self.get_logger().info(f"  /{NAMESPACE}/reset_costmap_config    (Trigger)")
        self.get_logger().info(
            f"  /{NAMESPACE}/controller_reconfig     (DynamicReconfig)"
        )
        self.get_logger().info(f"  /{NAMESPACE}/enable_backward_mode    (Trigger)")
        self.get_logger().info(f"  /{NAMESPACE}/disable_backward_mode   (Trigger)")

    # =========================================================================
    # Controller reconfig (generic)
    # =========================================================================

    def controller_reconfig_callback(self, request, response):
        try:
            self.get_logger().info(
                f"Controller reconfig: {request.parameter_name} = {request.value}"
            )

            if request.parameter_name not in cc.CONTROLLER_PARAM_MAPPING:
                response.success = False
                response.message = (
                    f"Parameter '{request.parameter_name}' not supported. "
                    f"Available: {list(cc.CONTROLLER_PARAM_MAPPING.keys())}"
                )
                return response

            param_type = cc.CONTROLLER_PARAM_TYPES[request.parameter_name]
            converted = cc.convert_value(request.value, param_type)
            if converted is None:
                response.success = False
                response.message = (
                    f"Cannot convert '{request.value}' for parameter "
                    f"'{request.parameter_name}'"
                )
                return response

            full_name = cc.CONTROLLER_PARAM_MAPPING[request.parameter_name]
            result = self._set_controller_param(full_name, converted, param_type)

            response.success = result["success"]
            response.message = (
                f"Set controller {request.parameter_name} = {request.value}"
                if result["success"]
                else result["error"]
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
        try:
            self.get_logger().info("Enabling backward-only mode...")

            snapshot_ok = self._snapshot_controller_params(
                ["allow_reversing", "max_vel_x", "min_vel_x", "max_vel_x_backwards"]
            )
            if not snapshot_ok:
                self.get_logger().warn(
                    "Could not snapshot all controller params — disable_backward_mode "
                    "will use hardcoded defaults instead."
                )

            failed = []
            for simple_name, value, ptype in cc.BACKWARD_MODE_ENABLE_CHANGES:
                full_name = cc.CONTROLLER_PARAM_MAPPING.get(simple_name)
                if full_name is None:
                    self.get_logger().warn(f"No mapping for '{simple_name}', skipping.")
                    continue
                result = self._set_controller_param(full_name, value, ptype)
                if not result["success"]:
                    failed.append(f"{simple_name}: {result['error']}")

            if failed:
                response.success = False
                response.message = "Backward mode partially failed: " + "; ".join(
                    failed
                )
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
        try:
            self.get_logger().info(
                "Disabling backward mode, restoring normal motion..."
            )

            failed = []
            for simple_name, (default_val, ptype) in cc.BACKWARD_MODE_DEFAULTS.items():
                full_name = cc.CONTROLLER_PARAM_MAPPING.get(simple_name)
                if full_name is None:
                    continue

                value = self._controller_original_snapshot.get(simple_name, default_val)
                result = self._set_controller_param(full_name, value, ptype)
                if not result["success"]:
                    failed.append(f"{simple_name}: {result['error']}")

            self._controller_original_snapshot.clear()

            if failed:
                response.success = False
                response.message = "Restore partially failed: " + "; ".join(failed)
            else:
                response.success = True
                response.message = "Normal motion mode RESTORED."
                self.get_logger().info(
                    "Backward mode DISABLED — normal motion restored."
                )

        except Exception as e:
            self.get_logger().error(f"disable_backward_mode error: {e}")
            response.success = False
            response.message = f"Internal error: {str(e)}"

        return response

    # =========================================================================
    # Internal helpers — controller (need rclpy service clients)
    # =========================================================================

    def _set_controller_param(self, full_param_name, value, param_type):
        """
        Send a single parameter to /controller_server via SetParameters.

        Returns dict with 'success' bool and optional 'error' string.
        """
        try:
            if not self.controller_set_client.wait_for_service(timeout_sec=2.0):
                return {
                    "success": False,
                    "error": f"{CONTROLLER_SERVER_NODE}/set_parameters not available",
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
                    self.get_logger().info(
                        f"Controller param set: {full_param_name} = {value}"
                    )
                    return {"success": True}
                else:
                    reason = (
                        resp.results[0].reason
                        if resp and resp.results
                        else "Unknown error"
                    )
                    return {"success": False, "error": reason}
            else:
                return {"success": False, "error": "SetParameters call timed out"}

        except Exception as e:
            return {"success": False, "error": str(e)}

    def _snapshot_controller_params(self, simple_names: list) -> bool:
        """
        Snapshot the listed /controller_server parameters.

        Fetches their current values into self._controller_original_snapshot.
        Returns True if all params were fetched successfully.
        """
        try:
            if not self.controller_get_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    "GetParameters service not available, skipping snapshot."
                )
                return False

            full_names = [
                cc.CONTROLLER_PARAM_MAPPING[n]
                for n in simple_names
                if n in cc.CONTROLLER_PARAM_MAPPING
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
                ptype = cc.CONTROLLER_PARAM_TYPES.get(simple_name)
                if ptype == ParameterType.PARAMETER_BOOL:
                    self._controller_original_snapshot[simple_name] = pval.bool_value
                elif ptype == ParameterType.PARAMETER_DOUBLE:
                    self._controller_original_snapshot[simple_name] = pval.double_value
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    self._controller_original_snapshot[simple_name] = pval.integer_value
                else:
                    self._controller_original_snapshot[simple_name] = pval.string_value

            self.get_logger().info(
                f"Snapshot saved: {self._controller_original_snapshot}"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Snapshot error: {e}")
            return False

    # =========================================================================
    # Costmap reconfig (delegates validation/YAML to costmap_config)
    # =========================================================================

    def reconfig_callback(self, request, response):
        try:
            self.get_logger().info(
                f"Reconfig request: {request.costmap_type}.{request.parameter_name} = "
                f"{request.value}"
            )

            if request.costmap_type not in ["local", "global"]:
                response.success = False
                response.message = (
                    f"Invalid costmap_type: {request.costmap_type}. Must be 'local' or "
                    "'global'"
                )
                return response

            if request.parameter_name not in cc.PARAM_MAPPING:
                response.success = False
                response.message = (
                    f"Parameter '{request.parameter_name}' not supported. "
                    f"Available: {list(cc.PARAM_MAPPING.keys())}"
                )
                return response

            validation_result = cc.validate_parameter(
                request.parameter_name, request.value, self.validation_rules
            )
            if not validation_result["valid"]:
                response.success = False
                response.message = validation_result["error"]
                return response

            update_result = cc.update_yaml_config(
                self.config_path,
                request.costmap_type,
                request.parameter_name,
                validation_result["converted_value"],
            )

            if not update_result["success"]:
                response.success = False
                response.message = update_result["error"]
                return response

            nav2_param_name = cc.PARAM_MAPPING[request.parameter_name]
            apply_result = self.apply_to_nav2(
                request.costmap_type,
                nav2_param_name,
                validation_result["converted_value"],
            )

            if not apply_result["success"]:
                cc.update_yaml_config(
                    self.config_path,
                    request.costmap_type,
                    request.parameter_name,
                    update_result["old_value"],
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

    def apply_to_nav2(self, costmap_type, param_name, new_value):
        try:
            client = self.param_clients[costmap_type]
            if not client.wait_for_service(timeout_sec=2.0):
                return {
                    "success": False,
                    "error": f"{costmap_type}_costmap parameter service not available",
                }

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
                if (
                    response
                    and len(response.results) > 0
                    and response.results[0].successful
                ):
                    self.get_logger().info(
                        f"Applied to Nav2: {costmap_type}.{param_name} = {new_value}"
                    )
                    return {"success": True}
                else:
                    reason = (
                        response.results[0].reason
                        if response and response.results
                        else "Unknown error"
                    )
                    return {
                        "success": False,
                        "error": f"Nav2 parameter update failed: {reason}",
                    }
            else:
                return {
                    "success": False,
                    "error": "Nav2 parameter service call timeout",
                }

        except Exception as e:
            return {"success": False, "error": f"Nav2 apply error: {str(e)}"}

    def reset_config_callback(self, request, response):
        try:
            backup_result = cc.create_backup_config(self.config_path)
            if backup_result["success"]:
                self.get_logger().info(
                    f"Created backup: {backup_result['backup_file']}"
                )

            cc.write_config(self.config_path, cc.get_original_config())

            response.success = True
            response.message = (
                "Config reset to original values. Restart nodes to apply changes."
            )
            self.get_logger().info("Costmap config reset to original values")

        except Exception as e:
            response.success = False
            response.message = f"Failed to reset config: {str(e)}"
            self.get_logger().error(f"Error resetting config: {e}")

        return response

    def initialize_config_file(self):
        """
        Seed the dynamic config file with defaults, never over an existing one.

        Otherwise every node restart would silently discard
        whatever an operator has tuned since the last colcon build.
        """
        if os.path.exists(self.config_path):
            self.get_logger().info(f"Using existing config file: {self.config_path}")
            return
        try:
            cc.write_config(self.config_path, cc.get_original_config())
            self.get_logger().info(
                "Config file did not exist — initialized with original values"
            )
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


if __name__ == "__main__":
    main()
