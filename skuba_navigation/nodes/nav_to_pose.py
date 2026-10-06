#!/usr/bin/env python3
"""
Navigation server for a SKUBA robot using ROS 2 Humble.
This module provides a comprehensive navigation solution for a SKUBA robot,
allowing it to navigate to saved positions or specific coordinates, save current
locations, and perform basic movement operations like rotation and linear movement.
It integrates with Nav2 for path planning and navigation execution.
"""

import os
import time
import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from skuba_interfaces.srv import (
    NavToPose,
    NavToName,
    SaveLocationCommand,
    Rotate,
    MoveWheel,
    NavToTF,
    Nav2ToTF,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformListener, Buffer
from pathlib import Path
from ..core.geometry_utils import (
    AUTO_SIDES,
    SIDE_OFFSETS,
    approach_pose_for_side,
    normalize_angle,
    quaternion_to_yaw,
)
from ..core import nav_to_pose_logic as logic
from ..core.package_paths import find_package_source_dir

NAMESPACE = "skuba/navigation"
package_name = "skuba_navigation"
install_share_path = Path(get_package_share_directory(package_name))
source_dir = find_package_source_dir(install_share_path, package_name)
pose_path = (
    source_dir / "share" if source_dir is not None else install_share_path
) / "saved_pose.yaml"
yaml_path = str(pose_path)


class NavigateToGoalServer(Node):
    """
    A ROS 2 node that provides navigation services for a SKUBA robot.
    This class implements services for navigating to saved positions,
    direct coordinates, saving positions, and basic movement control.
    It integrates with Nav2 for path planning and execution.
    """

    def __init__(self):
        """Initialize the navigation server node with all required services and publishers."""
        super().__init__("navigate_to_goal_server")
        self.callback_group = ReentrantCallbackGroup()

        # Initialize services
        self._init_services()

        # Velocity publisher for the manual point-turn/straight-line moves
        # below (rotate / move_forward / nudge_to_tf) — these bypass Nav2's
        # planner entirely, so they get their own twist_mux lane
        # (cmd_vel_nav_manual, priority 10) rather than sharing cmd_vel_nav
        # with collision_monitor's filtered Nav2 output. See
        # params/twist_mux.yaml.
        self.cmd_vel_manual_publisher = self.create_publisher(
            Twist, "cmd_vel_nav_manual", 10
        )

        # Initialize TF listener for getting robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize navigator
        self.navigator = None
        self.init_navigator()
        self.cancle = None

        self.saved_positions = self.load_saved_positions()
        self.get_logger().info("NavigateToGoal Action Server is ready.")

    def _init_services(self):
        """Initialize all ROS 2 services provided by this node."""
        # Service to cancel navigation
        self._cancel_service = self.create_service(
            Trigger,
            f"/{NAMESPACE}/cancel_goal",
            self.cancel_goal_callback,
            callback_group=self.callback_group,
        )

        # Service to clear costmaps
        self._clear_costmap_service = self.create_service(
            Trigger,
            f"/{NAMESPACE}/clear_costmap",
            self.clear_costmap_callback,
            callback_group=self.callback_group,
        )

        # Service to save the current position
        self._save_position_service = self.create_service(
            SaveLocationCommand,
            f"/{NAMESPACE}/save_pose",
            self.save_pose_callback,
            callback_group=self.callback_group,
        )

        # Service to navigate using a named position (YAML)
        self._named_goal_service = self.create_service(
            NavToName,
            f"/{NAMESPACE}/nav_to_name",
            self.set_named_goal_callback,
            callback_group=self.callback_group,
        )

        # Service to navigate using a direct PoseStamped
        self._pose_goal_service = self.create_service(
            NavToPose,
            f"/{NAMESPACE}/nav_to_position",
            self.set_pose_goal_callback,
            callback_group=self.callback_group,
        )

        # Service to reload the YAML file
        self._reload_yaml_service = self.create_service(
            Trigger,
            f"/{NAMESPACE}/reload_yaml",
            self.reload_yaml_callback,
            callback_group=self.callback_group,
        )

        # Service to rotate robot
        self.rotate_service = self.create_service(
            Rotate,
            f"/{NAMESPACE}/rotate",
            self.rotate_callback,
            callback_group=self.callback_group,
        )

        # Service to move forward/backward a straight-line distance
        self._move_forward_service = self.create_service(
            MoveWheel,
            f"/{NAMESPACE}/move_forward",
            self.move_forward_callback,
            callback_group=self.callback_group,
        )

        # Service to nudge towards a TF frame via raw cmd_vel (no Nav2 planning)
        self._nudge_to_tf_service = self.create_service(
            NavToTF,
            f"/{NAMESPACE}/nudge_to_tf",
            self.nudge_to_tf_callback,
            callback_group=self.callback_group,
        )

        # Service to navigate to TF using Nav2 Simple Commander (stop at stop_distance)
        self._nav_to_tf_service = self.create_service(
            Nav2ToTF,
            f"/{NAMESPACE}/nav_to_tf",
            self.nav_to_tf_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Waiting for services to be ready...")
        for _ in range(5):
            # Wall-clock sleep: with use_sim_time the node clock only advances
            # when /clock is processed, and nothing spins this node yet, so
            # get_clock().sleep_for() would block forever in simulation.
            time.sleep(0.2)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Services initialized and should be available now")

    def init_navigator(self):
        """
        Initialize the navigator without waiting for Nav2.
        Creates a BasicNavigator instance without waiting for Nav2 to become active.
        """
        self.navigator = BasicNavigator()
        self.get_logger().info(
            "BasicNavigator initialized - assuming Nav2 is already active."
        )

    def load_saved_positions(self):
        """
        Load named positions from a YAML file.
        Returns:
            dict: Dictionary of named positions as PoseStamped messages
        """
        try:
            self.get_logger().info(f"Loading saved positions from: {yaml_path}")
            positions = logic.load_saved_positions(yaml_path)
            self.get_logger().info(
                f"Loaded {len(positions)} named positions: {', '.join(positions.keys())}"
            )
            return positions
        except Exception as e:
            self.get_logger().error(f"Failed to load saved positions: {e}")
            return {}

    def set_named_goal_callback(self, request, response):
        """
        Service callback to navigate to a named position.
        Args:
            request: NavToName request containing position name and timeout
            response: Service response
        Returns:
            response: Updated service response with success status
        """
        try:
            # Reload YAML file to get latest positions
            self.get_logger().info("Reloading YAML file to get latest positions...")
            self.saved_positions = self.load_saved_positions()

            if request.pose.strip() in self.saved_positions:
                goal_pose = self.saved_positions[request.pose.strip()]
                self.get_logger().info(f"Using saved position '{request.pose.strip()}'")
                return self.navigate_to_goal(goal_pose, request.timeout, response)
            else:
                response.success = False
                self.get_logger().warning(f"Position '{request.pose}' not found.")
                return response
        except Exception as e:
            self.get_logger().error(f"Error in set_named_goal_callback: {e}")
            response.success = False
            return response

    def set_pose_goal_callback(self, request, response):
        """
        Service callback to navigate to a specific pose.
        Args:
            request: NavToPose request containing pose and timeout
            response: Service response
        Returns:
            response: Updated service response with success status
        """
        try:
            self.get_logger().info(f"Navigating to provided pose: {request.pose}")
            return self.navigate_to_goal(request.pose, request.timeout, response)
        except Exception as e:
            self.get_logger().error(f"Error in set_pose_goal_callback: {e}")
            response.success = False
            return response

    def navigate_to_goal(self, goal_pose, timeout, response):
        """
        Navigate to the specified goal with retry mechanism.
        Args:
            goal_pose: PoseStamped target position
            timeout: Time limit for navigation in seconds
            response: Service response to update
        Returns:
            response: Updated service response with success status
        """
        try:
            self.get_logger().info("Received a new navigation goal!")
            self.get_logger().info(
                f"Goal position: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
            )

            try:
                self.navigator.clearAllCostmaps()
                self.get_logger().debug("Successfully cleared all costmaps")
            except Exception as e:
                self.get_logger().warning(f"Error clearing costmaps: {e}")
                try:
                    self.clear_costmaps_humble()
                except Exception as e2:
                    self.get_logger().error(
                        f"Failed to clear costmaps in any way: {e2}"
                    )
                    # Continue anyway - not critical

            retry_count = 0
            success = False

            while retry_count <= 2:
                self.get_logger().info(f"Navigation attempt {retry_count + 1}/3")
                goal_pose.header.stamp = self.get_clock().now().to_msg()

                try:
                    self.navigator.goToPose(goal_pose)
                    self.get_logger().info("Successfully sent goal to navigator")
                except Exception as e:
                    self.get_logger().error(f"Failed to send goal to navigator: {e}")
                    retry_count += 1
                    time.sleep(1.0)
                    continue

                start_time = time.time()
                while rclpy.ok():
                    try:
                        feedback = self.navigator.getFeedback()
                        if feedback:
                            # Log feedback occasionally
                            elapsed = time.time() - start_time
                            if int(elapsed) % 5 == 0:  # Log every 5 seconds
                                self.get_logger().info(
                                    f"Navigation in progress: {elapsed:.1f}s elapsed"
                                )

                        if time.time() - start_time > timeout:
                            self.get_logger().info(
                                f"Timeout reached after {timeout}s! Retrying ({retry_count + 1})..."
                            )
                            try:
                                self.navigator.cancelTask()
                                self.cancle = True
                            except Exception as e:
                                self.get_logger().error(f"Error canceling task: {e}")
                            retry_count += 1
                            break
                    except Exception as e:
                        self.get_logger().warning(f"Error getting feedback: {e}")

                    # Check if task is complete
                    try:
                        if self.navigator.isTaskComplete():
                            self.get_logger().info("Task reported as complete")
                            break
                    except Exception as e:
                        self.get_logger().error(f"Error checking task completion: {e}")
                        time.sleep(1.0)

                    # Allow other callbacks to process
                    time.sleep(0.1)

                # Get the navigation result - only trust Nav2's decision
                try:
                    result = self.navigator.getResult()
                    self.get_logger().info(f"Navigation result: {result}")

                    if result == TaskResult.SUCCEEDED:
                        # Double-check with Nav2's task completion status
                        try:
                            task_complete = self.navigator.isTaskComplete()
                            self.get_logger().info(
                                f"Nav2 task complete status: {task_complete}"
                            )

                            if task_complete:
                                self.get_logger().info(
                                    "Nav2 reports SUCCESS and TASK COMPLETE - goal reached!"
                                )
                                success = True
                                break
                            else:
                                self.get_logger().warning(
                                    "Nav2 reports SUCCESS but task NOT complete! Retrying..."
                                )
                                retry_count += 1
                        except Exception as e:
                            self.get_logger().error(
                                f"Error checking task completion: {e}"
                            )
                            # If we can't check completion but got SUCCEEDED, make log warning and return false
                            self.get_logger().warning(
                                "Nav2 reports SUCCESS but cannot verify task completion - treating as FAILED for safety!"
                            )
                            retry_count += 1
                    elif result == TaskResult.FAILED:
                        self.get_logger().info("Nav2 reports FAILED. Retrying...")
                        retry_count += 1
                    elif result == TaskResult.CANCELED:
                        self.get_logger().info("Nav2 reports CANCELED. Retrying...")
                        retry_count += 1
                    else:
                        self.get_logger().warning(f"Unexpected result: {result}")
                        retry_count += 1
                except Exception as e:
                    self.get_logger().error(f"Error getting result: {e}")
                    retry_count += 1

            response.success = success
            status_msg = (
                "Goal reached successfully!" if success else "Failed to reach goal."
            )
            self.get_logger().info(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error in navigate_to_goal: {e}")
            response.success = False

        return response

    def _calculate_distance_to_goal(self, goal_pose):
        """
        Calculate the actual distance from current robot position to goal.
        Args:
            goal_pose: PoseStamped target position
        Returns:
            float: Distance in meters, or None if calculation fails
        """
        try:
            # Get current robot position
            current_tf = self._lookup_pose("base_link", timeout_sec=1.0)

            # Calculate distance
            distance = logic.distance_2d(
                current_tf.transform.translation.x,
                current_tf.transform.translation.y,
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
            )

            self.get_logger().debug(
                f"Current position: x={current_tf.transform.translation.x:.2f}, y={current_tf.transform.translation.y:.2f}"
            )
            self.get_logger().debug(
                f"Goal position: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}"
            )
            self.get_logger().debug(f"Distance to goal: {distance:.2f}m")

            return distance

        except Exception as e:
            self.get_logger().error(f"Error calculating distance to goal: {e}")
            return None

    def clear_costmaps_humble(self):
        """
        Alternative method to clear costmaps in Humble by clearing global and local maps separately.
        """
        try:
            # Try to use clearGlobalCostmap and clearLocalCostmap separately
            self.navigator.clearGlobalCostmap()
            self.navigator.clearLocalCostmap()
            self.get_logger().info("Costmaps cleared using Humble method")
        except Exception as e:
            self.get_logger().error(f"Failed to clear costmaps: {e}")

    def cancel_goal_callback(self, req, res):
        """
        Service callback to cancel current navigation.
        Args:
            req: std_srvs/Trigger request
            res: std_srvs/Trigger response
        Returns:
            res: Updated response with success status
        """
        try:
            self.get_logger().info("Goal cancellation requested.")
            self.navigator.cancelTask()
            res.success = True
            res.message = "Navigation goal canceled."
        except Exception as e:
            self.get_logger().error(f"Error in cancel_goal_callback: {e}")
            res.success = False
            res.message = f"Error canceling goal: {e}"
        return res

    def clear_costmap_callback(self, req, res):
        """
        Service callback to clear all costmaps.
        Args:
            req: std_srvs/Trigger request
            res: std_srvs/Trigger response
        Returns:
            res: Updated response with success status
        """
        try:
            self.get_logger().info("Clearing all costmaps.")
            try:
                self.navigator.clearAllCostmaps()
                self.get_logger().info("Successfully cleared all costmaps")
                res.success = True
                res.message = "Costmaps cleared."
            except Exception as e:
                self.get_logger().warning(f"Error clearing all costmaps: {e}")
                # Try alternative method
                try:
                    self.clear_costmaps_humble()
                    self.get_logger().info(
                        "Successfully cleared costmaps using Humble method"
                    )
                    res.success = True
                    res.message = "Costmaps cleared (Humble method)."
                except Exception as e2:
                    self.get_logger().error(
                        f"Failed to clear costmaps in any way: {e2}"
                    )
                    res.success = False
                    res.message = f"Failed to clear costmaps: {e2}"
        except Exception as e:
            self.get_logger().error(f"Error in clear_costmap_callback: {e}")
            res.success = False
            res.message = f"Error clearing costmaps: {e}"
        return res

    def _lookup_pose(self, frame, timeout_sec=1.0, ref_frame="map"):
        """Look up `frame` in `ref_frame` via TF.

        Centralizes the map->frame lookup that was duplicated across every
        motion callback. Callers keep their own try/except so failure handling
        (retry / abort / skip cycle) stays local to each routine.
        """
        return self.tf_buffer.lookup_transform(
            ref_frame,
            frame,
            rclpy.time.Time(),
            Duration(seconds=timeout_sec),
        )

    def _stop_robot(self, repeats=10, delay=0.02):
        """Publish a zero Twist `repeats` times to force the heavy base to stop."""
        twist = Twist()
        for _ in range(repeats):
            self.cmd_vel_manual_publisher.publish(twist)
            time.sleep(delay)

    def rotate_callback(self, request, response):
        """
        Service callback to rotate the robot with PD-Control for high precision.
        Optimized for 'Fast but Accurate' performance on heavy platforms.
        """
        try:
            # PD tuning constants + control law live in nav_to_pose_logic
            # (compute_rotation_command). TOLERANCE is per-request.
            tolerance_deg = (
                request.tolerance_degrees if request.tolerance_degrees > 0.0 else 10.0
            )
            TOLERANCE = math.radians(tolerance_deg)
            MAX_DURATION = 30.0
            # ---------------------------------------------

            # 1. Get initial orientation
            try:
                initial_tf = self._lookup_pose("base_link", timeout_sec=1.0)
                initial_yaw = quaternion_to_yaw(initial_tf.transform.rotation)
            except Exception as e:
                self.get_logger().warning(f"No transform available for rotation: {e}")
                response.success = False
                return response

            # 2. Calculate target yaw
            target_angle_rad = math.radians(request.degrees)
            target_yaw = normalize_angle(initial_yaw + target_angle_rad)

            twist = Twist()
            start_time = time.time()
            last_log_time = start_time
            prev_error = target_angle_rad
            prev_time = start_time

            self.get_logger().info(f"Starting Precision Rotation: {request.degrees}°")

            while rclpy.ok() and (time.time() - start_time) < MAX_DURATION:
                # 3. Get current orientation (Fast timeout for control loop)
                try:
                    current_tf = self._lookup_pose("base_link", timeout_sec=0.05)
                    current_yaw = quaternion_to_yaw(current_tf.transform.rotation)
                except Exception:
                    # If TF fails momentarily, skip this cycle but don't crash
                    time.sleep(0.02)
                    continue

                # 4. Calculate Error
                angle_diff = normalize_angle(target_yaw - current_yaw)
                error_degrees = math.degrees(angle_diff)

                # Log progress every 1.0s
                if time.time() - last_log_time > 1.0:
                    self.get_logger().info(
                        f"Rotating... Remaining Error: {error_degrees:.2f}°"
                    )
                    last_log_time = time.time()

                # 5. Check for Success
                if abs(angle_diff) < TOLERANCE:
                    self.get_logger().info(
                        f"Target reached within tolerance (Error: {error_degrees:.3f}°)"
                    )
                    break

                # 6. Control Logic (PD-Controller + Decel Zone)

                # A. Calculate Derivative (rate of change of error)
                now = time.time()
                dt = now - prev_time
                error_rate = (angle_diff - prev_error) / dt if dt > 0 else 0.0
                prev_error = angle_diff
                prev_time = now

                # B/C. PD output + speed constraints (see nav_to_pose_logic)
                twist.angular.z = logic.compute_rotation_command(angle_diff, error_rate)
                self.cmd_vel_manual_publisher.publish(twist)

                # Run at 50Hz
                time.sleep(0.02)

            # 7. STOP SEQUENCE (Aggressive)
            # Send stop multiple times to ensure the heavy base reacts immediately
            self._stop_robot(repeats=10, delay=0.02)

            # 8. Settling Time
            # Wait for inertia to settle before reading the final verification angle
            time.sleep(0.5)

            # 9. Final Verification
            try:
                final_tf = self._lookup_pose("base_link", timeout_sec=1.0)
                final_yaw = quaternion_to_yaw(final_tf.transform.rotation)
                actual_rotation = normalize_angle(final_yaw - initial_yaw)
                final_error = abs(math.degrees(actual_rotation) - request.degrees)

                self.get_logger().info(
                    f"Rotation Complete. Requested: {request.degrees}°, Actual: {math.degrees(actual_rotation):.2f}°, Final Error: {final_error:.2f}°"
                )

                # Success if error is under the requested tolerance (allowing for slight sensor noise)
                response.success = final_error < tolerance_deg

            except Exception as e:
                self.get_logger().error(f"Could not verify final rotation: {e}")
                response.success = False

        except Exception as e:
            self.get_logger().error(f"Error in rotation: {e}")
            # Emergency Stop
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_manual_publisher.publish(twist)
            response.success = False

        return response

    def move_forward_callback(self, request, response):
        """
        Service callback to move the robot forward by a specified distance with feedback.
        Args:
            request: MoveWheel request with distance
            response: Service response
        Returns:
            response: Updated service response with success status
        """
        try:
            target_distance = request.distance
            if target_distance >= 0:
                linear_speed = 0.2
            else:
                linear_speed = -0.2
            tolerance = 0.05  # 5cm tolerance
            max_duration = target_distance / linear_speed + 5.0  # Safety timeout

            # Get initial position
            try:
                initial_tf = self._lookup_pose("base_link", timeout_sec=1.0)
                initial_x = initial_tf.transform.translation.x
                initial_y = initial_tf.transform.translation.y
                self.get_logger().info(
                    f"Starting position: x={initial_x:.3f}, y={initial_y:.3f}"
                )
            except Exception as e:
                self.get_logger().warning(f"Could not get initial position: {e}")
                # Fallback to time-based movement
                duration = target_distance / linear_speed
                twist = Twist()
                twist.linear.x = linear_speed

                start_time = time.time()
                while time.time() - start_time < duration and rclpy.ok():
                    self.cmd_vel_manual_publisher.publish(twist)
                    time.sleep(0.1)

                twist.linear.x = 0.0
                self.cmd_vel_manual_publisher.publish(twist)
                response.success = True
                return response

            twist = Twist()
            start_time = time.time()

            self.get_logger().info(f"Moving {target_distance:.2f}m forward...")

            while rclpy.ok() and (time.time() - start_time) < max_duration:
                # Get current position
                try:
                    current_tf = self._lookup_pose("base_link", timeout_sec=0.1)
                    current_x = current_tf.transform.translation.x
                    current_y = current_tf.transform.translation.y

                    # Calculate distance traveled
                    distance_traveled = math.sqrt(
                        (current_x - initial_x) ** 2 + (current_y - initial_y) ** 2
                    )

                    # Log progress every second
                    elapsed = time.time() - start_time
                    if int(elapsed) % 1 == 0 and elapsed > 0:
                        progress = (distance_traveled / abs(target_distance)) * 100
                        self.get_logger().info(
                            f"Progress: {distance_traveled:.3f}m/{target_distance:.2f}m ({progress:.1f}%)"
                        )

                    # Check if target distance reached
                    if distance_traveled >= (abs(target_distance) - tolerance):
                        self.get_logger().info(
                            f"Target reached! Traveled: {distance_traveled:.3f}m"
                        )
                        break

                    # Adjust speed based on remaining distance
                    remaining_distance = target_distance - distance_traveled
                    if abs(remaining_distance) < 0.1:
                        twist.linear.x = linear_speed * 0.3  # Slow down near target
                    else:
                        twist.linear.x = linear_speed

                    self.cmd_vel_manual_publisher.publish(twist)

                except Exception:
                    # If TF fails, continue with current speed
                    twist.linear.x = linear_speed
                    self.cmd_vel_manual_publisher.publish(twist)

                time.sleep(0.1)  # 10Hz control loop

            # Stop movement
            twist.linear.x = 0.0
            self.cmd_vel_manual_publisher.publish(twist)
            time.sleep(0.1)
            self.cmd_vel_manual_publisher.publish(twist)  # Ensure stop

            # Verify final distance
            try:
                final_tf = self._lookup_pose("base_link", timeout_sec=0.5)
                final_x = final_tf.transform.translation.x
                final_y = final_tf.transform.translation.y
                actual_distance = math.sqrt(
                    (final_x - initial_x) ** 2 + (final_y - initial_y) ** 2
                )

                error = abs(actual_distance - target_distance)
                self.get_logger().info(
                    f"Movement complete. Target: {target_distance:.2f}m, Actual: {actual_distance:.3f}m, Error: {error:.3f}m"
                )
                response.success = (error < (tolerance * 2)) or (
                    abs(target_distance + actual_distance) < (tolerance * 2)
                )  # Allow 10cm tolerance for success
            except Exception as e:
                self.get_logger().error(f"Could not verify final distance: {e}")
                response.success = True  # Assume success if can't verify

        except Exception as e:
            self.get_logger().error(f"Error in move_forward_callback: {e}")
            response.success = False

        return response

    def save_pose_callback(self, request, response):
        """
        Service callback to save the current position with a name using TF transform.
        Args:
            request: SaveLocationCommand request with location name
            response: Service response
        Returns:
            response: Updated service response with success status
        """
        try:
            self.get_logger().info("Saving current position using TF transform...")
            self.get_logger().info(f"YAML file path: {yaml_path}")

            # Get transform data from TF
            try:
                # Wait for transform from map to base_link
                tf = self._lookup_pose("base_link", timeout_sec=1.0)
                self.get_logger().info(
                    f"Got TF transform: x={tf.transform.translation.x:.2f}, y={tf.transform.translation.y:.2f}"
                )

                position_name = request.location_name.strip()
                data = logic.merge_position(
                    yaml_path,
                    position_name,
                    "map",
                    (
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z,
                    ),
                    (
                        tf.transform.rotation.x,
                        tf.transform.rotation.y,
                        tf.transform.rotation.z,
                        tf.transform.rotation.w,
                    ),
                )
                if position_name in data["positions"]:
                    self.get_logger().info(
                        f"Position '{position_name}' already exists. Replacing with new position."
                    )

                # Save the updated data back to the YAML file
                try:
                    # Check if we have write permissions
                    if os.access(yaml_path, os.W_OK):
                        logic.write_positions_yaml(yaml_path, data)
                        self.get_logger().info(
                            f"Saved current position as '{position_name}' in YAML file."
                        )
                        response.success = True
                    else:
                        self.get_logger().error(f"No write permission to {yaml_path}")
                        response.success = False
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to save position to YAML file: {e}"
                    )
                    response.success = False

            except Exception as e:
                if "timeout" in str(e).lower():
                    self.get_logger().error(f"Timed out waiting for transform: {e}")
                else:
                    self.get_logger().error(f"Error looking up transform: {e}")
                response.success = False

        except Exception as e:
            self.get_logger().error(f"Error in save_pose_callback: {e}")
            response.success = False

        return response

    def reload_yaml_callback(self, request, response):
        """
        Service callback to reload saved positions from YAML file.
        Args:
            request: Trigger request
            response: Trigger response
        Returns:
            response: Updated response with success status
        """
        try:
            self.saved_positions = self.load_saved_positions()
            response.success = True
            response.message = "YAML file reloaded successfully."
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error(f"Error in reload_yaml_callback: {e}")
            response.success = False
            response.message = f"Error reloading YAML file: {str(e)}"
        return response

    def nudge_to_tf_callback(self, request, response):
        """
        Service callback to navigate to a fixed remaining distance towards a TF frame using cmd_vel.
        The robot will move towards the target TF frame but stop when it is a specified distance away,
        maintaining its current orientation and optionally rotating to face the target.

        Args:
            request: NavToTF request containing the target TF frame name
            response: Service response
        Returns:
            response: Updated service response with success status
        """
        try:
            target_tf_name = request.object_tf_name.strip()
            remaining_distance_goal = float(request.stop_distance)
            self.get_logger().info(
                f"Moving via cmd_vel towards TF frame: {target_tf_name} (stop {remaining_distance_goal}m before target)"
            )

            # Get initial robot position
            try:
                initial_tf = self._lookup_pose("base_link", timeout_sec=1.0)
                initial_x = initial_tf.transform.translation.x
                initial_y = initial_tf.transform.translation.y
                initial_yaw = quaternion_to_yaw(initial_tf.transform.rotation)

                self.get_logger().info(
                    f"Initial robot position: x={initial_x:.3f}, y={initial_y:.3f}, yaw={math.degrees(initial_yaw):.1f}°"
                )

            except Exception as e:
                self.get_logger().error(f"Failed to get robot position: {e}")
                response.success = False
                return response

            # Get target TF frame position
            try:
                target_tf = self._lookup_pose(target_tf_name, timeout_sec=1.0)
                target_x = target_tf.transform.translation.x
                target_y = target_tf.transform.translation.y

                self.get_logger().info(
                    f"Target TF position: x={target_x:.3f}, y={target_y:.3f}"
                )

            except Exception as e:
                self.get_logger().error(
                    f"Failed to get target TF frame '{target_tf_name}': {e}"
                )
                response.success = False
                return response

            # Calculate total distance and direction
            total_distance = math.sqrt(
                (target_x - initial_x) ** 2 + (target_y - initial_y) ** 2
            )
            self.get_logger().info(f"Total distance to target: {total_distance:.3f}m")

            # remaining_distance_goal กำหนดโดย request
            if total_distance <= remaining_distance_goal:
                self.get_logger().info(
                    "Robot is already within the desired remaining distance to the target."
                )
                response.success = True
                return response

            # Calculate distance to travel to stop at remaining distance
            target_distance = total_distance - remaining_distance_goal

            direction_x = (target_x - initial_x) / total_distance
            direction_y = (target_y - initial_y) / total_distance
            target_angle = math.atan2(direction_y, direction_x)

            self.get_logger().info(
                f"Target distance to travel: {target_distance:.3f}m, direction: {math.degrees(target_angle):.1f}°"
            )

            # Calculate required rotation to face target
            angle_diff = normalize_angle(target_angle - initial_yaw)

            # Step 1: Rotate to face target direction
            if abs(angle_diff) > math.radians(5):
                self.get_logger().info(
                    f"Rotating {math.degrees(angle_diff):.1f}° to face target"
                )

                angular_speed = 0.5
                tolerance = math.radians(3)
                max_rotation_time = abs(angle_diff) / angular_speed + 5.0

                twist = Twist()
                start_time = time.time()

                while rclpy.ok() and (time.time() - start_time) < max_rotation_time:
                    try:
                        current_tf = self._lookup_pose("base_link", timeout_sec=0.1)
                        current_yaw = quaternion_to_yaw(current_tf.transform.rotation)
                    except Exception:
                        time.sleep(0.1)
                        continue

                    remaining_angle = normalize_angle(target_angle - current_yaw)

                    if abs(remaining_angle) < tolerance:
                        self.get_logger().info("Rotation to target direction completed")
                        break

                    twist.angular.z = (
                        angular_speed if remaining_angle > 0 else -angular_speed
                    )
                    self.cmd_vel_manual_publisher.publish(twist)
                    time.sleep(0.05)

                twist.angular.z = 0.0
                self.cmd_vel_manual_publisher.publish(twist)
                time.sleep(0.1)

            # Step 2: Move forward the target distance
            self.get_logger().info(f"Moving forward {target_distance:.3f}m")

            linear_speed = 0.2
            tolerance = 0.05
            max_move_time = target_distance / linear_speed + 10.0

            twist = Twist()
            start_time = time.time()

            while rclpy.ok() and (time.time() - start_time) < max_move_time:
                try:
                    current_tf = self._lookup_pose("base_link", timeout_sec=0.1)
                    current_x = current_tf.transform.translation.x
                    current_y = current_tf.transform.translation.y

                    distance_traveled = math.sqrt(
                        (current_x - initial_x) ** 2 + (current_y - initial_y) ** 2
                    )

                    elapsed = time.time() - start_time
                    if int(elapsed) % 1 == 0 and elapsed > 0:
                        progress = (distance_traveled / target_distance) * 100
                        self.get_logger().info(
                            f"Movement progress: {distance_traveled:.3f}m/{target_distance:.3f}m ({progress:.1f}%)"
                        )

                    if distance_traveled >= (target_distance - tolerance):
                        self.get_logger().info(
                            f"Target distance reached! Traveled: {distance_traveled:.3f}m"
                        )
                        break

                    remaining_distance = target_distance - distance_traveled
                    twist.linear.x = (
                        linear_speed * 0.3 if remaining_distance < 0.1 else linear_speed
                    )

                    self.cmd_vel_manual_publisher.publish(twist)

                except Exception:
                    twist.linear.x = linear_speed
                    self.cmd_vel_manual_publisher.publish(twist)

                time.sleep(0.1)

            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_manual_publisher.publish(twist)
            time.sleep(0.1)
            self.cmd_vel_manual_publisher.publish(twist)

            # Optional: Rotate to face the target object TF frame
            self.get_logger().info(
                f"Rotating to face target object TF frame: {target_tf_name}"
            )

            try:
                current_tf = self._lookup_pose("base_link", timeout_sec=1.0)
                current_x = current_tf.transform.translation.x
                current_y = current_tf.transform.translation.y
                current_yaw = quaternion_to_yaw(current_tf.transform.rotation)

                target_tf_updated = self._lookup_pose(target_tf_name, timeout_sec=1.0)
                target_x_updated = target_tf_updated.transform.translation.x
                target_y_updated = target_tf_updated.transform.translation.y

                final_target_angle = math.atan2(
                    target_y_updated - current_y, target_x_updated - current_x
                )

                final_angle_diff = normalize_angle(final_target_angle - current_yaw)

                self.get_logger().info(
                    f"Final rotation needed: {math.degrees(final_angle_diff):.1f}° to face object"
                )

                if abs(final_angle_diff) > math.radians(3):
                    angular_speed = 0.3
                    tolerance = math.radians(2)
                    max_rotation_time = abs(final_angle_diff) / angular_speed + 5.0

                    twist = Twist()
                    start_time = time.time()

                    while rclpy.ok() and (time.time() - start_time) < max_rotation_time:
                        try:
                            current_tf_rot = self._lookup_pose(
                                "base_link", timeout_sec=0.1
                            )
                            current_yaw_rot = quaternion_to_yaw(
                                current_tf_rot.transform.rotation
                            )
                        except Exception:
                            time.sleep(0.1)
                            continue

                        remaining_angle = normalize_angle(
                            final_target_angle - current_yaw_rot
                        )

                        if abs(remaining_angle) < tolerance:
                            self.get_logger().info(
                                "Final rotation to face object completed"
                            )
                            break

                        speed = (
                            angular_speed * 0.5
                            if abs(remaining_angle) < math.radians(10)
                            else angular_speed
                        )
                        twist.angular.z = speed if remaining_angle > 0 else -speed
                        self.cmd_vel_manual_publisher.publish(twist)
                        time.sleep(0.05)

                    twist.angular.z = 0.0
                    self.cmd_vel_manual_publisher.publish(twist)
                    time.sleep(0.1)
                    self.cmd_vel_manual_publisher.publish(twist)
                    self.get_logger().info("Robot is now facing the target object")
                else:
                    self.get_logger().info(
                        "Robot is already facing the target object (within 3°)"
                    )

            except Exception as e:
                self.get_logger().warning(
                    f"Could not rotate to face target object: {e}"
                )

            # Final verification
            try:
                final_tf = self._lookup_pose("base_link", timeout_sec=0.5)
                final_x = final_tf.transform.translation.x
                final_y = final_tf.transform.translation.y
                actual_distance = math.sqrt(
                    (final_x - initial_x) ** 2 + (final_y - initial_y) ** 2
                )

                error = abs(actual_distance - target_distance)
                self.get_logger().info(
                    f"Movement complete. Target: {target_distance:.3f}m, Actual: {actual_distance:.3f}m, Error: {error:.3f}m"
                )

                response.success = error < 0.15

                if response.success:
                    self.get_logger().info(
                        f"Successfully moved within desired remaining distance to TF '{target_tf_name}' and rotated to face it"
                    )
                else:
                    self.get_logger().warning(
                        f"Movement completed but with large error: {error:.3f}m"
                    )

            except Exception as e:
                self.get_logger().error(f"Could not verify final position: {e}")
                response.success = True

        except Exception as e:
            self.get_logger().error(f"Error in nudge_to_tf_callback: {e}")
            response.success = False

        return response

    # Tolerances (SIDE_BEARING_TOLERANCE / STANDOFF_TOLERANCE / FACING_TOLERANCE)
    # live in nav_to_pose_logic, used here as `logic.<NAME>`.

    def _make_pose(self, x, y, yaw, frame_id="map"):
        """Build a PoseStamped in `frame_id` from a planar (x, y, yaw)."""
        return logic.make_pose(x, y, yaw, self.get_clock().now().to_msg(), frame_id)

    def _goal_is_plannable(self, start_pose, goal_pose):
        """Ask the global planner whether `goal_pose` is reachable.

        Returns True/False, or None when the planner could not be consulted
        (action unavailable, exception) so the caller can tell "unreachable"
        apart from "unknown" and only fail loudly on the former.
        """
        try:
            path = self.navigator.getPath(start_pose, goal_pose)
        except Exception as e:
            self.get_logger().warning(
                f"Could not consult planner for reachability: {e}"
            )
            return None
        return logic.path_is_plannable(path)

    def _face_target(self, target_x, target_y):
        """Spin in place until the robot points at (target_x, target_y).

        Reuses the PD loop in rotate_callback rather than adding a third
        hand-rolled spin loop. Best effort: returns True when the robot ends up
        within FACING_TOLERANCE, False if it could not be verified/corrected.
        """
        try:
            tf = self._lookup_pose("base_link", timeout_sec=1.0)
        except Exception as e:
            self.get_logger().warning(f"Could not read pose to face target: {e}")
            return False

        current_yaw = quaternion_to_yaw(tf.transform.rotation)
        desired_yaw = math.atan2(
            target_y - tf.transform.translation.y,
            target_x - tf.transform.translation.x,
        )
        error = normalize_angle(desired_yaw - current_yaw)

        if abs(error) <= logic.FACING_TOLERANCE:
            self.get_logger().info(
                f"Already facing target (error {math.degrees(error):.1f} deg)"
            )
            return True

        self.get_logger().info(
            f"Correcting final heading by {math.degrees(error):.1f} deg to face target"
        )
        rotate_request = Rotate.Request()
        rotate_request.degrees = float(math.degrees(error))
        rotate_request.tolerance_degrees = float(math.degrees(logic.FACING_TOLERANCE))
        return bool(self.rotate_callback(rotate_request, Rotate.Response()).success)

    def nav_to_tf_callback(self, request, response):
        """
        Service callback to navigate towards a TF frame using Nav2 Simple Commander.

        Two modes, selected by request.approach_side:

        AUTO (empty string, "AUTO" or "NONE") - legacy behaviour. The robot
        stops stop_distance short of the target along the line it already
        stands on, so which side it ends up on depends on where it started.

        Named side (FRONT, FRONT_LEFT, LEFT, ...) - the robot parks on that
        specific side of the target and faces it. Sides are MAP-RELATIVE
        (FRONT = map +X, LEFT = map +Y, ...) because the pinned object TFs are
        published with identity rotation and carry no usable heading of their
        own. request.object_radius is added to stop_distance in this mode so a
        caller can ask to stop clear of a known-size object.

        A requested side that the planner cannot reach fails loudly rather than
        silently falling back to another side.
        """
        response.message = ""
        try:
            target_tf_name = request.object_tf_name.strip()
            stop_distance = float(request.stop_distance)
            timeout_sec = int(request.timeout)
            object_radius = float(request.object_radius)
            requested_side = request.approach_side.strip().upper()

            directed = requested_side not in AUTO_SIDES
            if directed and requested_side not in SIDE_OFFSETS:
                response.success = False
                response.message = (
                    f"Unknown approach_side '{request.approach_side}'. "
                    f"Valid: {', '.join(sorted(SIDE_OFFSETS))}, or AUTO."
                )
                self.get_logger().error(response.message)
                return response

            self.get_logger().info(
                f"Nav2 navigation to TF '{target_tf_name}' with "
                f"stop_distance={stop_distance}m, side="
                f"{requested_side if directed else 'AUTO'}"
            )

            # Get current robot pose
            try:
                robot_tf = self._lookup_pose("base_link", timeout_sec=1.0)
                robot_x = robot_tf.transform.translation.x
                robot_y = robot_tf.transform.translation.y
                robot_yaw = quaternion_to_yaw(robot_tf.transform.rotation)
                self.get_logger().info(
                    f"Current robot pose: x={robot_x:.3f}, y={robot_y:.3f}, yaw={math.degrees(robot_yaw):.1f}°"
                )
            except Exception as e:
                response.success = False
                response.message = f"Failed to get robot pose: {e}"
                self.get_logger().error(response.message)
                return response

            # Get target frame pose
            try:
                target_tf = self._lookup_pose(target_tf_name, timeout_sec=1.0)
                target_x = target_tf.transform.translation.x
                target_y = target_tf.transform.translation.y
                self.get_logger().info(
                    f"Target TF '{target_tf_name}' pose: x={target_x:.3f}, y={target_y:.3f}"
                )
            except Exception as e:
                response.success = False
                response.message = f"Failed to get target TF '{target_tf_name}': {e}"
                self.get_logger().error(response.message)
                return response

            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx * dx + dy * dy)
            self.get_logger().info(
                f"Distance to target: {distance:.3f}m, stop_distance: {stop_distance:.3f}m"
            )

            if directed:
                # Sides are map-relative, hence a reference yaw of 0.0.
                standoff = object_radius + stop_distance
                goal_x, goal_y, goal_yaw = approach_pose_for_side(
                    target_x, target_y, 0.0, requested_side, standoff
                )
                self.get_logger().info(
                    f"Standoff on side {requested_side}: x={goal_x:.3f}, y={goal_y:.3f}, "
                    f"yaw={math.degrees(goal_yaw):.1f}° "
                    f"(standoff {standoff:.3f}m = radius {object_radius:.3f} + stop {stop_distance:.3f})"
                )

                # "Already there" must be judged against the standoff POSE, not
                # the raw distance to the target: the robot can be well inside
                # stop_distance and still be on entirely the wrong side.
                pose_error = math.hypot(goal_x - robot_x, goal_y - robot_y)
                yaw_error = abs(normalize_angle(goal_yaw - robot_yaw))
                if (
                    pose_error <= logic.STANDOFF_TOLERANCE
                    and yaw_error <= logic.FACING_TOLERANCE
                ):
                    response.success = True
                    response.message = f"Already at the {requested_side} standoff pose."
                    self.get_logger().info(response.message)
                    return response
            else:
                # Legacy behaviour, unchanged: stop short along the robot->target
                # line and face the target.
                if distance <= stop_distance:
                    response.success = True
                    response.message = "Robot is already within stop_distance."
                    self.get_logger().info(response.message)
                    return response

                direction_x = dx / distance
                direction_y = dy / distance
                approach_distance = distance - stop_distance
                goal_x = robot_x + direction_x * approach_distance
                goal_y = robot_y + direction_y * approach_distance
                goal_yaw = math.atan2(dy, dx)

            self.get_logger().info(
                f"Calculated goal: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.1f}°"
            )

            goal_pose = self._make_pose(goal_x, goal_y, goal_yaw)

            # Clear costmaps before navigation
            try:
                try:
                    self.navigator.clearAllCostmaps()
                    self.get_logger().info("Cleared all costmaps")
                except Exception:
                    self.clear_costmaps_humble()
                    self.get_logger().info("Cleared costmaps using Humble method")
            except Exception as e:
                self.get_logger().warning(f"Failed to clear costmaps: {e}")

            # Reachability check. A requested side that cannot be planned to is
            # a hard failure - we do not quietly park somewhere else.
            plannable = self._goal_is_plannable(
                self._make_pose(robot_x, robot_y, robot_yaw), goal_pose
            )
            if plannable is False:
                response.success = False
                response.message = (
                    f"No path to the {requested_side if directed else 'AUTO'} standoff pose "
                    f"({goal_x:.2f}, {goal_y:.2f}) - it is unreachable or in collision."
                )
                self.get_logger().error(response.message)
                return response

            # Send goal to Nav2
            try:
                self.navigator.goToPose(goal_pose)
                self.get_logger().info("Goal sent to Nav2")
            except Exception as e:
                response.success = False
                response.message = f"Failed to send goal to Nav2: {e}"
                self.get_logger().error(response.message)
                return response

            # Wait for navigation to complete
            start_time = time.time()
            success = False
            last_log_time = start_time

            while rclpy.ok():
                current_time = time.time()
                elapsed = current_time - start_time

                # Log progress every 5 seconds
                if current_time - last_log_time >= 5.0:
                    self.get_logger().info(
                        f"Navigation in progress: {elapsed:.1f}s elapsed"
                    )
                    last_log_time = current_time

                # Check if task is complete
                try:
                    if self.navigator.isTaskComplete():
                        result = self.navigator.getResult()
                        self.get_logger().info(
                            f"Navigation complete with result: {result}"
                        )
                        success = result == TaskResult.SUCCEEDED
                        break
                except Exception as e:
                    self.get_logger().warning(f"Error checking task completion: {e}")

                # Check timeout
                if timeout_sec > 0 and elapsed > timeout_sec:
                    self.get_logger().warning(
                        f"Navigation timeout reached ({timeout_sec}s); canceling task"
                    )
                    try:
                        self.navigator.cancelTask()
                    except Exception as e:
                        self.get_logger().error(f"Error canceling task: {e}")
                    success = False
                    break

                time.sleep(0.1)

            response.success = success

            if not success:
                self.get_logger().warning(
                    "Nav2 did not report SUCCEEDED (failed/canceled/timeout) - "
                    "checking actual robot position before deciding final result"
                )

            # Nav2's controller finishes anywhere inside xy_goal_tolerance, so
            # the heading it leaves us with is not necessarily pointing at the
            # target. Correct it in place before verifying.
            self._face_target(target_x, target_y)

            # Verify the final pose - this is authoritative: if the robot is
            # actually parked correctly, count it as success even if Nav2
            # canceled/timed out right before reporting completion.
            try:
                final_tf = self._lookup_pose("base_link", timeout_sec=0.5)
                final_x = final_tf.transform.translation.x
                final_y = final_tf.transform.translation.y

                if directed:
                    # Correct means BOTH the right radial distance and the right
                    # side of the target.
                    standoff = object_radius + stop_distance
                    ok, final_distance, log_msg = logic.verify_directed_standoff(
                        final_x, final_y, target_x, target_y, requested_side, standoff
                    )

                    if ok:
                        response.success = True
                        response.message = f"Parked on the {requested_side} side of '{target_tf_name}'."
                        self.get_logger().info(log_msg + " - Success!")
                    else:
                        response.success = False
                        response.message = log_msg
                        self.get_logger().warning(log_msg)
                else:
                    ok, final_distance, log_msg = logic.verify_auto_standoff(
                        final_x, final_y, target_x, target_y, stop_distance
                    )
                    if ok:
                        self.get_logger().info(log_msg + " - Success!")
                        response.success = True
                        response.message = (
                            f"Stopped {final_distance:.2f}m from '{target_tf_name}'."
                        )
                    else:
                        self.get_logger().warning(
                            log_msg
                            + f" - Outside tolerance by {(final_distance - stop_distance):.3f}m"
                        )
                        response.message = log_msg

            except Exception as e:
                self.get_logger().warning(f"Could not verify final pose: {e}")
                if not response.message:
                    response.message = f"Could not verify final pose: {e}"

            return response

        except Exception as e:
            self.get_logger().error(f"Error in nav_to_tf_callback: {e}")
            response.success = False
            response.message = f"Error in nav_to_tf_callback: {e}"
            return response


def main():
    """Main entry point for the navigate_to_goal_server node."""
    rclpy.init()

    # Set up logging to be more verbose for debugging
    rcl_logging_level = rclpy.logging.LoggingSeverity.DEBUG
    rclpy.logging.set_logger_level("navigate_to_goal_server", rcl_logging_level)

    node = NavigateToGoalServer()

    try:
        # Use MultiThreadedExecutor for better performance with callback groups
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        # Spin once to make sure service discovery happens before the main loop
        for _ in range(10):
            executor.spin_once(timeout_sec=0.1)

        node.get_logger().info(
            "Starting spin loop - all services should be available now"
        )

        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
