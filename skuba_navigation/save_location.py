#!/usr/bin/env python3

import yaml
import os
import rclpy
import math
import numpy as np
from collections import defaultdict
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from skuba_interfaces.srv import SaveLocationWithSide
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from pathlib import Path
from skuba_navigation.geometry_utils import quaternion_to_yaw, yaw_to_quaternion

NAMESPACE    = 'skuba/cheesecake/nav'
PACKAGE_NAME = 'skuba_navigation'

ZONES = {
    "FRONT":       0.0,
    "FRONT_LEFT":  45.0,
    "LEFT":        90.0,
    "BACK_LEFT":   135.0,
    "BACK":        180.0,
    "BACK_RIGHT":  225.0,
    "RIGHT":       270.0,
    "FRONT_RIGHT": 315.0,
}
HALF_WIDTH_DEG   = 25.0
OBSTACLE_RANGE   = 1.0   # m
ACCUMULATE_SCANS = 10


class SaveLocationServer(Node):
    def __init__(self):
        super().__init__('save_location_server')
        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.latest_scan = None
        self.zone_accumulator: dict[str, int] = defaultdict(int)
        self.scan_count = 0

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group
        )

        # ── YAML path ─────────────────────────────────────────────────────────
        try:
            install_share = Path(get_package_share_directory(PACKAGE_NAME))
            ws_root = install_share.parents[3]
            self.yaml_path = ws_root / 'src' / PACKAGE_NAME / 'share' / 'saved_pose.yaml'
            if not self.yaml_path.parent.exists():
                self.get_logger().warning(
                    f"Source dir not found at {self.yaml_path.parent}. "
                    "Falling back to install share."
                )
                self.yaml_path = install_share / 'saved_pose.yaml'
        except Exception as e:
            self.get_logger().error(f"Failed to determine YAML path: {e}")
            self.yaml_path = Path('saved_pose.yaml')

        self.get_logger().info(f"Using YAML path: {self.yaml_path}")

        self.srv = self.create_service(
            SaveLocationWithSide,
            f'{NAMESPACE}/save_location_with_side',
            self.save_location_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            f"Save Location Server ready  "
            f"[zones: {list(ZONES.keys())}  "
            f"±{HALF_WIDTH_DEG}°  accumulate {ACCUMULATE_SCANS} scans]"
        )

    # =========================================================================
    # Scan callback — accumulate zone hits
    # =========================================================================

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg

        ranges   = np.array(msg.ranges)
        ranges[~np.isfinite(ranges)] = np.inf

        num_beams = len(msg.ranges)
        # TF base_link→laser: yaw=π, roll=0 → θ_base = θ_laser + π
        # (pure 180° yaw about Z; laser X points to robot rear. The previous
        #  π - θ_laser reflected the angle and swapped robot left/right.)
        angles    = np.pi + (msg.angle_min + np.arange(num_beams) * msg.angle_increment)
        half_rad  = math.radians(HALF_WIDTH_DEG)

        for side, center_deg in ZONES.items():
            center_rad = math.radians(center_deg)
            # circular diff → [-π, π]  หลีกเลี่ยง ±180° wraparound
            diff     = (angles - center_rad + math.pi) % (2.0 * math.pi) - math.pi
            zone_idx = np.where(np.abs(diff) <= half_rad)[0]

            zone_ranges = ranges[zone_idx]
            hits = int(np.sum(
                (zone_ranges >= msg.range_min) & (zone_ranges <= OBSTACLE_RANGE)
            ))
            self.zone_accumulator[side] += hits

        self.scan_count += 1

    # =========================================================================
    # Identify side from accumulated scans
    # =========================================================================

    def identify_side(self) -> str:
        # reset แล้วรอสะสมใหม่
        self.zone_accumulator = defaultdict(int)
        self.scan_count       = 0

        self.get_logger().info(
            f"Accumulating {ACCUMULATE_SCANS} scans to identify side..."
        )

        while self.scan_count < ACCUMULATE_SCANS:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Zone hit counts: {dict(self.zone_accumulator)}")

        best_side = max(self.zone_accumulator, key=self.zone_accumulator.get)
        if self.zone_accumulator[best_side] == 0:
            return "NONE"
        return best_side

    # =========================================================================
    # TF helpers
    # =========================================================================

    # quaternion_to_yaw / yaw_to_quaternion now live in geometry_utils.

    # =========================================================================
    # Service callback
    # =========================================================================

    def save_location_callback(self, request, response):
        location_name = request.location_name.strip()
        self.get_logger().info(f"--- Request to save location: '{location_name}' ---")

        try:
            # ── 1. Get current pose ───────────────────────────────────────────
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            current_yaw = quaternion_to_yaw(tf.transform.rotation)

            # ── 2. Identify side via accumulated scans ────────────────────────
            detected_side = self.identify_side()
            self.get_logger().info(f"Detected side: {detected_side}")

            # ── 3. Orient yaw toward obstacle ─────────────────────────────────
            side_offset = {
                "FRONT":       0.0,
                "FRONT_LEFT":  math.pi / 4.0,
                "LEFT":        math.pi / 2.0,
                "BACK_LEFT":   3.0 * math.pi / 4.0,
                "BACK":        math.pi,
                "BACK_RIGHT": -3.0 * math.pi / 4.0,
                "RIGHT":      -math.pi / 2.0,
                "FRONT_RIGHT": -math.pi / 4.0,
                "NONE":        0.0,
            }
            final_yaw = current_yaw + side_offset.get(detected_side, 0.0)
            q_final_z, q_final_w = yaw_to_quaternion(final_yaw)

            # ── 4. Load existing YAML ─────────────────────────────────────────
            data = {"positions": {}}
            if self.yaml_path.exists():
                try:
                    with open(self.yaml_path, 'r') as f:
                        loaded = yaml.safe_load(f)
                        if loaded and "positions" in loaded:
                            data = loaded
                except Exception as e:
                    self.get_logger().error(f"YAML load error: {e}")

            # ── 5. Write position ─────────────────────────────────────────────
            data["positions"][location_name] = {
                "header": {
                    "frame_id": "map",
                    "side": detected_side,
                },
                "pose": {
                    "position": {
                        "x": tf.transform.translation.x,
                        "y": tf.transform.translation.y,
                        "z": tf.transform.translation.z,
                    },
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": q_final_z,
                        "w": q_final_w,
                    },
                },
            }

            os.makedirs(self.yaml_path.parent, exist_ok=True)
            with open(self.yaml_path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False)

            # ── 6. Broadcast BAR static TF (identical to what was saved) ─────
            t = TransformStamped()
            t.header.stamp    = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id  = 'BAR'
            t.transform.translation.x = tf.transform.translation.x
            t.transform.translation.y = tf.transform.translation.y
            t.transform.translation.z = tf.transform.translation.z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = q_final_z
            t.transform.rotation.w = q_final_w
            self.static_broadcaster.sendTransform(t)

            response.success = True
            response.side    = detected_side
            self.get_logger().info(
                f"Saved '{location_name}'  "
                f"side={detected_side}  "
                f"yaw={math.degrees(final_yaw):.1f}°  "
                f"→ BAR TF pinned at "
                f"x={tf.transform.translation.x:.3f} "
                f"y={tf.transform.translation.y:.3f} "
                f"z={tf.transform.translation.z:.3f}"
            )

        except Exception as e:
            self.get_logger().error(f"Callback failure: {e}")
            response.success = False
            response.side    = "ERROR"

        return response


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SaveLocationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()