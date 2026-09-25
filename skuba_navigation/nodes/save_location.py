#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from skuba_interfaces.srv import SaveLocationWithSide
from tf2_ros import TransformListener, Buffer, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from pathlib import Path
from ..core.geometry_utils import quaternion_to_yaw, yaw_to_quaternion
from ..paths import data_path
from ..core.zone_detection import (
    ZONES,
    HALF_WIDTH_DEG,
    accumulate_zone_hits,
    best_zone,
    new_zone_accumulator,
    save_location_to_yaml,
    yaw_for_side,
)

NAMESPACE    = 'skuba/navigation'
PACKAGE_NAME = 'skuba_navigation'
ACCUMULATE_SCANS = 10


class SaveLocationServer(Node):
    def __init__(self):
        super().__init__('save_location_server')
        self.callback_group = ReentrantCallbackGroup()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.latest_scan = None
        self.zone_accumulator = new_zone_accumulator()
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
            self.yaml_path = data_path('share', 'saved_pose.yaml')
        except Exception as e:
            self.get_logger().error(f"Failed to determine YAML path: {e}")
            self.yaml_path = Path('saved_pose.yaml')

        self.get_logger().info(f"Using YAML path: {self.yaml_path}")

        self.srv = self.create_service(
            SaveLocationWithSide,
            f'/{NAMESPACE}/save_location_with_side',
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
        accumulate_zone_hits(msg, self.zone_accumulator)
        self.scan_count += 1

    # =========================================================================
    # Identify side from accumulated scans
    # =========================================================================

    def identify_side(self) -> str:
        self.zone_accumulator = new_zone_accumulator()
        self.scan_count = 0

        self.get_logger().info(
            f"Accumulating {ACCUMULATE_SCANS} scans to identify side..."
        )

        while self.scan_count < ACCUMULATE_SCANS:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Zone hit counts: {dict(self.zone_accumulator)}")
        return best_zone(self.zone_accumulator)

    # =========================================================================
    # Service callback
    # =========================================================================

    def save_location_callback(self, request, response):
        location_name = request.location_name.strip()
        self.get_logger().info(f"--- Request to save location: '{location_name}' ---")

        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0)
            )
            current_yaw = quaternion_to_yaw(tf.transform.rotation)

            detected_side = self.identify_side()
            self.get_logger().info(f"Detected side: {detected_side}")

            final_yaw = yaw_for_side(current_yaw, detected_side)
            q_final_z, q_final_w = yaw_to_quaternion(final_yaw)

            save_location_to_yaml(
                self.yaml_path,
                location_name,
                detected_side,
                (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z),
                (0.0, 0.0, q_final_z, q_final_w),
            )

            # ── Broadcast BAR static TF (identical to what was saved) ─────
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
