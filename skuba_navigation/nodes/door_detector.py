#!/usr/bin/env python3
"""
Door Detector Node for SKUBA robot.

Monitors the forward-facing cone of the /scan LaserScan topic to detect
whether a door in front of the robot is open or closed. Detection logic
lives in door_detection.py.

Interfaces
----------
  Subscriber  : /scan                               (sensor_msgs/LaserScan)
  Publisher   : /skuba/nav/door_status        (std_msgs/Bool)
  Service     : /skuba/nav/door_detector/toggle  (std_srvs/SetBool)
                  data=true  → activate detection
                  data=false → deactivate detection

Parameters
----------
  door_threshold    (float, default 1.5)   Distance [m] separating open vs closed.
  front_angle_deg   (float, default 20.0)  Half-width [deg] of the forward cone.
  open_ratio        (float, default 0.7)   Fraction of valid beams that must exceed
                                           the threshold to declare door open.
  min_valid_beams   (int,   default 3)     Minimum valid beam count; fewer beams
                                           means the result is unreliable.
  confirm_count     (int,   default 3)     Consecutive scan confirmations required
                                           before the published state changes.
                                           Filters out momentary sensor noise.
  publish_rate      (float, default 10.0)  Door-status topic publish rate [Hz].

"""

import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from ..core.door_detection import DoorDetector

NAMESPACE = "skuba/nav"


class DoorDetectorNode(Node):
    """
    ROS 2 node that publishes whether the door in front of the robot is open.

    The node starts inactive. Call the toggle service with data=true to begin
    detection, and data=false to stop.
    """

    def __init__(self):
        super().__init__("door_detector")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("door_threshold", 1.5)
        self.declare_parameter("front_center_deg", 180.0)  # 0=normal mount, 180=swapped
        self.declare_parameter("front_angle_deg", 20.0)
        self.declare_parameter("open_ratio", 0.7)
        self.declare_parameter("min_valid_beams", 3)
        self.declare_parameter("confirm_count", 3)
        self.declare_parameter("publish_rate", 10.0)

        front_center_rad = math.radians(self.get_parameter("front_center_deg").value)
        front_rad = math.radians(self.get_parameter("front_angle_deg").value)
        publish_rate = self.get_parameter("publish_rate").value

        self._detector = DoorDetector(
            threshold=self.get_parameter("door_threshold").value,
            front_center_rad=front_center_rad,
            front_rad=front_rad,
            open_ratio=self.get_parameter("open_ratio").value,
            min_beams=self.get_parameter("min_valid_beams").value,
            confirm_need=self.get_parameter("confirm_count").value,
        )

        # ── Subscriber ────────────────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(Bool, f"/{NAMESPACE}/door_status", 10)

        # ── Service ───────────────────────────────────────────────────────────
        self._toggle_srv = self.create_service(
            SetBool, f"/{NAMESPACE}/door_detector/toggle", self._toggle_callback
        )

        # ── Publish timer ─────────────────────────────────────────────────────
        self._pub_timer = self.create_timer(1.0 / publish_rate, self._publish_status)

        self.get_logger().info("Door Detector Node initialized  [INACTIVE]")
        self.get_logger().info(f"  threshold     : {self._detector.threshold:.2f} m")
        reversed_mount = math.isclose(math.degrees(front_center_rad), 180.0)
        mount = "swapped — LiDAR mounted reversed" if reversed_mount else "normal mount"
        self.get_logger().info(
            f"  front center  : {math.degrees(front_center_rad):.1f}°  ({mount})"
        )
        self.get_logger().info(f"  front cone    : ±{math.degrees(front_rad):.1f}°")
        self.get_logger().info(f"  open_ratio    : {self._detector.open_ratio:.0%}")
        self.get_logger().info(f"  confirm_count : {self._detector.confirm_need} scans")
        self.get_logger().info(
            f"Service  : /{NAMESPACE}/door_detector/toggle  (std_srvs/SetBool)"
        )
        self.get_logger().info(
            f"Topic    : /{NAMESPACE}/door_status           (std_msgs/Bool)"
        )

    # =========================================================================
    # Service callback
    # =========================================================================

    def _toggle_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self._detector.set_active(request.data)

        state_str = "ACTIVE" if self._detector.active else "INACTIVE"
        response.success = True
        response.message = f"Door detector is now {state_str}"
        self.get_logger().info(response.message)
        return response

    # =========================================================================
    # Scan callback
    # =========================================================================

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._detector.active:
            return

        status, detail = self._detector.process_scan(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min
        )

        if status == "no_beams":
            self.get_logger().warn(
                "No laser beams found in the forward cone — "
                f"check front_angle_deg parameter (current: "
                f"{math.degrees(self._detector.front_rad):.1f}°)"
            )
        elif status == "too_few_beams":
            self.get_logger().warn(
                f"Too few valid beams in forward cone: "
                f"{detail} / {self._detector.min_beams} required — skipping scan"
            )
        elif status == "state_change":
            state_str, open_ratio, total_open, total_count = detail
            self.get_logger().info(
                f"Door state changed → {state_str}  "
                f"(open_ratio={open_ratio:.0%}, "
                f"open_beams={total_open}/{total_count})"
            )

    # =========================================================================
    # Publish timer callback
    # =========================================================================

    def _publish_status(self) -> None:
        if not self._detector.active:
            return
        msg = Bool()
        msg.data = self._detector.door_open
        self._status_pub.publish(msg)


# =============================================================================
# Entry point
# =============================================================================


def main(args=None):
    rclpy.init(args=args)
    node = DoorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
