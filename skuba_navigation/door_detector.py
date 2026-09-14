#!/usr/bin/env python3
"""
Door Detector Node for SKUBA robot.

Monitors the forward-facing cone of the /scan LaserScan topic to detect
whether a door in front of the robot is open or closed.

Logic
-----
  Door CLOSED : most front beams return distance < threshold
                (the door surface reflects the laser back)
  Door OPEN   : most front beams return distance >= threshold or inf
                (the laser passes through the open doorway)

Interfaces
----------
  Subscriber  : /scan                              (sensor_msgs/LaserScan)
  Publisher   : skuba/cheesecake/nav/door_status  (std_msgs/Bool)
  Service     : skuba/cheesecake/nav/door_detector/toggle  (std_srvs/SetBool)
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

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

NAMESPACE = 'skuba/cheesecake/nav'


class DoorDetectorNode(Node):
    """
    ROS 2 node that publishes whether the door in front of the robot is open.

    The node starts inactive. Call the toggle service with data=true to begin
    detection, and data=false to stop.
    """

    def __init__(self):
        super().__init__('door_detector')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('door_threshold',   1.5)
        self.declare_parameter('front_center_deg', 180.0)   # 0=normal mount, 180=swapped
        self.declare_parameter('front_angle_deg',  20.0)
        self.declare_parameter('open_ratio',       0.7)
        self.declare_parameter('min_valid_beams',  3)
        self.declare_parameter('confirm_count',    3)
        self.declare_parameter('publish_rate',     10.0)

        self._threshold       = self.get_parameter('door_threshold').value
        self._front_center_rad = math.radians(self.get_parameter('front_center_deg').value)
        self._front_rad       = math.radians(self.get_parameter('front_angle_deg').value)
        self._open_ratio   = self.get_parameter('open_ratio').value
        self._min_beams    = self.get_parameter('min_valid_beams').value
        self._confirm_need = self.get_parameter('confirm_count').value
        publish_rate       = self.get_parameter('publish_rate').value

        # ── Internal state ────────────────────────────────────────────────────
        self._active          = False   # detection on/off
        self._door_open       = False   # current confirmed state
        self._pending_open    = False   # candidate state being confirmed
        self._confirm_streak  = 0       # consecutive confirmations so far

        # ── Subscriber ────────────────────────────────────────────────────────
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos_profile_sensor_data
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(
            Bool,
            f'{NAMESPACE}/door_status',
            10
        )

        # ── Service ───────────────────────────────────────────────────────────
        self._toggle_srv = self.create_service(
            SetBool,
            f'{NAMESPACE}/door_detector',
            self._toggle_callback
        )

        # ── Publish timer ─────────────────────────────────────────────────────
        self._pub_timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_status
        )

        self.get_logger().info('Door Detector Node initialized  [INACTIVE]')
        self.get_logger().info(
            f'  threshold     : {self._threshold:.2f} m'
        )
        self.get_logger().info(
            f'  front center  : {math.degrees(self._front_center_rad):.1f}°'
            f'  ({"swapped — LiDAR mounted reversed" if math.isclose(math.degrees(self._front_center_rad), 180.0) else "normal mount"})'
        )
        self.get_logger().info(
            f'  front cone    : ±{math.degrees(self._front_rad):.1f}°'
        )
        self.get_logger().info(
            f'  open_ratio    : {self._open_ratio:.0%}'
        )
        self.get_logger().info(
            f'  confirm_count : {self._confirm_need} scans'
        )
        self.get_logger().info(
            f'Service  : {NAMESPACE}/door_detector/toggle  (std_srvs/SetBool)'
        )
        self.get_logger().info(
            f'Topic    : {NAMESPACE}/door_status           (std_msgs/Bool)'
        )

    # =========================================================================
    # Service callback
    # =========================================================================

    def _toggle_callback(self, request: SetBool.Request,
                         response: SetBool.Response) -> SetBool.Response:
        self._active = request.data

        if not self._active:
            # Reset all state when deactivated so detection starts fresh next time
            self._door_open      = False
            self._pending_open   = False
            self._confirm_streak = 0

        state_str = 'ACTIVE' if self._active else 'INACTIVE'
        response.success = True
        response.message = f'Door detector is now {state_str}'
        self.get_logger().info(response.message)
        return response

    # =========================================================================
    # Scan callback — core detection logic
    # =========================================================================

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._active:
            return

        # ── 1. Find indices inside the forward cone ───────────────────────────
        # Use circular angle difference so the cone works correctly at any
        # center angle, including 180° (swapped LiDAR mount).
        # Formula: |wrap(angle - center)| <= half_width
        #   where wrap maps the difference to [-π, +π].
        num_beams = len(msg.ranges)
        angles = np.fromiter(
            (msg.angle_min + i * msg.angle_increment for i in range(num_beams)),
            dtype=np.float64,
            count=num_beams
        )
        # Circular difference wrapped to [-π, π]
        diff = (angles - self._front_center_rad + math.pi) % (2.0 * math.pi) - math.pi
        front_indices = np.where(np.abs(diff) <= self._front_rad)[0]

        if front_indices.size == 0:
            self.get_logger().warn(
                'No laser beams found in the forward cone — '
                f'check front_angle_deg parameter (current: '
                f'{math.degrees(self._front_rad):.1f}°)'
            )
            return

        front_ranges = np.array(msg.ranges)[front_indices]

        # ── 2. Classify each beam ─────────────────────────────────────────────
        # Positive inf  → beam reached max range with no return (open space)
        # Finite value  → actual distance measurement
        # NaN or -inf   → invalid reading, skip
        valid_mask    = np.isfinite(front_ranges) & (front_ranges >= msg.range_min)
        inf_open_mask = np.isposinf(front_ranges)

        valid_ranges = front_ranges[valid_mask]

        if valid_ranges.size < self._min_beams:
            self.get_logger().warn(
                f'Too few valid beams in forward cone: '
                f'{valid_ranges.size} / {self._min_beams} required — skipping scan'
            )
            return

        # ── 3. Count "open" beams ─────────────────────────────────────────────
        # A finite beam is "open" if it exceeds the threshold distance
        # An inf beam is always "open"
        open_finite = int(np.sum(valid_ranges >= self._threshold))
        open_inf    = int(np.sum(inf_open_mask))
        total_open  = open_finite + open_inf
        total_count = valid_ranges.size + open_inf

        open_ratio = total_open / total_count if total_count > 0 else 0.0
        candidate  = open_ratio >= self._open_ratio

        # ── 4. Debounce: require N consecutive matching readings ───────────────
        if candidate == self._pending_open:
            self._confirm_streak += 1
        else:
            # Candidate changed — restart the confirmation streak
            self._pending_open   = candidate
            self._confirm_streak = 1

        if self._confirm_streak >= self._confirm_need:
            if candidate != self._door_open:
                self._door_open = candidate
                status = 'OPEN' if self._door_open else 'CLOSED'
                self.get_logger().info(
                    f'Door state changed → {status}  '
                    f'(open_ratio={open_ratio:.0%}, '
                    f'open_beams={total_open}/{total_count})'
                )

    # =========================================================================
    # Publish timer callback
    # =========================================================================

    def _publish_status(self) -> None:
        if not self._active:
            return
        msg = Bool()
        msg.data = self._door_open
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


if __name__ == '__main__':
    main()
