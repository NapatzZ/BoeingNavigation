"""BaseCommander on a Twist publisher.

Publishes on the manual twist_mux lane (cmd_vel_nav_manual by default, see
params/twist_mux.yaml), not on Nav2's cmd_vel_nav, so it never mixes with
collision_monitor's filtered Nav2 output.
"""
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelBase:

    def __init__(self, node: Node, topic: str, stop_repeats: int, stop_interval: float = 0.02):
        self._publisher = node.create_publisher(Twist, topic, 10)
        self._stop_repeats = max(1, stop_repeats)
        self._stop_interval = stop_interval

    def command(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self._publisher.publish(twist)

    def stop(self) -> None:
        # Several zeros: the heavy base (and twist_mux) must not miss the stop.
        for _ in range(self._stop_repeats):
            self._publisher.publish(Twist())
            time.sleep(self._stop_interval)
