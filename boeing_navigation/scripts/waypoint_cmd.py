#!/usr/bin/env python3
"""Command-line helper for waypoint_manager.

    ros2 run boeing_navigation waypoint_cmd save kitchen
    ros2 run boeing_navigation waypoint_cmd goto kitchen
    ros2 run boeing_navigation waypoint_cmd route kitchen door home
    ros2 run boeing_navigation waypoint_cmd list

It publishes the words as one command string to /waypoint_manager/command and
prints the next status message(s) from /waypoint_manager/status.
"""
import shlex
import sys
import time

import rclpy
from std_msgs.msg import String


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if not args:
        print(__doc__)
        return 1

    rclpy.init()
    node = rclpy.create_node('waypoint_cmd')
    replies = []
    node.create_subscription(String, '/waypoint_manager/status',
                             lambda m: replies.append(m.data), 10)
    pub = node.create_publisher(String, '/waypoint_manager/command', 10)

    # Wait until waypoint_manager is discovered (a --once publish would otherwise be lost).
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('waypoint_manager is not running (nothing subscribed to /waypoint_manager/command)')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # Give our status subscription a moment to match before sending.
    end = time.time() + 0.5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    pub.publish(String(data=' '.join(shlex.quote(a) for a in args)))

    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for r in replies:
        print(r)
    if not replies:
        print('(sent - see the waypoint_manager terminal for the result)')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
