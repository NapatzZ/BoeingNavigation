"""
The head camera's cloud without the robot itself, for the costmaps' depth layer.

Glue only: core/self_filter.py decides which points are the robot.

    in:  cloud_topic (default /camera/camera/depth/color/points)
         /local_costmap/published_footprint (grown by arm_footprint round the arm
         and what the hand carries)
    out: /skuba/nav/filtered_points (same frame and stamp as the input)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from ..core.self_filter import not_the_robot

OUT_TOPIC = "/skuba/nav/filtered_points"


def _matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class CloudSelfFilter(Node):
    def __init__(self):
        super().__init__("cloud_self_filter")
        self._base = self.declare_parameter("base_frame", "base_footprint").value
        self._margin = float(self.declare_parameter("margin_m", 0.12).value)
        cloud_topic = self.declare_parameter(
            "cloud_topic", "/camera/camera/depth/color/points"
        ).value
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._footprint = None  # (k x 2), base frame
        self.create_subscription(
            PolygonStamped, "/local_costmap/published_footprint", self._on_footprint, 1
        )
        self.create_subscription(
            PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(
            PointCloud2, OUT_TOPIC, qos_profile_sensor_data
        )
        self.get_logger().info(f"cloud_self_filter ready: {cloud_topic} -> {OUT_TOPIC}")

    def _to_base(self, frame):
        try:
            t = self._tf.lookup_transform(self._base, frame, Time())
        except TransformException:
            return None
        p = t.transform.translation
        return _matrix(t.transform.rotation), np.array([p.x, p.y, p.z])

    def _on_footprint(self, msg):
        tf = self._to_base(msg.header.frame_id)
        if tf is None:
            return
        rot, trans = tf
        pts = np.array([[p.x, p.y, p.z] for p in msg.polygon.points])
        if len(pts) >= 3:
            self._footprint = (pts @ rot.T + trans)[:, :2]

    def _on_cloud(self, msg):
        tf = self._to_base(msg.header.frame_id)
        xyz = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if tf is None or self._footprint is None or xyz.size == 0:
            # Nothing known of the robot yet: pass the cloud through.
            self._pub.publish(msg)
            return
        rot, trans = tf
        keep = not_the_robot(xyz @ rot.T + trans, self._footprint, self._margin)
        self._pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, xyz[keep].astype(np.float32))
        )


def main(args=None):
    rclpy.init(args=args)
    node = CloudSelfFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
