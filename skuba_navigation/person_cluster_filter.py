import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from skuba_navigation.person_cluster import remove_person_clusters


class PersonClusterFilter(Node):
    """
    Removes person-sized clusters from /scan before SLAM Toolbox
    so walking people are not baked into the occupancy map.

    People are still visible on /scan_filtered (used by STVL costmap),
    so the robot continues to detect and avoid them during navigation.

    Tunable parameters (ros2 param set or yaml):
      min_person_width  – smallest cluster diagonal treated as a person [m]
      max_person_width  – largest cluster diagonal treated as a person [m]
      cluster_gap       – max gap between consecutive points in one cluster [m]
      min_cluster_pts   – discard clusters with fewer points (noise guard)
    """

    def __init__(self):
        super().__init__('person_cluster_filter')

        self.declare_parameter('min_person_width', 0.25)
        self.declare_parameter('max_person_width', 0.85)
        self.declare_parameter('cluster_gap', 0.20)
        self.declare_parameter('min_cluster_pts', 3)

        self.min_width = self.get_parameter('min_person_width').value
        self.max_width = self.get_parameter('max_person_width').value
        self.cluster_gap = self.get_parameter('cluster_gap').value
        self.min_pts = self.get_parameter('min_cluster_pts').value

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, '/scan_slam', 10)

        self.get_logger().info(
            f'PersonClusterFilter ready: person width [{self.min_width}m – {self.max_width}m], '
            f'gap={self.cluster_gap}m, min_pts={self.min_pts}'
        )

    def scan_callback(self, msg: LaserScan):
        out = remove_person_clusters(
            msg, self.min_width, self.max_width, self.cluster_gap, self.min_pts)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonClusterFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
