import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Fixed output resolution for /scan_slam.
# YDLidar X2L produces 451-453 points/scan depending on timing jitter.
# SLAM Toolbox rejects any scan where len(ranges) != round((max-min)/inc)+1,
# so we resample every scan to exactly this many uniformly-spaced points.
_OUT_N = 360
_OUT_ANGLE_MIN = -math.pi
_OUT_ANGLE_MAX = math.pi
_OUT_ANGLE_INC = (_OUT_ANGLE_MAX - _OUT_ANGLE_MIN) / (_OUT_N - 1)


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
        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        da = msg.angle_increment

        # --- Build list of valid points (index, x, y) ---
        valid = []
        for i, r in enumerate(ranges):
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                a = angle_min + i * da
                valid.append((i, r * math.cos(a), r * math.sin(a)))

        if not valid:
            self.pub.publish(msg)
            return

        # --- Sequential clustering on ordered scan points ---
        clusters = []
        cluster = [valid[0]]
        for i in range(1, len(valid)):
            px, py = valid[i - 1][1], valid[i - 1][2]
            cx, cy = valid[i][1], valid[i][2]
            if math.hypot(cx - px, cy - py) <= self.cluster_gap:
                cluster.append(valid[i])
            else:
                clusters.append(cluster)
                cluster = [valid[i]]
        clusters.append(cluster)

        # --- Find person-sized clusters and collect scan indices to blank ---
        remove = set()
        for cl in clusters:
            if len(cl) < self.min_pts:
                continue
            xs = [p[1] for p in cl]
            ys = [p[2] for p in cl]
            width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if self.min_width <= width <= self.max_width:
                for p in cl:
                    remove.add(p[0])

        # --- Apply person filter ---
        new_ranges = ranges[:]
        for idx in remove:
            new_ranges[idx] = math.inf

        # --- Resample to fixed angular grid so SLAM Toolbox never rejects the scan ---
        # The YDLidar X2L produces 451-453 points per rotation due to timing jitter.
        # SLAM Toolbox checks len(ranges) == round((max-min)/inc)+1 and drops mismatches.
        # Nearest-neighbour resample onto _OUT_N uniformly-spaced angles guarantees
        # a consistent count on every scan.
        resampled: list[float] = []
        n_src = len(new_ranges)
        for i in range(_OUT_N):
            target_angle = _OUT_ANGLE_MIN + i * _OUT_ANGLE_INC
            src_idx = round((target_angle - msg.angle_min) / msg.angle_increment)
            src_idx = max(0, min(n_src - 1, src_idx))
            resampled.append(new_ranges[src_idx])

        out = LaserScan()
        out.header = msg.header
        out.angle_min = _OUT_ANGLE_MIN
        out.angle_max = _OUT_ANGLE_MAX
        out.angle_increment = _OUT_ANGLE_INC
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = resampled
        out.intensities = []
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PersonClusterFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
