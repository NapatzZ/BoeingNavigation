"""Pure person-cluster removal + fixed-grid resampling for /scan_slam. No rclpy import."""
import math

from sensor_msgs.msg import LaserScan

# Fixed output resolution for /scan_slam.
# YDLidar X2L produces 451-453 points/scan depending on timing jitter.
# SLAM Toolbox rejects any scan where len(ranges) != round((max-min)/inc)+1,
# so we resample every scan to exactly this many uniformly-spaced points.
OUT_N = 360
OUT_ANGLE_MIN = -math.pi
OUT_ANGLE_MAX = math.pi
OUT_ANGLE_INC = (OUT_ANGLE_MAX - OUT_ANGLE_MIN) / (OUT_N - 1)


def remove_person_clusters(msg: LaserScan, min_width, max_width, cluster_gap, min_pts):
    """Blank out person-sized clusters in `msg.ranges`, resample to a fixed grid.

    Returns a new LaserScan on the fixed OUT_N-point angular grid.
    """
    ranges = list(msg.ranges)
    angle_min = msg.angle_min
    da = msg.angle_increment

    valid = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and msg.range_min < r < msg.range_max:
            a = angle_min + i * da
            valid.append((i, r * math.cos(a), r * math.sin(a)))

    if not valid:
        return msg

    clusters = []
    cluster = [valid[0]]
    for i in range(1, len(valid)):
        px, py = valid[i - 1][1], valid[i - 1][2]
        cx, cy = valid[i][1], valid[i][2]
        if math.hypot(cx - px, cy - py) <= cluster_gap:
            cluster.append(valid[i])
        else:
            clusters.append(cluster)
            cluster = [valid[i]]
    clusters.append(cluster)

    remove = set()
    for cl in clusters:
        if len(cl) < min_pts:
            continue
        xs = [p[1] for p in cl]
        ys = [p[2] for p in cl]
        width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if min_width <= width <= max_width:
            for p in cl:
                remove.add(p[0])

    new_ranges = ranges[:]
    for idx in remove:
        new_ranges[idx] = math.inf

    # Nearest-neighbour resample onto OUT_N uniformly-spaced angles guarantees
    # a consistent point count on every scan.
    resampled: list[float] = []
    n_src = len(new_ranges)
    for i in range(OUT_N):
        target_angle = OUT_ANGLE_MIN + i * OUT_ANGLE_INC
        src_idx = round((target_angle - msg.angle_min) / msg.angle_increment)
        src_idx = max(0, min(n_src - 1, src_idx))
        resampled.append(new_ranges[src_idx])

    out = LaserScan()
    out.header = msg.header
    out.angle_min = OUT_ANGLE_MIN
    out.angle_max = OUT_ANGLE_MAX
    out.angle_increment = OUT_ANGLE_INC
    out.time_increment = msg.time_increment
    out.scan_time = msg.scan_time
    out.range_min = msg.range_min
    out.range_max = msg.range_max
    out.ranges = resampled
    out.intensities = []
    return out
