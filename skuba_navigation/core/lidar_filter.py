"""Pure scan-filtering logic for filter_lidar.py. No rclpy import."""

import copy
import numpy as np

_ANGLE_START_1 = np.deg2rad(260)
_ANGLE_END_1 = 2 * np.pi
_ANGLE_START_2 = 0.0
_ANGLE_END_2 = np.deg2rad(100)
_MIN_RANGE = 0.5


class LidarFilter:
    """Holds the angle-filter on/off mode and filters LaserScan messages."""

    def __init__(self):
        self.enable_angle_filter = False

    def set_mode(self, enable: bool) -> str:
        """Set the angle-filter mode. Returns a human-readable mode string."""
        self.enable_angle_filter = enable
        if enable:
            return "angle filter  + distance filter"
        return "distance filter only"

    def filter_scan(self, msg):
        """Return a filtered copy of `msg` (sensor_msgs/LaserScan)."""
        filtered_msg = copy.deepcopy(msg)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i in range(len(filtered_msg.ranges)):
            current_range = filtered_msg.ranges[i]
            if np.isinf(current_range) or np.isnan(current_range):
                continue
            angle = angle_min + i * angle_increment
            angle = angle % (2 * np.pi)
            if current_range < _MIN_RANGE:
                filtered_msg.ranges[i] = np.inf
                continue
            if self.enable_angle_filter:
                in_front_sector = (_ANGLE_START_1 <= angle <= _ANGLE_END_1) or (
                    _ANGLE_START_2 <= angle <= _ANGLE_END_2
                )
                if in_front_sector:
                    filtered_msg.ranges[i] = np.inf

        return filtered_msg
