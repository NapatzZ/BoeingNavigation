"""Pure door open/closed detection logic for door_detector.py. No rclpy import."""

import math

import numpy as np


class DoorDetector:
    """
    Forward-cone beam classification + debounce state machine.

    Door CLOSED : most front beams return distance < threshold
                  (the door surface reflects the laser back)
    Door OPEN   : most front beams return distance >= threshold or inf
                  (the laser passes through the open doorway)
    """

    def __init__(
        self,
        threshold,
        front_center_rad,
        front_rad,
        open_ratio,
        min_beams,
        confirm_need,
    ):
        self.threshold = threshold
        self.front_center_rad = front_center_rad
        self.front_rad = front_rad
        self.open_ratio = open_ratio
        self.min_beams = min_beams
        self.confirm_need = confirm_need

        self.active = False
        self.door_open = False
        self._pending_open = False
        self._confirm_streak = 0

    def set_active(self, active: bool) -> None:
        self.active = active
        if not active:
            self.door_open = False
            self._pending_open = False
            self._confirm_streak = 0

    def process_scan(self, ranges, angle_min, angle_increment, range_min):
        """
        Feed one scan through the detector.

        Returns (status, detail):
          ('no_beams', None)               - forward cone empty
          ('too_few_beams', valid_count)   - not enough valid beams
          ('state_change', 'OPEN'|'CLOSED')- confirmed door state changed
          (None, open_ratio)               - scan processed, no state change

        `ranges` may be a list or numpy array of float distances.
        """
        num_beams = len(ranges)
        angles = np.fromiter(
            (angle_min + i * angle_increment for i in range(num_beams)),
            dtype=np.float64,
            count=num_beams,
        )
        # Circular difference wrapped to [-pi, pi], so the cone works at any
        # center angle, including 180 deg (swapped LiDAR mount).
        diff = (angles - self.front_center_rad + math.pi) % (2.0 * math.pi) - math.pi
        front_indices = np.where(np.abs(diff) <= self.front_rad)[0]

        if front_indices.size == 0:
            return "no_beams", None

        front_ranges = np.array(ranges)[front_indices]

        valid_mask = np.isfinite(front_ranges) & (front_ranges >= range_min)
        inf_open_mask = np.isposinf(front_ranges)
        valid_ranges = front_ranges[valid_mask]

        if valid_ranges.size < self.min_beams:
            return "too_few_beams", int(valid_ranges.size)

        open_finite = int(np.sum(valid_ranges >= self.threshold))
        open_inf = int(np.sum(inf_open_mask))
        total_open = open_finite + open_inf
        total_count = valid_ranges.size + open_inf

        open_ratio = total_open / total_count if total_count > 0 else 0.0
        candidate = open_ratio >= self.open_ratio

        if candidate == self._pending_open:
            self._confirm_streak += 1
        else:
            self._pending_open = candidate
            self._confirm_streak = 1

        if self._confirm_streak >= self.confirm_need and candidate != self.door_open:
            self.door_open = candidate
            return "state_change", (
                "OPEN" if self.door_open else "CLOSED",
                open_ratio,
                total_open,
                total_count,
            )

        return None, open_ratio
