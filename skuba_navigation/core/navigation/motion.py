"""
Closed-loop base motions on raw cmd_vel (no Nav2, no collision checking).

Every loop here:
  * checks cancel and the deadline on each tick,
  * holds (sends nothing) through a short TF dropout and fails with
    TF_LOOKUP_FAILED if it lasts longer than `tf_grace` - it never drives blind,
  * always stops the base on the way out, including on errors,
  * returns what was measured after stopping, so callers verify the result
    instead of assuming it.
"""

import math
from typing import Callable, Optional

from ..geometry_utils import normalize_angle
from .cancellation import CancelToken
from .clock import Clock, Deadline
from .config import MotionConfig
from .ports import BaseCommander, OdomSource, PoseSource
from .types import ErrorCode, Pose2D, TaskFailure

ProgressCallback = Optional[Callable[[float], None]]

# Lookup timeout for the one-off poses read before/after a loop.
_SNAPSHOT_TIMEOUT = 1.0


def rotation_command(error: float, error_rate: float, config: MotionConfig) -> float:
    """
    PD law for in-place rotation. Returns a signed angular speed (rad/s).

    `error` is the signed remaining angle, `error_rate` the rate of change of
    |error| (negative while closing in), so the D term brakes symmetrically in
    both directions. Inside the decel zone the base is held at min speed:
    below that floor friction stalls it before it reaches the target.
    """
    magnitude = abs(error) * config.rotate_kp + error_rate * config.rotate_kd
    if abs(error) < math.radians(config.rotate_decel_zone_deg):
        magnitude = config.rotate_min_speed
    magnitude = min(max(magnitude, config.rotate_min_speed), config.rotate_max_speed)
    return math.copysign(magnitude, error)


def spin_command(
    remaining: float, remaining_rate: float, config: MotionConfig
) -> float:
    """
    Speed magnitude (rad/s) of the P-Learn rotate loop, `remaining` > 0 rad.

    As in P-Learn the D term is `- remaining_rate * kd`: while closing in
    (rate < 0) it adds speed; the decel zone, not the D term, does the braking.
    """
    if remaining < math.radians(config.spin_creep_zone_deg):
        return config.spin_creep_speed
    if remaining < math.radians(config.spin_decel_zone_deg):
        return config.spin_min_speed
    control = remaining * config.spin_kp - remaining_rate * config.spin_kd
    return min(max(control, config.spin_min_speed), config.spin_max_speed)


def drive_command(remaining: float, speed: float, config: MotionConfig) -> float:
    """Forward speed magnitude for a straight drive with `remaining` metres left."""
    if remaining < config.drive_slow_zone:
        return speed * config.drive_slow_factor
    return speed


class YawTracker:
    """Accumulates heading change across the +/-pi wrap (so 270 deg turns work)."""

    def __init__(self, start_yaw: float):
        self._last = start_yaw
        self.turned = 0.0

    def update(self, yaw: float) -> float:
        self.turned += normalize_angle(yaw - self._last)
        self._last = yaw
        return self.turned


class BaseMotion:
    """Rotate / drive-straight controllers on top of the ports."""

    def __init__(
        self,
        poses: PoseSource,
        base: BaseCommander,
        config: MotionConfig,
        clock: Clock,
        odom: Optional[OdomSource] = None,
    ):
        self._poses = poses
        self._odom = odom
        self._base = base
        self._config = config
        self._clock = clock

    def rotate_by(
        self,
        angle: float,
        tolerance: float,
        token: CancelToken,
        deadline: Deadline,
        on_progress: ProgressCallback = None,
    ) -> float:
        """Turn by `angle` rad (any magnitude). Returns the measured rotation."""
        cfg = self._config
        tracker = YawTracker(self._poses.robot_pose(_SNAPSHOT_TIMEOUT).yaw)
        previous_error, previous_time = abs(angle), self._clock.now()
        try:
            for pose in self._samples(token, deadline, cfg.rotate_rate):
                if pose is None:
                    continue
                error = angle - tracker.update(pose.yaw)
                if on_progress:
                    on_progress(error)
                if abs(error) < tolerance:
                    break
                now = self._clock.now()
                dt = now - previous_time
                error_rate = (abs(error) - previous_error) / dt if dt > 0.0 else 0.0
                previous_error, previous_time = abs(error), now
                self._base.command(0.0, rotation_command(error, error_rate, cfg))
        finally:
            self._base.stop()
        self._clock.sleep(cfg.rotate_settle_time, token)
        return tracker.update(self._poses.robot_pose(_SNAPSHOT_TIMEOUT).yaw)

    def spin(
        self,
        angle: float,
        tolerance: float,
        token: CancelToken,
        deadline: Deadline,
        on_progress: ProgressCallback = None,
    ) -> float:
        """
        Turn by `angle` rad on odometry: the P-Learn motion_controller.rotate loop.

        Drives the commanded direction the whole way and counts only progress
        that way, so angles past 180 deg are not short-cut. Returns the
        measured rotation (start-to-end yaw, unwrapped next to `angle`).

        Stops early when the base stalls (no `spin_stall_deg` of progress for
        `spin_stall_time`) already within `tolerance` rad of the target: at min
        speed friction can hold it there, and waiting only burns the timeout.
        """
        cfg = self._config
        if self._odom is None:
            raise TaskFailure(ErrorCode.TF_LOOKUP_FAILED, "No odometry source.")
        direction = 1.0 if angle >= 0.0 else -1.0
        target = abs(angle)
        stop = math.radians(cfg.spin_stop_tolerance_deg)
        stall_step = math.radians(cfg.spin_stall_deg)
        period = 1.0 / cfg.spin_rate
        start_yaw = previous_yaw = self._odom.yaw(_SNAPSHOT_TIMEOUT)
        turned = 0.0
        previous_remaining, previous_time = target, self._clock.now()
        last_seen = progress_time = previous_time
        progress_turned = 0.0
        try:
            while True:
                token.raise_if_cancelled()
                if deadline.expired():
                    raise TaskFailure(ErrorCode.TIMEOUT, "Motion timed out.")
                try:
                    yaw = self._odom.yaw(period)
                    last_seen = self._clock.now()
                except TaskFailure:
                    if self._clock.now() - last_seen > cfg.tf_grace:
                        raise
                    self._clock.sleep(period, token)
                    continue
                turned += max(normalize_angle(yaw - previous_yaw) * direction, 0.0)
                previous_yaw = yaw
                remaining = target - turned
                if on_progress:
                    on_progress(direction * remaining)
                if remaining < stop:
                    break
                now = self._clock.now()
                # a threshold, not "any growth": odom noise alone keeps `turned` creeping up
                if turned - progress_turned > stall_step:
                    progress_turned, progress_time = turned, now
                elif (
                    remaining < tolerance and now - progress_time > cfg.spin_stall_time
                ):
                    break
                dt = now - previous_time
                rate = (remaining - previous_remaining) / dt if dt > 0.0 else 0.0
                previous_remaining, previous_time = remaining, now
                self._base.command(0.0, direction * spin_command(remaining, rate, cfg))
                self._clock.sleep(period, token)
        finally:
            self._base.stop()
        self._clock.sleep(cfg.spin_settle_time, token)
        moved = normalize_angle(self._odom.yaw(_SNAPSHOT_TIMEOUT) - start_yaw)
        return angle + normalize_angle(moved - angle)

    def turn_to_heading(
        self,
        heading: float,
        tolerance: float,
        token: CancelToken,
        deadline: Deadline,
        on_progress: ProgressCallback = None,
    ) -> float:
        """Turn to an absolute map heading. Returns the final heading error (rad)."""
        error = normalize_angle(heading - self._poses.robot_pose(_SNAPSHOT_TIMEOUT).yaw)
        if abs(error) <= tolerance:
            return error
        self.rotate_by(error, tolerance, token, deadline, on_progress)
        return normalize_angle(heading - self._poses.robot_pose(_SNAPSHOT_TIMEOUT).yaw)

    def face_point(
        self,
        x: float,
        y: float,
        tolerance: float,
        token: CancelToken,
        deadline: Deadline,
    ) -> float:
        """Turn to look at (x, y). Returns the final heading error (rad)."""
        heading = self._poses.robot_pose(_SNAPSHOT_TIMEOUT).bearing_to(x, y)
        return self.turn_to_heading(heading, tolerance, token, deadline)

    def drive_straight(
        self,
        distance: float,
        speed: float,
        token: CancelToken,
        deadline: Deadline,
        on_progress: ProgressCallback = None,
    ) -> float:
        """
        Drive `distance` m along the current heading (negative = backward).

        Progress is measured as the projection onto the starting heading, so
        the returned distance is signed and sideways drift does not count.
        """
        cfg = self._config
        start = self._poses.robot_pose(_SNAPSHOT_TIMEOUT)
        direction = 1.0 if distance >= 0.0 else -1.0
        target = abs(distance)
        try:
            for pose in self._samples(token, deadline, cfg.drive_rate):
                if pose is None:
                    continue
                remaining = target - direction * _travelled(start, pose)
                if on_progress:
                    on_progress(max(remaining, 0.0))
                if remaining <= cfg.drive_stop_tolerance:
                    break
                self._base.command(
                    direction * drive_command(remaining, speed, cfg), 0.0
                )
        finally:
            self._base.stop()
        self._clock.sleep(cfg.drive_settle_time, token)
        return _travelled(start, self._poses.robot_pose(_SNAPSHOT_TIMEOUT))

    def _samples(self, token: CancelToken, deadline: Deadline, rate: float):
        """Yield one robot pose per control tick (None during a short TF dropout)."""
        period = 1.0 / rate
        last_seen = self._clock.now()
        while True:
            token.raise_if_cancelled()
            if deadline.expired():
                raise TaskFailure(ErrorCode.TIMEOUT, "Motion timed out.")
            try:
                pose = self._poses.robot_pose(period)
                last_seen = self._clock.now()
            except TaskFailure:
                if self._clock.now() - last_seen > self._config.tf_grace:
                    raise
                pose = None
            yield pose
            self._clock.sleep(period, token)


def _travelled(start: Pose2D, pose: Pose2D) -> float:
    """Signed distance from `start` to `pose` along the starting heading."""
    return (pose.x - start.x) * math.cos(start.yaw) + (pose.y - start.y) * math.sin(
        start.yaw
    )
