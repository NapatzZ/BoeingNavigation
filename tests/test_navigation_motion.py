"""Closed-loop motions against the simulated robot."""

import math

import pytest

from navigation_fakes import Harness
from skuba_navigation.core.navigation.clock import Deadline
from skuba_navigation.core.navigation.config import MotionConfig
from skuba_navigation.core.navigation.motion import (
    rotation_command,
    spin_command,
    YawTracker,
)
from skuba_navigation.core.navigation.types import ErrorCode, Pose2D, TaskFailure

CFG = MotionConfig()
TOL = math.radians(CFG.rotate_default_tolerance_deg)


def deadline(h, seconds=60.0):
    return Deadline(h.clock, seconds)


def test_rotation_command_sign_follows_error():
    assert rotation_command(math.radians(30), 0.0, CFG) > 0
    assert rotation_command(-math.radians(30), 0.0, CFG) < 0


def test_rotation_command_respects_speed_limits():
    for deg in (0.5, 5.0, 45.0, 179.0):
        for sign in (1, -1):
            speed = abs(rotation_command(sign * math.radians(deg), 0.0, CFG))
            assert CFG.rotate_min_speed - 1e-9 <= speed <= CFG.rotate_max_speed + 1e-9


def test_rotation_command_decel_zone_uses_min_speed():
    error = math.radians(CFG.rotate_decel_zone_deg) / 2
    assert rotation_command(error, 0.0, CFG) == pytest.approx(CFG.rotate_min_speed)


def test_rotation_damping_is_symmetric():
    # Closing in (|error| shrinking) must brake equally in both directions.
    error, rate = math.radians(20), -0.5
    mirrored = -rotation_command(-error, rate, CFG)
    assert rotation_command(error, rate, CFG) == pytest.approx(mirrored)


def test_spin_command_zones():
    deg = math.radians
    assert spin_command(deg(1.0), 0.0, CFG) == pytest.approx(CFG.spin_creep_speed)
    assert spin_command(deg(5.0), 0.0, CFG) == pytest.approx(CFG.spin_min_speed)
    assert spin_command(deg(90.0), 0.0, CFG) == pytest.approx(CFG.spin_max_speed)


def test_spin_command_d_term_adds_speed_while_closing_in():
    # P-Learn's sign: remaining shrinking (rate < 0) pushes the speed up.
    remaining = math.radians(12.0)
    assert spin_command(remaining, -0.5, CFG) > spin_command(remaining, 0.0, CFG)


@pytest.mark.parametrize("angle_deg", [90.0, -90.0, 180.0, -180.0, 270.0])
def test_spin_reaches_the_angle(angle_deg):
    h = Harness()
    achieved = h.motion.spin(math.radians(angle_deg), TOL, h.token, deadline(h))
    assert math.degrees(achieved) == pytest.approx(angle_deg, abs=2.5)
    assert not h.robot.moving and h.robot.stops >= 1


def test_spin_turns_the_commanded_way_past_180():
    h = Harness()
    turns = []
    h.clock.listeners.append(lambda dt: turns.append(h.robot.angular))
    h.motion.spin(math.radians(270), TOL, h.token, deadline(h))
    assert min(turns) >= 0.0  # never took the 90 deg short cut to the right


def test_spin_fails_and_stops_when_odometry_goes_quiet():
    h = Harness()
    h.clock.listeners.append(lambda dt: setattr(h.robot, "tf_down", h.clock.t > 0.5))
    with pytest.raises(TaskFailure) as failure:
        h.motion.spin(math.radians(180), TOL, h.token, deadline(h))
    assert failure.value.code == ErrorCode.TF_LOOKUP_FAILED
    assert not h.robot.moving


def test_cancel_stops_a_spin():
    h = Harness()
    h.clock.listeners.append(lambda dt: h.clock.t > 0.5 and h.token.cancel())
    with pytest.raises(TaskFailure) as failure:
        h.motion.spin(math.radians(180), TOL, h.token, deadline(h))
    assert failure.value.code == ErrorCode.CANCELED
    assert not h.robot.moving


def _stall_at_min_speed(h):
    # the real base: the floor holds it at spin_min_speed (run3, 20 s stuck)
    command = h.robot.command
    h.robot.command = lambda linear, angular: command(
        linear, angular if abs(angular) > CFG.spin_min_speed else 0.0
    )


def test_spin_stops_when_stalled_inside_the_goal_tolerance():
    h = Harness()
    _stall_at_min_speed(h)
    achieved = h.motion.spin(math.radians(90), math.radians(12), h.token, deadline(h))
    assert math.degrees(achieved) == pytest.approx(90.0, abs=12.0)
    assert h.clock.t < 5.0
    assert not h.robot.moving


def test_spin_keeps_pushing_when_stalled_outside_the_goal_tolerance():
    h = Harness()
    _stall_at_min_speed(h)
    with pytest.raises(TaskFailure) as failure:
        h.motion.spin(math.radians(90), math.radians(5), h.token, deadline(h, 5.0))
    assert failure.value.code == ErrorCode.TIMEOUT
    assert not h.robot.moving


def test_yaw_tracker_accumulates_across_wrap():
    tracker = YawTracker(0.0)
    for step in range(1, 28):  # 27 x 10 deg = 270 deg
        tracker.update(math.radians((10 * step + 180) % 360 - 180))
    assert math.degrees(tracker.turned) == pytest.approx(270.0)


@pytest.mark.parametrize("angle_deg", [90.0, -45.0, 270.0])
def test_rotate_by_reaches_the_angle(angle_deg):
    h = Harness()
    achieved = h.motion.rotate_by(
        math.radians(angle_deg), math.radians(2.0), h.token, deadline(h)
    )
    assert math.degrees(achieved) == pytest.approx(angle_deg, abs=2.5)
    assert not h.robot.moving and h.robot.stops >= 1


def test_turn_to_heading_takes_the_short_way():
    h = Harness(start=Pose2D(0.0, 0.0, math.radians(170)))
    error = h.motion.turn_to_heading(
        math.radians(-170), math.radians(2.0), h.token, deadline(h)
    )
    assert abs(error) < math.radians(2.5)
    assert h.clock.now() < 5.0  # 20 deg, not 340


@pytest.mark.parametrize("distance", [1.0, -0.5])
def test_drive_straight_reaches_signed_distance(distance):
    h = Harness(start=Pose2D(1.0, 1.0, math.radians(90)))
    achieved = h.motion.drive_straight(distance, 0.2, h.token, deadline(h))
    assert achieved == pytest.approx(distance, abs=CFG.drive_verify_tolerance)
    assert h.robot.pose.y == pytest.approx(
        1.0 + distance, abs=CFG.drive_verify_tolerance
    )
    assert not h.robot.moving


def test_cancel_stops_the_base():
    h = Harness()
    h.clock.listeners.append(lambda dt: h.clock.t > 0.5 and h.token.cancel())
    with pytest.raises(TaskFailure) as failure:
        h.motion.rotate_by(math.radians(180), math.radians(2.0), h.token, deadline(h))
    assert failure.value.code == ErrorCode.CANCELED
    assert not h.robot.moving


def test_deadline_stops_the_base():
    h = Harness()
    with pytest.raises(TaskFailure) as failure:
        h.motion.drive_straight(5.0, 0.2, h.token, deadline(h, 2.0))
    assert failure.value.code == ErrorCode.TIMEOUT
    assert not h.robot.moving


def test_long_tf_dropout_fails_instead_of_driving_blind():
    h = Harness()
    h.clock.listeners.append(lambda dt: setattr(h.robot, "tf_down", h.clock.t > 0.5))
    with pytest.raises(TaskFailure) as failure:
        h.motion.drive_straight(2.0, 0.2, h.token, deadline(h))
    assert failure.value.code == ErrorCode.TF_LOOKUP_FAILED
    assert not h.robot.moving


def test_short_tf_dropout_is_tolerated():
    h = Harness()
    h.clock.listeners.append(
        lambda dt: setattr(h.robot, "tf_down", 0.5 < h.clock.t < 0.7)
    )
    achieved = h.motion.drive_straight(0.5, 0.2, h.token, deadline(h))
    assert achieved == pytest.approx(0.5, abs=CFG.drive_verify_tolerance)
