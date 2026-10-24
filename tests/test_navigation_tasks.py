"""End-to-end task behaviour with fakes (what each action does)."""

import math

import pytest

from navigation_fakes import Harness
from skuba_navigation.core.navigation import tasks
from skuba_navigation.core.navigation.location_store import Location, MapLocations
from skuba_navigation.core.navigation.ports import Nav2Status
from skuba_navigation.core.navigation.types import ErrorCode, Pose2D, TaskFailure


def test_go_to_pose_reports_attempts_and_final_pose():
    h = Harness()
    result = tasks.go_to_pose(h.context(), Pose2D(1.0, 2.0, 0.5), "", 30.0, "")
    assert result.succeeded
    assert result.fields["attempts"] == 1
    assert result.fields["final_pose"] == Pose2D(1.0, 2.0, 0.5)
    assert h.nav2.goals[0][1] == "map"  # empty frame_id defaults to map


def test_go_to_location_uses_the_stored_pose(tmp_path):
    h = Harness(tmp_path)
    h.locations.save("kitchen", Location("map", Pose2D(3.0, -1.0, 1.0)))
    result = tasks.go_to_location(h.context(), " kitchen ", 30.0, "")
    assert result.succeeded
    goal = h.nav2.goals[0][0]
    assert (goal.x, goal.y, goal.yaw) == pytest.approx((3.0, -1.0, 1.0))


def test_go_to_location_without_a_map_fails_as_not_found(tmp_path):
    h = Harness(tmp_path)
    h.locations = MapLocations(lambda: None)
    ctx = h.context()
    with pytest.raises(TaskFailure) as err:
        tasks.go_to_location(ctx, "kitchen", 30.0, "")
    assert err.value.code == ErrorCode.LOCATION_NOT_FOUND
    assert "no map loaded" in err.value.message


def test_go_to_location_unknown_name(tmp_path):
    with pytest.raises(TaskFailure) as failure:
        tasks.go_to_location(Harness(tmp_path).context(), "nowhere", 30.0, "")
    assert failure.value.code == ErrorCode.LOCATION_NOT_FOUND


def frame_harness(**kwargs):
    h = Harness(**kwargs)
    h.robot.frames["cup"] = (3.0, 0.0)
    return h


def test_go_to_frame_auto_parks_short_and_faces_target():
    h = frame_harness(start=Pose2D(0.0, 0.0, math.radians(90)))
    result = tasks.go_to_frame(h.context(), "cup", 1.0, "", 0.0, 30.0, "")
    assert result.succeeded, result.message
    assert result.fields["final_distance"] == pytest.approx(1.0, abs=0.01)
    assert abs(h.robot.pose.yaw) < math.radians(6)


def test_go_to_frame_named_side():
    h = frame_harness()
    result = tasks.go_to_frame(h.context(), "cup", 0.5, "left", 0.3, 30.0, "")
    assert result.succeeded, result.message
    assert (h.robot.pose.x, h.robot.pose.y) == pytest.approx((3.0, 0.8), abs=0.01)


def test_go_to_frame_final_pose_is_authoritative():
    # Nav2 aborts every attempt, yet the robot is parked correctly -> success.
    h = frame_harness(script=("abort_but_arrive",))
    result = tasks.go_to_frame(h.context(), "cup", 1.0, "", 0.0, 30.0, "")
    assert result.succeeded, result.message
    assert result.fields["attempts"] == 3


def test_go_to_frame_unreachable_side_fails_loudly():
    h = frame_harness(plannable=False)
    result = tasks.go_to_frame(h.context(), "cup", 0.5, "BACK", 0.0, 30.0, "")
    assert result.code == ErrorCode.NO_VALID_PATH
    assert h.nav2.goals == [] and h.robot.commands == 0


def test_go_to_frame_already_there_skips_nav2():
    h = frame_harness(start=Pose2D(2.5, 0.0, 0.0))
    result = tasks.go_to_frame(h.context(), "cup", 1.0, "AUTO", 0.0, 30.0, "")
    assert result.succeeded and h.nav2.goals == []


def test_go_to_frame_cancel_is_not_masked_by_verification():
    h = frame_harness(script=("hang",))
    h.clock.listeners.append(lambda dt: h.clock.t > 1.0 and h.token.cancel())
    result = tasks.go_to_frame(h.context(), "cup", 1.0, "", 0.0, 30.0, "")
    assert result.code == ErrorCode.CANCELED


def test_go_to_frame_wrong_final_pose_fails():
    h = frame_harness(script=(Nav2Status.ABORTED,))
    result = tasks.go_to_frame(h.context(), "cup", 1.0, "", 0.0, 30.0, "")
    assert result.code == ErrorCode.NAVIGATION_FAILED
    assert result.fields["final_distance"] == pytest.approx(3.0, abs=0.01)


def test_go_to_frame_missing_tf():
    with pytest.raises(TaskFailure) as failure:
        tasks.go_to_frame(Harness().context(), "ghost", 1.0, "", 0.0, 30.0, "")
    assert failure.value.code == ErrorCode.TF_LOOKUP_FAILED


def test_rotate_in_place_default_tolerance():
    h = Harness()
    result = tasks.rotate_in_place(h.context(), 90.0, 0.0, 0.0)
    assert result.succeeded, result.message
    assert result.fields["achieved_angle_deg"] == pytest.approx(90.0, abs=10.0)
    assert any("remaining_angle_deg" in f for f in h.feedback)


def test_drive_straight_backward():
    h = Harness()
    result = tasks.drive_straight(h.context(), -0.4, 0.0, 0.0)
    assert result.succeeded, result.message
    assert result.fields["achieved_distance"] == pytest.approx(-0.4, abs=0.1)


def test_nudge_to_frame_stops_short_and_faces():
    h = Harness(start=Pose2D(0.0, 0.0, math.radians(60)))
    h.robot.frames["table"] = (1.5, 0.0)
    result = tasks.nudge_to_frame(h.context(), "table", 0.5, 0.0)
    assert result.succeeded, result.message
    assert result.fields["final_distance"] == pytest.approx(0.5, abs=0.1)
    phases = [f["phase"] for f in h.feedback if "phase" in f]
    assert (
        phases[0] == tasks.NudgePhase.ALIGNING and phases[-1] == tasks.NudgePhase.FACING
    )
