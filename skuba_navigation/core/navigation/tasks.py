"""
One function per action - read this file to know what each action does.

Every task gets a TaskContext (ports + config + this goal's cancel token and
feedback sink), takes plain Python goal values and returns a TaskResult. A
TaskFailure raised anywhere ends the goal with that code; the node turns it
into the action result.
"""
from dataclasses import dataclass
import enum
import math
from typing import Callable, Optional

from ..geometry_utils import normalize_angle
from .cancellation import CancelToken
from .clock import Clock, Deadline
from .config import MotionConfig, NavigationConfig
from .location_store import LocationStore
from .motion import BaseMotion
from .navigation_job import NavigationJob, NavigationReport, NavigationRequest
from .ports import Nav2Port, PoseSource
from .standoff import parse_side, standoff_goal, verify_standoff
from .types import ErrorCode, Phase, Pose2D, Progress, TaskFailure, TaskResult

FeedbackSink = Callable[[dict, bool], None]   # (feedback fields, force_publish)

# GoToFrame: Nav2 giving up (or the clock running out) while the robot is in
# fact parked correctly still counts as success - the final pose check decides.
_SETTLED_BY_VERIFICATION = (ErrorCode.NONE, ErrorCode.NAVIGATION_FAILED, ErrorCode.TIMEOUT)


class NudgePhase(enum.IntEnum):
    """Mirror of the PHASE_* constants in NudgeToFrame feedback."""

    ALIGNING = 0
    DRIVING = 1
    FACING = 2


@dataclass
class TaskContext:
    nav2: Nav2Port
    poses: PoseSource
    motion: BaseMotion
    locations: LocationStore
    nav_config: NavigationConfig
    motion_config: MotionConfig
    clock: Clock
    token: CancelToken
    feedback: FeedbackSink
    started_at: float

    def elapsed(self) -> float:
        return self.clock.now() - self.started_at

    def deadline(self, timeout: float, default: float) -> Deadline:
        return Deadline(self.clock, timeout if timeout > 0.0 else default)

    def navigate(self, request: NavigationRequest, timeout: float) -> NavigationReport:
        job = NavigationJob(
            self.nav2, self.poses, self.nav_config, self.clock, self.token,
            self.deadline(timeout, self.nav_config.default_timeout), self.elapsed,
            on_progress=lambda progress, force: self.feedback({'progress': progress}, force))
        return job.run(request)

    def report_phase(self, phase: Phase, target: Pose2D, attempt: int) -> None:
        self.feedback({'progress': Progress(phase, attempt, target, self.elapsed())}, True)

    def final_pose(self) -> Optional[Pose2D]:
        try:
            return self.poses.robot_pose(0.5)
        except TaskFailure:
            return None


# ── Nav2-backed ───────────────────────────────────────────────────────────


def go_to_pose(ctx: TaskContext, target: Pose2D, frame_id: str, timeout: float,
               behavior_tree: str) -> TaskResult:
    request = NavigationRequest(target, frame_id or 'map', behavior_tree)
    return _navigation_result(ctx, ctx.navigate(request, timeout))


def go_to_location(ctx: TaskContext, location: str, timeout: float,
                   behavior_tree: str) -> TaskResult:
    stored = ctx.locations.get(location)
    if stored is None:
        raise TaskFailure(ErrorCode.LOCATION_NOT_FOUND, f"Unknown location '{location}'.")
    request = NavigationRequest(stored.pose, stored.frame_id, behavior_tree)
    return _navigation_result(ctx, ctx.navigate(request, timeout))


def go_to_frame(ctx: TaskContext, frame: str, stop_distance: float, approach_side: str,
                object_radius: float, timeout: float, behavior_tree: str) -> TaskResult:
    cfg = ctx.nav_config
    frame = frame.strip()
    side = parse_side(approach_side)
    robot = ctx.poses.robot_pose()
    target_x, target_y = ctx.poses.frame_position(frame)

    goal = standoff_goal(robot, target_x, target_y, stop_distance, side, object_radius,
                         cfg.standoff_tolerance, math.radians(cfg.facing_tolerance_deg))
    report = NavigationReport(ErrorCode.NONE, 'Already at the standoff pose.', 0)
    if goal is not None:
        report = ctx.navigate(NavigationRequest(goal, 'map', behavior_tree, check_plan=True),
                              timeout)
        if report.code not in _SETTLED_BY_VERIFICATION:
            return _navigation_result(ctx, report)

    # Nav2 stops anywhere inside its xy_goal_tolerance, so the heading it
    # leaves is not necessarily pointing at the target: correct it in place.
    ctx.report_phase(Phase.ALIGNING, goal or robot, report.attempts)
    note = _best_effort(lambda: ctx.motion.face_point(
        target_x, target_y, math.radians(cfg.facing_tolerance_deg), ctx.token,
        Deadline(ctx.clock, cfg.align_timeout)), 'final heading correction')

    ctx.report_phase(Phase.VERIFYING, goal or robot, report.attempts)
    final = ctx.poses.robot_pose()
    ok, distance, detail = verify_standoff(
        final, target_x, target_y, stop_distance, side, object_radius,
        cfg.standoff_tolerance, math.radians(cfg.side_bearing_tolerance_deg))
    fields = {'attempts': report.attempts, 'final_pose': final, 'final_distance': distance}
    if ok:
        return TaskResult(ErrorCode.NONE, detail + note, fields)
    code = report.code if report.code != ErrorCode.NONE else ErrorCode.VERIFICATION_FAILED
    return TaskResult(code, f'{report.message} {detail}{note}'.strip(), fields)


def _navigation_result(ctx: TaskContext, report: NavigationReport) -> TaskResult:
    return TaskResult(report.code, report.message,
                      {'attempts': report.attempts, 'final_pose': ctx.final_pose()})


# ── cmd_vel motions ───────────────────────────────────────────────────────


def rotate_in_place(ctx: TaskContext, angle_deg: float, tolerance_deg: float,
                    timeout: float) -> TaskResult:
    cfg = ctx.motion_config
    tolerance_deg = tolerance_deg if tolerance_deg > 0.0 else cfg.rotate_default_tolerance_deg
    achieved = math.degrees(ctx.motion.rotate_by(
        math.radians(angle_deg), math.radians(tolerance_deg), ctx.token,
        ctx.deadline(timeout, cfg.rotate_default_timeout),
        on_progress=lambda error: ctx.feedback(
            {'remaining_angle_deg': math.degrees(error)}, False)))

    error = abs(achieved - angle_deg)
    fields = {'achieved_angle_deg': achieved}
    detail = f'Requested {angle_deg:.1f} deg, turned {achieved:.1f} deg (error {error:.1f}).'
    if error < tolerance_deg:
        return TaskResult(ErrorCode.NONE, detail, fields)
    return TaskResult(ErrorCode.VERIFICATION_FAILED, detail, fields)


def drive_straight(ctx: TaskContext, distance: float, speed: float,
                   timeout: float) -> TaskResult:
    cfg = ctx.motion_config
    speed = speed if speed > 0.0 else cfg.drive_speed
    default_timeout = abs(distance) / speed + cfg.drive_timeout_margin
    achieved = ctx.motion.drive_straight(
        distance, speed, ctx.token, ctx.deadline(timeout, default_timeout),
        on_progress=lambda remaining: ctx.feedback({'remaining_distance': remaining}, False))

    error = abs(achieved - distance)
    fields = {'achieved_distance': achieved}
    detail = f'Requested {distance:.3f}m, drove {achieved:.3f}m (error {error:.3f}m).'
    if error <= cfg.drive_verify_tolerance:
        return TaskResult(ErrorCode.NONE, detail, fields)
    return TaskResult(ErrorCode.VERIFICATION_FAILED, detail, fields)


def nudge_to_frame(ctx: TaskContext, frame: str, stop_distance: float,
                   timeout: float) -> TaskResult:
    cfg = ctx.motion_config
    frame = frame.strip()
    deadline = ctx.deadline(timeout, cfg.nudge_default_timeout)
    robot = ctx.poses.robot_pose()
    target_x, target_y = ctx.poses.frame_position(frame)
    travel = robot.distance_to(target_x, target_y) - stop_distance

    def phase(p: NudgePhase, remaining: float) -> None:
        ctx.feedback({'phase': int(p), 'remaining_distance': max(remaining, 0.0)}, True)

    if travel > 0.0:
        phase(NudgePhase.ALIGNING, travel)
        bearing = robot.bearing_to(target_x, target_y)
        if abs(normalize_angle(bearing - robot.yaw)) > math.radians(cfg.nudge_align_threshold_deg):
            ctx.motion.turn_to_heading(
                bearing, math.radians(cfg.nudge_align_tolerance_deg), ctx.token, deadline)
        phase(NudgePhase.DRIVING, travel)
        ctx.motion.drive_straight(
            travel, cfg.drive_speed, ctx.token, deadline,
            on_progress=lambda remaining: ctx.feedback(
                {'phase': int(NudgePhase.DRIVING), 'remaining_distance': remaining}, False))

    phase(NudgePhase.FACING, 0.0)
    target_x, target_y = ctx.poses.frame_position(frame)
    note = _best_effort(lambda: ctx.motion.face_point(
        target_x, target_y, math.radians(cfg.nudge_face_tolerance_deg), ctx.token, deadline),
        'facing the frame')

    distance = ctx.poses.robot_pose().distance_to(target_x, target_y)
    tolerance = cfg.nudge_verify_tolerance
    too_far = distance > stop_distance + tolerance
    too_close = travel > 0.0 and distance < stop_distance - tolerance
    detail = f'Final distance to {frame} {distance:.3f}m (want {stop_distance:.3f}m).'
    fields = {'final_distance': distance}
    if too_far or too_close:
        return TaskResult(ErrorCode.VERIFICATION_FAILED, detail + note, fields)
    return TaskResult(ErrorCode.NONE, detail + note, fields)


def _best_effort(step: Callable[[], object], what: str) -> str:
    """Run a cosmetic step; a failure becomes a note, a cancel/preempt still ends the goal."""
    try:
        step()
    except TaskFailure as failure:
        if failure.code in (ErrorCode.CANCELED, ErrorCode.PREEMPTED):
            raise
        return f' ({what} failed: {failure.message})'
    return ''
