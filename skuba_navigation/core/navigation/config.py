"""Tunables for navigation_server.

Every field is declared as a ROS parameter named `<section>.<field>` (see
params/navigation_server.yaml), so adding a field here is all it takes to
make a new knob configurable. Defaults reproduce the old nav_to_pose node.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class NavigationConfig:
    """Nav2-backed goals (GoToPose / GoToLocation / GoToFrame)."""

    default_timeout: float = 60.0         # s, total across attempts when goal.timeout <= 0
    max_retries: int = 2                  # extra attempts after a Nav2 ABORTED
    retry_delay: float = 1.0              # s between attempts
    clear_costmaps_on_start: bool = True
    clear_costmaps_on_retry: bool = True
    feedback_rate: float = 5.0            # Hz, upper bound on action feedback
    align_timeout: float = 10.0           # s for the final face-the-target turn
    standoff_tolerance: float = 0.15      # m, radial slack when verifying a standoff
    facing_tolerance_deg: float = 5.0     # final heading error that triggers a re-face
    side_bearing_tolerance_deg: float = 22.5  # half-width of a named approach side


@dataclass(frozen=True)
class MotionConfig:
    """Closed-loop cmd_vel motions (RotateInPlace / DriveStraight / NudgeToFrame)."""

    # Rotation PD controller, tuned for the heavy base.
    rotate_kp: float = 3.0
    rotate_kd: float = 0.3
    rotate_max_speed: float = 0.5         # rad/s
    rotate_min_speed: float = 0.18        # rad/s, below this floor friction stalls the base
    rotate_decel_zone_deg: float = 5.0    # inside this error, command min speed
    rotate_rate: float = 50.0             # Hz
    rotate_default_tolerance_deg: float = 10.0
    rotate_default_timeout: float = 30.0  # s
    rotate_settle_time: float = 0.5       # s to let inertia settle before verifying

    # Straight-line driving.
    drive_speed: float = 0.2              # m/s
    drive_slow_zone: float = 0.1          # m before the target where we slow down
    drive_slow_factor: float = 0.3
    drive_stop_tolerance: float = 0.05    # m, loop exit threshold
    drive_verify_tolerance: float = 0.10  # m, success threshold
    drive_rate: float = 10.0              # Hz
    drive_timeout_margin: float = 5.0     # s added to |distance| / speed
    drive_settle_time: float = 0.1        # s

    # NudgeToFrame.
    nudge_align_threshold_deg: float = 5.0   # turn toward the frame only above this
    nudge_align_tolerance_deg: float = 3.0
    nudge_face_tolerance_deg: float = 2.0
    nudge_default_timeout: float = 30.0      # s
    nudge_verify_tolerance: float = 0.15     # m on the final distance to the frame

    # Shared.
    tf_grace: float = 0.5                 # s of TF dropout tolerated inside a control loop
    stop_repeats: int = 10                # zero-twist messages sent to stop the base
