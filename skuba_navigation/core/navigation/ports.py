"""
What the navigation core needs from the outside world.

The adapters/ package implements these on rclpy (Nav2 action clients, tf2,
a cmd_vel publisher); tests implement them with in-memory fakes. Core code
depends only on these Protocols, never on rclpy.
"""

from dataclasses import dataclass
import enum
from typing import Callable, Optional, Protocol, Tuple

from .types import Pose2D


class Nav2Status(enum.Enum):
    SUCCEEDED = "succeeded"
    ABORTED = "aborted"
    CANCELED = "canceled"  # our stop request, or someone else canceled Nav2
    REJECTED = "rejected"
    UNAVAILABLE = "unavailable"  # action server not reachable


@dataclass(frozen=True)
class Nav2Feedback:
    pose: Optional[Pose2D] = None
    distance_remaining: float = 0.0
    estimated_time_remaining: float = 0.0
    number_of_recoveries: int = 0


class Nav2Port(Protocol):
    def navigate(
        self,
        goal: Pose2D,
        frame_id: str,
        behavior_tree: str,
        on_feedback: Callable[[Nav2Feedback], None],
        should_stop: Callable[[], bool],
    ) -> Nav2Status:
        """
        Send one NavigateToPose goal and block until it ends.

        If should_stop() turns true, cancel the Nav2 goal, wait for it to
        wind down, and return CANCELED.
        """

    def is_plannable(
        self, start: Pose2D, goal: Pose2D, frame_id: str
    ) -> Optional[bool]:
        """Ask the global planner for a path. None = planner not reachable."""

    def clear_costmaps(self) -> bool:
        """Clear global and local costmaps. False if any clear failed."""


class PoseSource(Protocol):
    def robot_pose(self, timeout: Optional[float] = None) -> Pose2D:
        """Robot pose in the map frame. Raises TaskFailure(TF_LOOKUP_FAILED)."""

    def frame_position(
        self, frame: str, timeout: Optional[float] = None
    ) -> Tuple[float, float]:
        """(x, y) of `frame` in the map frame. Raises TaskFailure(TF_LOOKUP_FAILED)."""


class BaseCommander(Protocol):
    def command(self, linear: float, angular: float) -> None:
        """Send one velocity command to the base."""

    def stop(self) -> None:
        """Bring the base to a halt; may send several zero commands."""
