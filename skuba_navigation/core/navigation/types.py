"""Plain data types shared across the navigation core."""
from dataclasses import dataclass, field
import enum
import math
from typing import Any, Dict


class ErrorCode(enum.IntEnum):
    """Mirror of skuba_interfaces/msg/NavigationError (kept in sync by a test)."""

    NONE = 0
    UNKNOWN = 1
    INVALID_GOAL = 2
    LOCATION_NOT_FOUND = 3
    TF_LOOKUP_FAILED = 4
    NAV2_UNAVAILABLE = 5
    GOAL_REJECTED = 6
    NO_VALID_PATH = 7
    NAVIGATION_FAILED = 8
    TIMEOUT = 9
    CANCELED = 10
    PREEMPTED = 11
    VERIFICATION_FAILED = 12


class Phase(enum.IntEnum):
    """Mirror of the PHASE_* constants in skuba_interfaces/msg/NavigationProgress."""

    RESOLVING = 0
    PLANNING = 1
    NAVIGATING = 2
    RETRYING = 3
    ALIGNING = 4
    VERIFYING = 5


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in the map frame (metres, radians)."""

    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)

    def bearing_to(self, x: float, y: float) -> float:
        return math.atan2(y - self.y, x - self.x)


@dataclass(frozen=True)
class Progress:
    """Feedback of a Nav2-backed goal; converted to NavigationProgress by the node."""

    phase: Phase
    attempt: int
    target: Pose2D
    elapsed: float
    current: Pose2D = None
    distance_remaining: float = 0.0
    estimated_time_remaining: float = 0.0
    number_of_recoveries: int = 0


class TaskFailure(Exception):
    """Raised anywhere in a task to end the goal with `code`."""

    def __init__(self, code: ErrorCode, message: str):
        super().__init__(message)
        self.code = ErrorCode(code)
        self.message = message


@dataclass
class TaskResult:
    """What a task returns to the node.

    `fields` are copied onto the action Result by name (e.g. 'attempts',
    'final_pose'), so a task only reports what its action defines.
    """

    code: ErrorCode = ErrorCode.NONE
    message: str = ''
    fields: Dict[str, Any] = field(default_factory=dict)

    @property
    def succeeded(self) -> bool:
        return self.code == ErrorCode.NONE
