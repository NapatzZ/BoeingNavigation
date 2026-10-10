"""Core enums must match the constants in skuba_interfaces (skipped without ROS)."""

import pytest

from skuba_navigation.core.navigation.tasks import NudgePhase
from skuba_navigation.core.navigation.types import ErrorCode, Phase

interfaces = pytest.importorskip("skuba_interfaces.msg")
actions = pytest.importorskip("skuba_interfaces.action")


@pytest.mark.parametrize(
    "enum_cls, msg_cls, prefix",
    [
        (ErrorCode, interfaces.NavigationError, ""),
        (Phase, interfaces.NavigationProgress, "PHASE_"),
        (NudgePhase, actions.NudgeToFrame.Feedback, "PHASE_"),
    ],
)
def test_enum_matches_interface(enum_cls, msg_cls, prefix):
    constants = {
        name.removeprefix(prefix): value
        for name, value in vars(msg_cls).items()
        if name.isupper() and name.startswith(prefix) and isinstance(value, int)
    }
    assert constants == {member.name: member.value for member in enum_cls}
