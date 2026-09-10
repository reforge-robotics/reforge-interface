"""Tests for Standard Bots widget command validation."""

from collections.abc import Callable

import pytest

from app import _build_calibrate_command, _build_connect_command


@pytest.mark.parametrize(
    "builder",
    [_build_connect_command, _build_calibrate_command],
)
def test_widget_commands_require_robot_id(
    builder: Callable[[dict[str, object]], list[str]],
) -> None:
    """Verify widget commands reject requests without a Standard Bots ID.

    Args:
        builder: Widget command builder under test.

    Raises:
        AssertionError: If a builder accepts a missing robot ID.
    """
    with pytest.raises(ValueError, match="Missing required field: robotId"):
        builder({"robotIp": "robot-host:3000"})
