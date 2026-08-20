"""Read-only Standard Bots SDK probe for the frozen ROS-control state."""

from __future__ import annotations

import os
from typing import Any


def _sdk_url(endpoint: str) -> str:
    """Return the endpoint in the URL form required by the SDK."""
    if not endpoint:
        raise ValueError("REFORGE_QUALIFICATION_ENDPOINT must be set.")
    if endpoint.startswith(("http://", "https://")):
        return endpoint
    return f"http://{endpoint}"


def _is_enabled(value: Any) -> bool:
    """Return whether an SDK response represents ROS control Enabled."""
    for attribute in ("state", "action", "value", "name"):
        nested = getattr(value, attribute, None)
        if nested is not None and nested is not value and _is_enabled(nested):
            return True
    normalized = str(value).strip().lower()
    return normalized == "enabled" or normalized.endswith(".enabled")


def query_control_enabled() -> bool:
    """Read the live SDK ROS-control state without changing it."""
    endpoint = os.environ.get("REFORGE_QUALIFICATION_ENDPOINT", "")
    token = os.environ.get("STANDARD_BOTS_SDK_TOKEN", "")
    if not token:
        raise ValueError("STANDARD_BOTS_SDK_TOKEN must be set in the environment.")

    from standardbots import StandardBotsRobot

    robot = StandardBotsRobot(
        url=_sdk_url(endpoint),
        token=token,
        robot_kind=StandardBotsRobot.RobotKind.Live,
    )
    with robot.connection():
        state = robot.ros.status.get_ros_control_state().ok()
    return _is_enabled(state)


def main() -> int:
    """Exit successfully only when the read-only SDK state is Enabled."""
    try:
        enabled = query_control_enabled()
    except Exception as error:
        print(f"Standard Bots control-state probe failed: {error}")
        return 2
    print(f"Standard Bots SDK ROS control: {'Enabled' if enabled else 'not Enabled'}")
    return 0 if enabled else 1


if __name__ == "__main__":
    raise SystemExit(main())
