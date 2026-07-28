"""Compatibility entrypoint for robot command-line operations."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

from reforge_core.calibration import run_helpers
from robot.robot_interface import BOT_ID, RobotInterface


def main(argv: Sequence[str] | None = None) -> None:
    """Parse robot CLI arguments and dispatch to the SDK-owned implementation.

    May connect to robot hardware, run calibration, or write model files.

    Args:
        argv: Optional command-line arguments. Uses `sys.argv` when omitted.

    Returns:
        `None`.

    Raises:
        SystemExit: If argument parsing fails.
    """
    return run_helpers.main(
        robot_interface_class=RobotInterface,
        default_robot_id=BOT_ID,
        argv=argv,
        script_path=Path(__file__).resolve(),
    )


if __name__ == "__main__":
    main()
