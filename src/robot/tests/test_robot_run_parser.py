from collections.abc import Callable
from pathlib import Path

import pytest

from robot import run as robot_run
from robot.run import _build_parser


def test_connect_test_accepts_explicit_robot_id() -> None:
    """Verify the customer supplies its robot identifier explicitly.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If `connect_test` discards the supplied identifier.
    """
    parser = _build_parser()

    args = parser.parse_args(
        [
            "connect_test",
            "robot-host:3000",
            "--sdk_token",
            "token",
            "--robot_id",
            "customer_bot",
        ]
    )

    assert args.robot_id == "customer_bot"


@pytest.mark.parametrize(
    "arguments",
    [
        ["connect_test", "robot-host:3000"],
        ["kinecal", "robot-host:3000"],
        ["calibrate", "robot-host:3000"],
        ["vibration_test", "robot-host:3000", "data"],
        ["velocity_test", "robot-host:3000", "data"],
    ],
)
def test_standard_bots_hardware_routes_require_robot_id(
    arguments: list[str],
) -> None:
    """Verify hardware-facing Standard Bots routes reject an omitted robot ID.

    Args:
        arguments: Command-line arguments without `--robot_id`.

    Raises:
        AssertionError: If a route accepts an omitted robot ID.
    """
    parser = _build_parser()

    with pytest.raises(SystemExit, match="2"):
        parser.parse_args(arguments)


def test_kinecal_threads_robot_id_to_robot_factory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Verify Kinecal supplies its required robot ID to the adapter factory.

    Args:
        monkeypatch: Pytest fixture used to isolate the Kinecal runner.

    Raises:
        AssertionError: If the factory receives the wrong robot ID.
    """
    captured: dict[str, str] = {}

    def fake_robot_interface(*, robot_ip: str, robot_id: str) -> object:
        """Capture the adapter arguments without initializing hardware."""
        captured.update(robot_ip=robot_ip, robot_id=robot_id)
        return object()

    def fake_run_kinecal_entrypoint(
        *,
        cli_options: object,
        robot_factory: Callable[[str], object],
        script_path: Path,
    ) -> None:
        """Invoke the application-provided robot factory as Kinecal does."""
        del cli_options, script_path
        robot_factory("resolved-robot-host:3000")

    monkeypatch.setattr(robot_run, "RobotInterface", fake_robot_interface)
    monkeypatch.setattr(
        robot_run, "run_kinecal_entrypoint", fake_run_kinecal_entrypoint
    )
    args = _build_parser().parse_args(
        ["kinecal", "robot-host:3000", "--robot_id", "customer_bot"]
    )

    robot_run.route_user_input(args)

    assert captured == {
        "robot_ip": "resolved-robot-host:3000",
        "robot_id": "customer_bot",
    }
