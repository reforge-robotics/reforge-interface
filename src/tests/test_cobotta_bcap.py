"""Unit tests for the DENSO COBOTTA PRO b-CAP adapter."""

from __future__ import annotations

import math
from typing import Any

import pytest

from robot.cobotta_bcap import (
    SLAVE_RECV_FORMAT_TIMESTAMP_POSE_JOINT_CURRENT,
    CobottaBcapRobot,
    CobottaControlMode,
)
from robot.robot_interface import RobotInterface


class FakeBcapTransport:
    """Record b-CAP calls and return deterministic controller data for tests."""

    def __init__(self, fail_at: str | None = None) -> None:
        """Initialize the fake transport.

        Args:
            fail_at: Optional method name that raises a connection failure.
        """
        self.calls: list[tuple[str, tuple[Any, ...]]] = []
        self.fail_at = fail_at
        self.slave_responses: list[object] = []

    def _record(self, name: str, *args: Any) -> None:
        """Record a call and optionally raise its configured failure.

        Args:
            name: Method name.
            args: Method arguments.
        """
        self.calls.append((name, args))
        if self.fail_at == name:
            raise RuntimeError(f"forced failure at {name}")

    def service_start(self, option: str = "") -> None:
        """Record service startup.

        Args:
            option: b-CAP option.
        """
        self._record("service_start", option)

    def service_stop(self) -> None:
        """Record service shutdown."""
        self._record("service_stop")

    def controller_connect(
        self, name: str, provider: str, machine: str, option: str
    ) -> int:
        """Record controller connection.

        Args:
            name: Controller name.
            provider: Provider name.
            machine: Provider machine.
            option: Provider option.

        Returns:
            Deterministic controller handle.
        """
        self._record("controller_connect", name, provider, machine, option)
        return 10

    def controller_disconnect(self, handle: int) -> None:
        """Record controller release.

        Args:
            handle: Controller handle.
        """
        self._record("controller_disconnect", handle)

    def controller_getrobot(self, handle: int, name: str, option: str = "") -> int:
        """Record robot handle acquisition.

        Args:
            handle: Controller handle.
            name: Robot name.
            option: Provider option.

        Returns:
            Deterministic robot handle.
        """
        self._record("controller_getrobot", handle, name, option)
        return 20

    def robot_release(self, handle: int) -> None:
        """Record robot handle release.

        Args:
            handle: Robot handle.
        """
        self._record("robot_release", handle)

    def robot_execute(self, handle: int, command: str, param: object = None) -> object:
        """Record a robot command and return matching fake telemetry.

        Args:
            handle: Robot handle.
            command: RC9 command.
            param: Command parameter.

        Returns:
            Matching fake command result.
        """
        self._record("robot_execute", handle, command, param)
        if command == "CurJntEx":
            return [1_000.0, 0.0, 90.0, 180.0, 0.0, -90.0, 45.0]
        if command == "CurPos":
            return [1_000.0, 200.0, 300.0, 0.0, 0.0, 90.0, 0.0]
        if command == "slvMove":
            return self.slave_responses.pop(0)
        return None

    def robot_move(
        self, handle: int, comp: int, pose: object, option: str = ""
    ) -> None:
        """Record a point motion.

        Args:
            handle: Robot handle.
            comp: Completion mode.
            pose: Pose value.
            option: Motion option.
        """
        self._record("robot_move", handle, comp, pose, option)

    def close(self) -> None:
        """Record TCP transport closure."""
        self._record("close")


def connected_robot(
    transport: FakeBcapTransport,
) -> CobottaBcapRobot:
    """Connect one wrapper to a supplied fake transport.

    Args:
        transport: Fake b-CAP transport.

    Returns:
        Connected Cobotta wrapper.
    """
    robot = CobottaBcapRobot(
        "192.0.2.1",
        transport_factory=lambda _host, _port, _timeout_ms: transport,
    )
    robot.connect()
    return robot


def slave_response(timestamp_ms: float, joint_degrees: list[float]) -> list[object]:
    """Build one documented `0x0054` Slave Mode response.

    Args:
        timestamp_ms: Controller timestamp [ms].
        joint_degrees: Six joint positions [deg].

    Returns:
        Timestamp, P+J values, and electric-current values.
    """
    return [
        timestamp_ms,
        [100.0, 200.0, 300.0, 0.0, 0.0, 0.0, 0.0, *joint_degrees],
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    ]


def command_calls(transport: FakeBcapTransport, command: str) -> list[tuple[Any, ...]]:
    """Return recorded `robot_execute` arguments for one command.

    Args:
        transport: Fake b-CAP transport.
        command: RC9 command name.

    Returns:
        Recorded argument tuples.
    """
    return [
        args
        for name, args in transport.calls
        if name == "robot_execute" and args[1] == command
    ]


def test_connect_cleanup_releases_partial_resources() -> None:
    """Clean up the controller and service after robot acquisition fails."""
    transport = FakeBcapTransport(fail_at="controller_getrobot")
    robot = CobottaBcapRobot(
        "192.0.2.1",
        transport_factory=lambda _host, _port, _timeout_ms: transport,
    )

    with pytest.raises(RuntimeError, match="forced failure"):
        robot.connect()

    assert [name for name, _args in transport.calls] == [
        "service_start",
        "controller_connect",
        "controller_getrobot",
        "controller_disconnect",
        "service_stop",
        "close",
    ]
    assert not robot.is_connected


def test_normal_motion_converts_reforge_units_at_bcap_boundary() -> None:
    """Convert radians/meters to degree/millimeter b-CAP motion values."""
    transport = FakeBcapTransport()
    robot = connected_robot(transport)

    robot.move_j(
        [0.0, math.pi / 2, math.pi, 0.0, -math.pi / 2, math.pi / 4],
        speed_percent=25.0,
        wait=False,
    )
    robot.move_pose(
        [0.0, 0.0, 0.0, 1.0], [1.0, 2.0, 3.0], speed_percent=50.0, wait=True
    )

    moves = [args for name, args in transport.calls if name == "robot_move"]
    assert moves[0][2] == [[0.0, 90.0, 180.0, 0.0, -90.0, 45.0], "J", "@E"]
    assert moves[0][3] == "NEXT"
    assert moves[1][2] == [[1000.0, 2000.0, 3000.0, 0.0, 0.0, 0.0, 0], "P", "@E"]
    assert moves[1][3] == ""


def test_slave_mode_routes_commands_and_preserves_latest_response_state() -> None:
    """Route Slave commands through `slvMove` and use their cached state."""
    transport = FakeBcapTransport()
    transport.slave_responses.append(slave_response(1_000.0, [0.0] * 6))
    robot = connected_robot(transport)

    robot.enter_servo_mode()
    sample = robot.command_servo_j([0.0] * 6)

    assert robot.control_mode is CobottaControlMode.SLAVE
    assert (
        command_calls(transport, "slvRecvFormat")[-1][2]
        == SLAVE_RECV_FORMAT_TIMESTAMP_POSE_JOINT_CURRENT
    )
    assert command_calls(transport, "slvMove")[-1][2] == [0.0] * 6
    assert robot.get_joint_sample() == sample
    assert sample.efforts == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert sample.positions_rad == [0.0] * 6

    with pytest.raises(RuntimeError, match="Slave Mode"):
        robot.move_j([0.0] * 6, speed_percent=10.0, wait=True)
    with pytest.raises(RuntimeError, match="Slave Mode"):
        robot.set_gravity_compensation(True)

    robot.enter_position_mode()
    assert robot.control_mode is CobottaControlMode.NORMAL


def test_slave_velocity_uses_only_strictly_increasing_timestamps() -> None:
    """Derive velocity only when controller time advances by a valid interval."""
    transport = FakeBcapTransport()
    transport.slave_responses.extend(
        [
            slave_response(1_000.0, [0.0] * 6),
            slave_response(1_008.0, [90.0] * 6),
            slave_response(1_008.0, [180.0] * 6),
        ]
    )
    robot = connected_robot(transport)
    robot.enter_servo_mode()

    first = robot.command_servo_j([0.0] * 6)
    second = robot.command_servo_j([0.0] * 6)
    duplicate_timestamp = robot.command_servo_j([0.0] * 6)

    assert not first.velocity_valid
    assert second.velocity_valid
    assert second.velocities_rad_s == pytest.approx([math.pi / 2.0 / 0.008] * 6)
    assert not duplicate_timestamp.velocity_valid
    assert duplicate_timestamp.velocities_rad_s == [0.0] * 6


def test_invalid_mode_and_joint_commands_are_rejected() -> None:
    """Reject servo commands outside Slave Mode and malformed arm targets."""
    transport = FakeBcapTransport()
    robot = connected_robot(transport)

    with pytest.raises(RuntimeError, match="requires active Slave Mode"):
        robot.command_servo_j([0.0] * 6)
    with pytest.raises(ValueError, match="exactly 6"):
        robot.move_j([0.0] * 5, speed_percent=10.0, wait=True)
    with pytest.raises(ValueError, match="speed_percent"):
        robot.move_j([0.0] * 6, speed_percent=0.0, wait=True)


def test_robot_interface_delegates_to_the_cobotta_wrapper() -> None:
    """Use RobotInterface methods with a connected mocked b-CAP wrapper."""
    transport = FakeBcapTransport()
    transport.slave_responses.append(slave_response(1_000.0, [0.0] * 6))
    interface = RobotInterface.__new__(RobotInterface)
    interface.robot = connected_robot(transport)
    interface.num_joints = 6

    assert interface.command_move_j([0.0] * 6) == 0
    assert interface.enter_servo_mode() == 0
    assert interface.command_servo_j([0.0] * 6) == 0
    assert interface.get_joint_state() == (
        [0.0] * 6,
        [0.0] * 6,
        [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )
    assert interface.enter_position_mode() == 0
    assert not interface.supports_teaching_mode()
    assert interface.enter_teaching_mode() is None
    assert not command_calls(transport, "GrvCtrl")


def test_robot_interface_rejects_faster_than_125_hz_trajectory_timing() -> None:
    """Reject trajectory samples that claim unsupported fresh-state cadence."""
    interface = RobotInterface.__new__(RobotInterface)
    interface.num_joints = 6

    with pytest.raises(ValueError, match="at least 0.008"):
        interface.command_joint_trajectory(
            [0.0, 0.005],
            [[0.0] * 6, [0.0] * 6],
            Ts=0.005,
        )
