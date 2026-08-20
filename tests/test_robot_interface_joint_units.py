"""Regression tests for robot-interface joint-state units."""

import numpy as np
import pytest

from reforge_core.hw_interfaces import arm_imu_time_calibration
from robot.robot_interface import RobotInterface


class FakeXArm:
    """Provide deterministic xArm joint states for unit-conversion tests.

    Args:
        is_radian: Whether returned positions and velocities use radians.
        positions: Joint positions in the configured SDK angular units.
        velocities: Joint velocities in the configured SDK angular units per
            second.

    Raises:
        None.
    """

    def __init__(
        self,
        *,
        is_radian: bool,
        positions: list[float],
        velocities: list[float],
    ) -> None:
        """Store one deterministic xArm state response.

        Args:
            is_radian: Whether returned angular values use radians.
            positions: Joint positions in SDK angular units.
            velocities: Joint velocities in SDK angular units per second.

        Returns:
            `None`.

        Raises:
            None.
        """
        self._is_radian = is_radian
        self._positions = positions
        self._velocities = velocities

    def get_joint_states(
        self,
    ) -> tuple[int, tuple[list[float], list[float], list[float]]]:
        """Return one successful xArm joint-state response.

        Args:
            None.

        Returns:
            `tuple[int, tuple[list[float], list[float], list[float]]]`
            containing the status code, positions, velocities, and efforts.

        Raises:
            None.
        """
        efforts = [0.0] * len(self._positions)
        return 0, (self._positions, self._velocities, efforts)


class FakeServoArm:
    """Record servo commands produced by the time-calibration primitive.

    Args:
        None.

    Raises:
        None.
    """

    def __init__(self) -> None:
        """Initialize an empty servo-command history.

        Args:
            None.

        Returns:
            `None`.

        Raises:
            None.
        """
        self.commands: list[np.ndarray] = []

    def set_servo_mode(self) -> None:
        """Accept the calibration primitive's servo-mode transition.

        Args:
            None.

        Returns:
            `None`.

        Raises:
            None.
        """

    def command_servo_j(self, target_joints: np.ndarray) -> int:
        """Record one commanded joint position.

        Args:
            target_joints: Commanded joint positions [rad].

        Returns:
            `int` zero status code indicating success.

        Raises:
            None.
        """
        self.commands.append(target_joints.copy())
        return 0


@pytest.mark.parametrize(
    "is_radian, sdk_position, sdk_velocity",
    [
        (True, np.pi / 2.0, np.pi),
        (False, 90.0, 180.0),
    ],
)
def test_get_joint_state_returns_radians(
    is_radian: bool,
    sdk_position: float,
    sdk_velocity: float,
) -> None:
    """Verify xArm joint states are converted to radians exactly once.

    Args:
        is_radian: Whether the fake SDK reports radians.
        sdk_position: Position returned by the fake SDK.
        sdk_velocity: Velocity returned by the fake SDK.

    Returns:
        `None`.

    Raises:
        None.
    """
    robot_interface = RobotInterface.__new__(RobotInterface)
    robot_interface.robot = FakeXArm(
        is_radian=is_radian,
        positions=[sdk_position],
        velocities=[sdk_velocity],
    )
    robot_interface.num_joints = 1

    positions, velocities, _ = robot_interface.get_joint_state()

    assert positions == pytest.approx([np.pi / 2.0])
    assert velocities == pytest.approx([np.pi])


def test_servo_time_calibration_starts_at_measured_pose(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Verify asymmetric calibration limits do not create an initial jump.

    Args:
        monkeypatch: Pytest helper used to replace wall-clock waits.

    Returns:
        `None`.

    Raises:
        None.
    """
    fake_arm = FakeServoArm()
    q0 = np.array([0.7, -0.4])
    q_plus = np.array([0.8, -0.4])
    q_minus = np.array([0.5, -0.4])
    clock_time_s = iter(np.arange(100, dtype=float))
    monkeypatch.setattr(
        arm_imu_time_calibration.time,
        "time",
        lambda: next(clock_time_s),
    )
    monkeypatch.setattr(arm_imu_time_calibration.time, "sleep", lambda _: None)

    arm_imu_time_calibration.run_servo_motion_primitive(
        fake_arm,
        q0,
        q_plus=q_plus,
        q_minus=q_minus,
        control_hz=2.0,
    )

    np.testing.assert_allclose(fake_arm.commands[0], q0)
