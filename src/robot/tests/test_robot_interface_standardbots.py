import pytest
import inspect

from reforge_core.hw_interfaces.arm_client import ArmClient

from robot.robot_interface import (
    RobotInterface,
    _joint_values_for_sdk,
    _joint_values_in_radians,
    _require,
    _standardbots_url,
)


def test_standardbots_url_adds_http_scheme() -> None:
    """Verify Standard Bots hostnames are converted to SDK URLs.

    Args:
        None.

    Returns:
        `None`.

    Side Effects:
        None.

    Raises:
        AssertionError: If URL normalization changes unexpectedly.

    Preconditions:
        None.
    """
    assert _standardbots_url("192.168.1.10") == "http://192.168.1.10"
    assert _standardbots_url("https://robot.example") == "https://robot.example"


def test_standardbots_url_rejects_empty_host() -> None:
    """Verify live Standard Bots mode requires a controller host.

    Args:
        None.

    Returns:
        `None`.

    Side Effects:
        None.

    Raises:
        AssertionError: If empty hosts stop raising `ValueError`.

    Preconditions:
        None.
    """
    with pytest.raises(ValueError, match="robot_ip must be non-empty"):
        _standardbots_url("")


def test_require_rejects_missing_sdk_value() -> None:
    """Verify omitted SDK telemetry raises a clear adapter error.

    Args:
        None.

    Returns:
        `None`.

    Side Effects:
        None.

    Raises:
        AssertionError: If missing SDK values stop raising `RuntimeError`.

    Preconditions:
        None.
    """
    assert _require(3.0, "missing") == 3.0
    with pytest.raises(RuntimeError, match="missing"):
        _require(None, "missing")


def test_joint_conversion_helpers_preserve_radian_values() -> None:
    """Verify Standard Bots joint conversion helpers preserve radian values.

    Args:
        None.

    Returns:
        `None`.

    Side Effects:
        None.

    Raises:
        AssertionError: If radian joint values are converted unexpectedly.

    Preconditions:
        `IS_DEGREES` remains false for the Standard Bots adapter.
    """
    joints = [0.0, 1.0, -2.0]

    assert _joint_values_in_radians(joints) == joints
    assert _joint_values_for_sdk(joints) == tuple(joints)


def test_servo_command_does_not_spin_ros_executor() -> None:
    """Verify servo publishing avoids re-entering a ROS executor.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If `command_servo_j` calls `rclpy.spin_once`.

    Preconditions:
        ROS callbacks may already be spinning in the native IMU recorder.
    """
    source = inspect.getsource(RobotInterface.command_servo_j)

    assert "spin_once" not in source


def test_standardbots_overrides_full_trajectory_command_hook() -> None:
    """Verify Standard Bots keeps the ROS trajectory-controller command hook.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If Standard Bots falls back to base one-point servo streaming.

    Preconditions:
        The robot's ROS controller requires trajectory-level command context.
    """
    source = inspect.getsource(RobotInterface.command_joint_trajectory)

    assert "JOINT_PUBLISHER" in source
    assert "JointTrajectory" in source
    assert "command_servo_j" not in source
    assert (
        RobotInterface.publish_and_record_joint_positions
        is ArmClient.publish_and_record_joint_positions
    )
