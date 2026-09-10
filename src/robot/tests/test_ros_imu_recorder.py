from types import SimpleNamespace

from robot.ros_imu_recorder import (
    DEFAULT_ROS_IMU_TOPIC_TEMPLATE,
    ROS_UNIX_TIMESTAMP_MINIMUM_S,
    RosImuRecorder,
    _imu_state_from_ros_message,
)


def _fake_imu_message(timestamp_s: float) -> SimpleNamespace:
    """Return a minimal ROS IMU-like message.

    Args:
        timestamp_s: Message header timestamp [s].

    Returns:
        `SimpleNamespace` with the fields read from `sensor_msgs/Imu`.

    Raises:
        None.
    """
    seconds = int(timestamp_s)
    nanoseconds = int((timestamp_s - seconds) * 1e9)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)),
        linear_acceleration=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        angular_velocity=SimpleNamespace(x=4.0, y=5.0, z=6.0),
    )


def test_ros_imu_recorder_resolves_standardbots_topic() -> None:
    """Verify the recorder subscribes to the Standard Bots ROS IMU topic.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If topic templating changes unexpectedly.
    """
    bot_id = "customer_bot"
    recorder = RosImuRecorder(bot_id=bot_id)

    assert recorder.topic == DEFAULT_ROS_IMU_TOPIC_TEMPLATE.replace("<BOT_ID>", bot_id)


def test_ros_imu_message_uses_unix_header_timestamp() -> None:
    """Verify Unix ROS timestamps remain on the arm-compatible timeline.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If timestamp conversion stops using valid ROS headers.
    """
    message_time_s = ROS_UNIX_TIMESTAMP_MINIMUM_S + 12.25
    receive_time_s = message_time_s + 1.0

    state = _imu_state_from_ros_message(
        _fake_imu_message(message_time_s),
        receive_time_s,
    )

    assert state.ts == message_time_s
    assert state.accel().tolist() == [1.0, 2.0, 3.0]
    assert state.gyro().tolist() == [4.0, 5.0, 6.0]


def test_ros_imu_message_falls_back_to_receive_timestamp_for_ros_time() -> None:
    """Verify non-Unix ROS timestamps are mapped to host receive time.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If ROS-time messages leak non-Unix timestamps.
    """
    receive_time_s = ROS_UNIX_TIMESTAMP_MINIMUM_S + 20.0

    state = _imu_state_from_ros_message(
        _fake_imu_message(2.5),
        receive_time_s,
    )

    assert state.ts == receive_time_s
