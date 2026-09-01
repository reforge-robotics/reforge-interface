from robot.ros_topics import (
    IMU_TOPIC_TEMPLATE,
    JOINT_COMMAND_TOPIC_TEMPLATE,
    JOINT_STATE_TOPIC_TEMPLATE,
)


def test_standardbots_ros_topics_use_hardware_namespace() -> None:
    """Verify Standard Bots ROS topic templates target hardware controllers.

    Args:
        None.

    Returns:
        `None`.

    Raises:
        AssertionError: If a topic template drifts away from the hardware
            namespace used by the robot controller.
    """
    bot_id = "customer_bot"

    assert (
        JOINT_COMMAND_TOPIC_TEMPLATE.replace("<BOT_ID>", bot_id)
        == f"/{bot_id}/ro1/hardware/joint_trajectory"
    )
    assert (
        JOINT_STATE_TOPIC_TEMPLATE.replace("<BOT_ID>", bot_id)
        == f"/{bot_id}/ro1/hardware/joint_state"
    )
    assert (
        IMU_TOPIC_TEMPLATE.replace("<BOT_ID>", bot_id)
        == f"/{bot_id}/ro1/hardware/end_effector_imu"
    )
