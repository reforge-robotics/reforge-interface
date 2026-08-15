"""Shared Standard Bots ROS topic templates."""

# ROS topic for `trajectory_msgs/JointTrajectory` joint commands.
JOINT_COMMAND_TOPIC_TEMPLATE = "/<BOT_ID>/ro1/hardware/joint_trajectory"
# ROS topic for `sensor_msgs/JointState` hardware feedback.
JOINT_STATE_TOPIC_TEMPLATE = "/<BOT_ID>/ro1/hardware/joint_state"
# ROS topic for `sensor_msgs/Imu` end-effector IMU feedback.
IMU_TOPIC_TEMPLATE = "/<BOT_ID>/ro1/hardware/end_effector_imu"
