import numpy as np
import time
import threading
from collections import deque
from pydantic import BaseModel
from typing import Any, Optional
import rclpy  # noqa: F401

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState, Imu
from builtin_interfaces.msg import Duration as DurationMsg

# ---- NOTES ----
# This node publishes JointTrajectory messages to command robot joints
# and subscribes to JointState and Imu messages for feedback.
# It records incoming data in buffers and aligns them with command timestamps.
# ----------------
# HOW TO USE:
# 1. Update the topic names in JOINT_PUBLISHER, JOINT_SUBSCRIBER, IMU_SUBSCRIBER
#    to match your robot's ROS2 topics, replacing <BOT_ID> with your robot's ID, as needed.
# 2. Create an instance of JointTrajectoryController with the desired parameters.
# 3. Call rclpy.spin(your_instance) to run the node.
# 4. Access recorded and aligned data in your_instance.state_data, imu_data, and aligned_log.
# ----------------

JOINT_PUBLISHER = "/<BOT_ID>/joint_trajectory"
JOINT_SUBSCRIBER = "/<BOT_ID>/joint_state"
IMU_SUBSCRIBER = "/<BOT_ID>/end_effector_imu"


class JointStateSample(BaseModel):
    """Container for a single joint state sample with timestamps.

    Args:
        None.

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        None.
    """

    send_time: float
    recv_time: float
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


class ImuSample(BaseModel):
    """Container for a single IMU sample with timestamps.

    Args:
        None.

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        None.
    """

    send_time: float
    recv_time: float
    orientation: list[float]  # Quaternion as a list [x, y, z, w]
    orientation_covariance: list[float]
    angular_velocity: list[float]  # Angular velocity as a list [x, y, z]
    angular_velocity_covariance: list[float]
    linear_acceleration: list[float]  # Linear acceleration as a list [x, y, z]
    linear_acceleration_covariance: list[float]


class JointTrajectoryController(Node):
    """Publish joint trajectories and record aligned state/IMU data.

    Args:
        bot_id: Robot identifier used to build ROS topic names.
        Ts: Sampling time [s].
        time_data: Command timestamps.
        position_data: Joint position commands.
        velocity_data: Joint velocity commands.
        acceleration_data: Joint acceleration commands.
        publish_complete_event: Optional event signaled after publishing completes.
        buffer_len_s: Buffer length in seconds for stored data.

    Side Effects:
        Creates ROS publishers/subscribers and buffers data.

    Raises:
        None.

    Preconditions:
        ROS is initialized and topic names are valid.
    """

    def __init__(
        self,
        bot_id: str,
        Ts: float = 0.005,
        time_data: list = [],
        position_data: list = [],
        velocity_data: list = [],
        acceleration_data: list = [],
        publish_complete_event: Optional[threading.Event] = None,
        buffer_len_s: float = 3600,  # 60 minutes
    ) -> None:
        """Initialize ROS publishers, subscribers, and data buffers.

        Args:
            bot_id: Robot identifier used to build ROS topic names.
            Ts: Sampling time [s].
            time_data: Command timestamps.
            position_data: Joint position commands.
            velocity_data: Joint velocity commands.
            acceleration_data: Joint acceleration commands.
            publish_complete_event: Optional event signaled after publishing completes.
            buffer_len_s: Buffer length in seconds for stored data.

        Side Effects:
            Creates ROS publishers/subscribers and allocates buffers.

        Raises:
            None.

        Preconditions:
            ROS is initialized and topic names are valid.
        """
        super().__init__("joint_trajectory_controller")
        self.bot_id = bot_id

        # Callback groups so timer and subs can run in parallel
        cb = ReentrantCallbackGroup()

        # Publisher for joint trajectory commands
        self.joint_publisher = self.create_publisher(
            JointTrajectory,
            JOINT_PUBLISHER.replace("<BOT_ID>", bot_id),
            10,
            callback_group=cb,
        )

        qos = QoSProfile(
            depth=10,  # equivalent to keep_last(1)
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscription for joint state feedback
        self.joint_subscriber = self.create_subscription(
            JointState,
            JOINT_SUBSCRIBER.replace("<BOT_ID>", bot_id),
            self.joint_state_callback,
            qos,
            callback_group=cb,
        )

        # Subscription for imu data
        self.imu_subscriber = self.create_subscription(
            Imu,
            IMU_SUBSCRIBER.replace("<BOT_ID>", bot_id),
            self.imu_callback,
            10,
            callback_group=cb,
        )

        # Buffers to store incoming data
        maxlen = int(buffer_len_s * (1.0 / Ts))
        self.state_data: deque[JointStateSample] = deque(maxlen=maxlen)
        self.imu_data: deque[ImuSample] = deque(maxlen=maxlen)

        # We'll keep raw command-time stamps as well
        self.cmd_times: list[float] = []
        self.cmd_points: list[JointTrajectoryPoint] = []

        # aligned logs: tuples of (timestamp, command_point, state_sample, imu_sample)
        self.aligned_log: deque[Any] = deque(maxlen=maxlen)

        # Trajectory to send
        self.time_data = time_data
        self.position_data = position_data
        self.velocity_data = velocity_data
        self.acceleration_data = acceleration_data
        self.publish_complete_event = publish_complete_event

        # Timer to publish joint trajectory points and immediately sample data at Ts
        self.index = 0
        self.timer = self.create_timer(Ts, self.publish_and_record)
        self.get_logger().info("Joint Controller has Started.")

    def joint_state_callback(self, msg: JointState) -> None:
        """Handle incoming joint state messages.

        Args:
            msg: ROS JointState message.

        Returns:
            `None`.

        Side Effects:
            Appends a `JointStateSample` to `self.state_data`.

        Raises:
            None.

        Preconditions:
            ROS node is active.
        """
        sample = JointStateSample(
            send_time=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            recv_time=self.get_clock().now().nanoseconds * 1e-9,
            positions=msg.position,
            velocities=msg.velocity,
            efforts=msg.effort,
        )
        self.state_data.append(sample)

    def encoder_is_ready(self) -> bool:
        """Check if recent joint state data has been received.

        Returns:
            `bool` indicating whether encoder data is fresh.

        Side Effects:
            None.

        Raises:
            None.

        Preconditions:
            `self.state_data` is being populated by callbacks.
        """
        return (
            len(self.state_data) > 0 and self.state_data[-1].recv_time > time.time() - 1
        )

    def imu_callback(self, msg: Imu) -> None:
        """Handle incoming IMU messages.

        Args:
            msg: ROS Imu message.

        Returns:
            `None`.

        Side Effects:
            Appends an `ImuSample` to `self.imu_data`.

        Raises:
            None.

        Preconditions:
            ROS node is active.
        """
        sample = ImuSample(
            send_time=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            recv_time=self.get_clock().now().nanoseconds * 1e-9,
            orientation=[
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
            orientation_covariance=list(msg.orientation_covariance),
            angular_velocity=[
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            angular_velocity_covariance=list(msg.angular_velocity_covariance),
            linear_acceleration=[
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            linear_acceleration_covariance=list(msg.linear_acceleration_covariance),
        )
        self.imu_data.append(sample)

    def imu_is_ready(self) -> bool:
        """Check if recent IMU data has been received.

        Returns:
            `bool` indicating whether IMU data is fresh.

        Side Effects:
            None.

        Raises:
            None.

        Preconditions:
            `self.imu_data` is being populated by callbacks.
        """
        return len(self.imu_data) > 0 and self.imu_data[-1].recv_time > time.time() - 1

    def publish_and_record(self) -> None:
        """Publish trajectory points and record synchronized data.

        Returns:
            `None`.

        Side Effects:
            Publishes ROS messages, updates buffers, and signals completion.

        Raises:
            None.

        Preconditions:
            `encoder_is_ready()` and `imu_is_ready()` return True before publishing.
        """
        # wait until we're recording data before starting trajectory
        if (not self.encoder_is_ready()) or (not self.imu_is_ready()):
            self.get_logger().info("encoder/imu is not ready, retrying")
            return

        # 1) if there’s still trajectory left, publish next point
        if self.index < len(self.position_data):
            now = self.get_clock().now()
            point = JointTrajectoryPoint()
            point.positions = self.position_data[self.index]
            point.velocities = (
                self.velocity_data[self.index] if len(self.velocity_data) > 0 else []
            )
            point.accelerations = (
                self.acceleration_data[self.index]
                if len(self.acceleration_data) > 0
                else []
            )

            # Use provided time_data to set the time_from_start field [seconds]
            point.time_from_start = DurationMsg(
                sec=int(now.nanoseconds * 1e-9), nanosec=int(now.nanoseconds % 1e9)
            )

            msg = JointTrajectory()
            msg.joint_names = [f"joint{i}" for i in range(len(point.positions))]
            msg.points.append(point)
            self.joint_publisher.publish(msg)

            # record raw command timestamp and point
            self.cmd_times.append(now.nanoseconds * 1e-9)
            self.cmd_points.append(point)

            self.index += 1
        else:
            # Once the trajectory is complete, cancel the timer
            self.timer.cancel()
            self.get_logger().info("Trajectory publishing complete")
            # Align the data
            self.align_offline()
            self.get_logger().info("Aligned data")
            if self.publish_complete_event is not None:
                self.publish_complete_event.set()

    def align_offline(self) -> None:
        """Align recorded state and IMU data to command timestamps.

        Returns:
            `None`.

        Side Effects:
            Appends aligned records to `self.aligned_log`.

        Raises:
            RuntimeError: If `time_data` is not provided.

        Preconditions:
            `self.cmd_times` and sensor buffers are populated.
        """
        if self.time_data is None:
            raise RuntimeError("time_data must be provided for offline alignment")

        # Helper to extract arrays from deque of dicts
        def extract(
            deq: deque[JointStateSample] | deque[ImuSample],
            key: str,
        ) -> tuple[np.ndarray, np.ndarray]:
            """Extract timestamped arrays for a given field.

            Args:
                deq: Deque of samples.
                key: Attribute name to extract.

            Returns:
                `tuple[np.ndarray, np.ndarray]` of timestamps and values.

            Side Effects:
                None.

            Raises:
                AttributeError: If samples do not contain the requested field.

            Preconditions:
                `deq` contains objects with a `send_time` field and `key` attribute.
            """
            arr = np.array([getattr(d, key) for d in deq])
            stamps = np.array([getattr(d, "send_time") for d in deq])
            return stamps, arr

        # Joint states
        t_state, pos_state = extract(self.state_data, "positions")
        _, vel_state = extract(self.state_data, "velocities")
        _, eff_state = extract(self.state_data, "efforts")

        # IMU
        t_imu, accel_imu = extract(self.imu_data, "linear_acceleration")
        _, angvel_imu = extract(self.imu_data, "angular_velocity")
        _, quat_imu = extract(self.imu_data, "orientation")

        # For each command time, interpolate latest sensor readings
        for t_cmd, point in zip(self.cmd_times, self.cmd_points):

            # find nearest or interpolate
            def interp(t: np.ndarray, x: np.ndarray) -> float:
                """Interpolate values at the command time.

                Args:
                    t: Time stamps array.
                    x: Values array aligned with `t`.

                Returns:
                    `float` interpolated value at the current command time.

                Side Effects:
                    None.

                Raises:
                    None.

                Preconditions:
                    `t` and `x` are the same length and sorted by time.
                """
                return float(np.interp(t_cmd, t, x, left=np.nan, right=np.nan))

            pos_i = np.array(
                [interp(t_state, pos_state[:, j]) for j in range(pos_state.shape[1])]
            )
            vel_i = np.array(
                [interp(t_state, vel_state[:, j]) for j in range(vel_state.shape[1])]
            )
            eff_i = np.array(
                [interp(t_state, eff_state[:, j]) for j in range(eff_state.shape[1])]
            )
            accel_i = np.array(
                [interp(t_imu, accel_imu[:, j]) for j in range(accel_imu.shape[1])]
            )
            angv_i = np.array(
                [interp(t_imu, angvel_imu[:, j]) for j in range(angvel_imu.shape[1])]
            )
            quat_i = np.array(
                [interp(t_imu, quat_imu[:, j]) for j in range(quat_imu.shape[1])]
            )

            self.aligned_log.append(
                {
                    "cmd_time": t_cmd,
                    "input_positions": point.positions,
                    "output_positions": pos_i,
                    "velocities": vel_i,
                    "efforts": eff_i,
                    "imu_time": t_cmd,
                    "linear_acceleration": accel_i,
                    "angular_velocity": angv_i,
                    "orientation": quat_i,
                }
            )
