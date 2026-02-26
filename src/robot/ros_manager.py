import numpy as np
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
    """Publish joint trajectories while recording aligned encoder and IMU data.
    Args:
        bot_id: Robot identifier used to build ROS topic names.
        Ts: Sampling period for command publishing [s].
        time_data: Command timestamps [s].
        position_data: Joint position commands [rad].
        velocity_data: Joint velocity commands [rad/s].
        acceleration_data: Joint acceleration commands [rad/s^2].
        publish_complete_event: Event signaled after publishing completes.
        buffer_len_s: Buffer length for stored data [s].
    Returns:
        `None`.
    Side Effects:
        Creates ROS publishers/subscribers, timers, and data buffers.
    Raises:
        None.
    Preconditions:
        ROS is initialized and topic names are valid for the robot.
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
        """Initialize ROS publishers, subscribers, timers, and data buffers.
        Args:
            bot_id: Robot identifier used to build ROS topic names.
            Ts: Sampling period for command publishing [s].
            time_data: Command timestamps [s].
            position_data: Joint position commands [rad].
            velocity_data: Joint velocity commands [rad/s].
            acceleration_data: Joint acceleration commands [rad/s^2].
            publish_complete_event: Event signaled after publishing completes.
            buffer_len_s: Buffer length for stored data [s].
        Returns:
            `None`.
        Side Effects:
            Creates ROS publishers/subscribers and allocates buffers.
        Raises:
            None.
        Preconditions:
            ROS is initialized and topic names are valid.
        """
        super().__init__("joint_trajectory_controller")
        self.bot_id = bot_id  # Robot identifier used for topic name construction.

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
        self.state_data: deque[JointStateSample] = deque(
            maxlen=maxlen
        )  # Encoder buffer.
        self.imu_data: deque[ImuSample] = deque(maxlen=maxlen)  # IMU buffer.

        # We'll keep raw command-time stamps as well
        self.cmd_times: list[float] = []  # Command timestamps [s] aligned to publish.
        self.cmd_points: list[JointTrajectoryPoint] = []  # Commanded points per tick.

        # aligned logs: tuples of (timestamp, command_point, state_sample, imu_sample)
        self.aligned_log: deque[Any] = deque(
            maxlen=maxlen
        )  # Aligned sensor/command log.

        # Trajectory to send
        self.time_data = time_data  # Command timestamps [s].
        self.position_data = position_data  # Position commands [rad].
        self.velocity_data = velocity_data  # Velocity commands [rad/s].
        self.acceleration_data = acceleration_data  # Acceleration commands [rad/s^2].
        self.publish_complete_event = (
            publish_complete_event  # Signals publish completion.
        )

        # Timer to publish joint trajectory points and immediately sample data at Ts
        self.index = 0
        self.timer = None  # Active publish timer after sensors are ready.
        self._start_timer = None  # Startup timer that waits for sensor readiness.
        self._waiting_logged = False  # Log once while waiting for readiness.
        self._start_timer = self.create_timer(0.05, self._wait_for_ready_start)
        self._publish_period = Ts  # Publish period [s] used once ready.

    def _now_ros(self) -> float:
        """Return the current ROS time.
        Args:
            None.
        Returns:
            `float` current ROS time [s].
        Side Effects:
            None.
        Raises:
            None.
        Preconditions:
            ROS clock is initialized.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _wait_for_ready_start(self) -> None:
        """Gate trajectory publishing until encoder and IMU data are live.
        Args:
            None.
        Returns:
            `None`.
        Side Effects:
            Starts the publish timer once readiness is satisfied.
        Raises:
            None.
        Preconditions:
            Encoder and IMU callbacks are registered.
        """
        if (not self.encoder_is_ready()) or (not self.imu_is_ready()):
            if not self._waiting_logged:
                self.get_logger().info(
                    "Waiting for encoder/imu before starting trajectory publish..."
                )
                self._waiting_logged = True
            return

        if self._start_timer is not None:
            self._start_timer.cancel()
            self._start_timer = None

        if self.timer is None:
            self.timer = self.create_timer(
                self._publish_period, self.publish_and_record
            )
            self.get_logger().info("Joint Controller has Started.")

    def joint_state_callback(self, msg: JointState) -> None:
        """Append a joint state sample with receive timing.
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
        """Check whether recent joint state data is available.
        Args:
            None.
        Returns:
            `bool` True when encoder data is fresh.
        Side Effects:
            None.
        Raises:
            None.
        Preconditions:
            `self.state_data` is being populated by callbacks.
        """
        now = self._now_ros()
        return len(self.state_data) > 0 and self.state_data[-1].recv_time > now - 1

    def imu_callback(self, msg: Imu) -> None:
        """Append an IMU sample with receive timing.
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
        """Check whether recent IMU data is available.
        Args:
            None.
        Returns:
            `bool` True when IMU data is fresh.
        Side Effects:
            None.
        Raises:
            None.
        Preconditions:
            `self.imu_data` is being populated by callbacks.
        """
        now = self._now_ros()
        return len(self.imu_data) > 0 and self.imu_data[-1].recv_time > now - 1

    def publish_and_record(self) -> None:
        """Publish the next trajectory point and record aligned data.
        Args:
            None.
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

            # Publish exactly one point per tick to preserve the command cadence.
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
            if self.timer is not None:
                self.timer.cancel()
            self.get_logger().info("Trajectory publishing complete")
            # Align the data
            self.align_offline()
            self.get_logger().info("Aligned data")
            if self.publish_complete_event is not None:
                self.publish_complete_event.set()

    def align_offline(self) -> None:
        """Align recorded encoder and IMU data to command timestamps.
        Args:
            None.
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
            time_field: str,
        ) -> tuple[np.ndarray, np.ndarray]:
            """Extract timestamped arrays for a given field.

            Args:
                deq: Deque of samples.
                key: Attribute name to extract.
                time_field: Timestamp field to align against.

            Returns:
                `tuple[np.ndarray, np.ndarray]` of timestamps and values.

            Side Effects:
                None.

            Raises:
                AttributeError: If samples do not contain the requested field.

            Preconditions:
                `deq` contains objects with `time_field` and `key` attributes.
            """
            # Align with receive time to keep a single ROS-time base across streams.
            arr = np.array([getattr(d, key) for d in deq])
            stamps = np.array([getattr(d, time_field) for d in deq])
            return stamps, arr

        # Joint states
        # Use receive time for all sensor streams to avoid header-stamp drift.
        t_state, pos_state = extract(self.state_data, "positions", "recv_time")
        _, vel_state = extract(self.state_data, "velocities", "recv_time")
        _, eff_state = extract(self.state_data, "efforts", "recv_time")

        # IMU
        t_imu, accel_imu = extract(self.imu_data, "linear_acceleration", "recv_time")
        _, angvel_imu = extract(self.imu_data, "angular_velocity", "recv_time")
        _, quat_imu = extract(self.imu_data, "orientation", "recv_time")

        # For each command time, interpolate latest sensor readings
        # Each iteration aligns one command point with the nearest sensor samples.
        for t_cmd, point in zip(self.cmd_times, self.cmd_points):

            # find nearest or interpolate
            def interp(t: np.ndarray, x: np.ndarray) -> float:
                """Interpolate values at the command time.
                Args:
                    t: Time stamps array [s].
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