"""ROS-backed IMU recorder for robot-native IMU topics."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any

from reforge_core.hw_interfaces.arm_client import ArmClient
from reforge_core.hw_interfaces.imu_recorder import ImuRecorder
from reforge_core.imu.data import IMUState, IMUStateTraj
from robot.ros_topics import IMU_TOPIC_TEMPLATE

DEFAULT_ROS_IMU_TOPIC_TEMPLATE = IMU_TOPIC_TEMPLATE
ROS_UNIX_TIMESTAMP_MINIMUM_S = 946684800.0
ROS_IMU_READY_TIMEOUT_S = 5.0
ROS_SPIN_PERIOD_S = 0.01


def _stamp_to_seconds(stamp: Any) -> float:
    """Return a ROS timestamp as seconds.

    Args:
        stamp: ROS message timestamp with `sec` and `nanosec` fields.

    Returns:
        `float` timestamp [s].

    Raises:
        AttributeError: If `stamp` does not expose ROS timestamp fields.
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _imu_state_from_ros_message(message: Any, receive_time_s: float) -> IMUState:
    """Return one standardized IMU sample from a ROS `sensor_msgs/Imu`.

    Args:
        message: ROS IMU message whose acceleration is [m/s^2] and angular
            velocity is [rad/s].
        receive_time_s: Host Unix receive timestamp [s].

    Returns:
        `IMUState` sample in SI units on the host Unix timeline.

    Raises:
        AttributeError: If `message` is missing `sensor_msgs/Imu` fields.
    """
    message_time_s = _stamp_to_seconds(message.header.stamp)
    timestamp_s = (
        message_time_s
        if message_time_s >= ROS_UNIX_TIMESTAMP_MINIMUM_S
        else receive_time_s
    )
    return IMUState(
        ax=float(message.linear_acceleration.x),
        ay=float(message.linear_acceleration.y),
        az=float(message.linear_acceleration.z),
        gx=float(message.angular_velocity.x),
        gy=float(message.angular_velocity.y),
        gz=float(message.angular_velocity.z),
        ts=float(timestamp_s),
    )


class RosImuRecorder(ImuRecorder):
    """Record robot-native IMU samples from a ROS topic.

    Args:
        bot_id: Robot identifier used to resolve `<BOT_ID>` in the topic
            template.
        topic_template: ROS topic template for `sensor_msgs/Imu` messages.
        ready_timeout_s: Maximum preparation wait for the first IMU sample [s].

    Raises:
        None.
    """

    def __init__(
        self,
        *,
        bot_id: str,
        topic_template: str = DEFAULT_ROS_IMU_TOPIC_TEMPLATE,
        ready_timeout_s: float = ROS_IMU_READY_TIMEOUT_S,
    ) -> None:
        """Store ROS IMU subscription settings.

        Args:
            bot_id: Robot identifier used in the IMU topic path.
            topic_template: Topic template containing `<BOT_ID>`.
            ready_timeout_s: Maximum wait for first IMU sample during prepare [s].

        Returns:
            `None`.

        Raises:
            ValueError: If `bot_id` is empty.
        """
        if not bot_id:
            raise ValueError("bot_id must be non-empty for ROS IMU recording.")
        self.topic = topic_template.replace("<BOT_ID>", bot_id)
        self.ready_timeout_s = float(ready_timeout_s)
        self._is_prepared = False
        self._is_recording = False
        self._latest_state: IMUState | None = None
        self._recorded_states: deque[IMUState] = deque()
        self._lock = threading.Lock()
        self._spin_stop = threading.Event()
        self._spin_thread: threading.Thread | None = None
        self._node: Any | None = None
        self._rclpy: Any | None = None

    @property
    def is_prepared(self) -> bool:
        """Return whether the ROS subscription has received an IMU sample.

        Returns:
            `bool` preparation state.

        Raises:
            None.
        """
        return self._is_prepared

    def prepare(
        self,
        *,
        arm: ArmClient,
        arm_recording_hz: float,
        force_recalibration: bool,
    ) -> None:
        """Create the ROS subscription and wait for live IMU data.

        Args:
            arm: Arm client sharing the host Unix timestamp timeline.
            arm_recording_hz: Arm recording frequency [Hz].
            force_recalibration: Whether to rebuild the ROS subscription even
                if it is already prepared.

        Returns:
            `None`.

        Raises:
            RuntimeError: If ROS dependencies are missing or no IMU sample
                arrives before `ready_timeout_s`.
        """
        del arm, arm_recording_hz
        if self._is_prepared and not force_recalibration:
            return
        if force_recalibration:
            self._close_ros_node()
        self._ensure_ros_subscription()
        deadline_s = time.time() + self.ready_timeout_s

        # Wait for the subscription callback to receive at least one sample.
        while time.time() < deadline_s:
            with self._lock:
                has_latest_state = self._latest_state is not None
            if has_latest_state:
                self._is_prepared = True
                return
            time.sleep(ROS_SPIN_PERIOD_S)
        raise RuntimeError(
            f"No ROS IMU samples received on topic '{self.topic}' within "
            f"{self.ready_timeout_s:.1f} s."
        )

    def start_recording(self) -> None:
        """Start one fresh ROS IMU capture.

        Returns:
            `None`.

        Raises:
            RuntimeError: If `prepare()` has not completed.
        """
        if not self._is_prepared:
            raise RuntimeError("Prepare the ROS IMU before starting recording.")
        with self._lock:
            self._recorded_states.clear()
            self._is_recording = True

    def stop_recording(self) -> IMUStateTraj:
        """Stop ROS IMU capture and return recorded samples.

        Returns:
            `IMUStateTraj` samples in callback arrival order.

        Raises:
            RuntimeError: If no samples were captured.
        """
        with self._lock:
            self._is_recording = False
            states = list(self._recorded_states)
        trajectory = IMUStateTraj(maxlen=max(1, len(states)))
        for state in states:
            trajectory.append(state)
        if len(trajectory) == 0:
            raise RuntimeError("ROS IMU recording completed with no samples.")
        return trajectory

    def get_latest_state(self) -> IMUState:
        """Return the newest ROS IMU sample.

        Returns:
            `IMUState` latest ROS IMU sample.

        Raises:
            RuntimeError: If no IMU sample has arrived.
        """
        with self._lock:
            latest_state = self._latest_state
        if latest_state is None:
            raise RuntimeError("No ROS IMU sample is available.")
        return latest_state

    def _ensure_ros_subscription(self) -> None:
        """Start the ROS node used for IMU subscription callbacks.

        Returns:
            `None`.

        Raises:
            RuntimeError: If ROS Python packages cannot be imported.
        """
        if self._node is not None:
            return
        try:
            import rclpy
            from sensor_msgs.msg import Imu
        except ImportError as exc:
            raise RuntimeError(
                "ROS IMU recording requires rclpy and sensor_msgs to be installed."
            ) from exc

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node("reforge_ros_imu_recorder")
        self._node.create_subscription(Imu, self.topic, self._imu_callback, 10)
        self._spin_stop.clear()
        self._spin_thread = threading.Thread(
            target=self._spin_until_stopped,
            name="reforge_ros_imu_recorder_spin",
            daemon=True,
        )
        self._spin_thread.start()

    def _spin_until_stopped(self) -> None:
        """Spin the ROS node until recorder shutdown.

        Returns:
            `None`.

        Raises:
            None.
        """
        while not self._spin_stop.is_set():
            if self._rclpy is None or self._node is None:
                return
            try:
                self._rclpy.spin_once(self._node, timeout_sec=ROS_SPIN_PERIOD_S)
            except Exception as exc:
                if exc.__class__.__name__ == "ExternalShutdownException":
                    return
                raise

    def _imu_callback(self, message: Any) -> None:
        """Append a standardized sample from one ROS IMU callback.

        Args:
            message: `sensor_msgs/Imu` callback payload.

        Returns:
            `None`.

        Raises:
            None.
        """
        state = _imu_state_from_ros_message(message, time.time())
        with self._lock:
            self._latest_state = state
            if self._is_recording:
                self._recorded_states.append(state)

    def _close_ros_node(self) -> None:
        """Destroy the recorder-owned ROS node.

        Returns:
            `None`.

        Raises:
            None.
        """
        self._spin_stop.set()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._is_prepared = False
