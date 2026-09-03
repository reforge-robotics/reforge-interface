# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0

import threading
import time
from importlib.resources import files, as_file
from typing import Any, Optional, Sequence, TypeVar, cast

import numpy as np

from reforge_core.hw_interfaces.arm_client import ArmClient
from reforge_core.hw_interfaces.imu_recorder import ImuRecorder

# ------NOTES-----
# 1. Where you see the #{~.~} symbol, you need to make a change. Use Ctrl+F to find all instances.
# The general flow will be the following:
#   a. Import the robot's Python SDK
#   b. Change the BOT_ID, URDF_PATH, ROBOT_MAX_FREQ, and
#      FULL_STRETCH_SHOULDER_ANGLE, FULL_STRETCH_XYZ, FULL_STRETCH_QUAT, and FULL_STRETCH_JOINTS constants
#   c. Change the IS_DEGREES constant if the robot uses degrees instead of radians
#   d. Change the code in the REQUIRED METHODS section to use the robot's SDK
# 2. The REQUIRED METHODS section contains methods that must be implemented for the robot to work with the
#    system identification and calibration workflow. The rest of the methods are pre-defined and should not
#    need to be changed.
# 3. The code contains examples for Standard Bots' robots, which can be used as a reference.
# 4. If you opt to use ROS for publishing joint positions, you can use the ros_manager.py file
# in the robots folder. See detailed instructions in that file.

# {~.~} Import robot's Python SDK with required modules here

# ------------------------------- EXAMPLE -------------------------------
# from standardbots import StandardBotsRobot, models
# https://docs.standardbots.com/docs/latest/-/rest/intro/configuring-sdk
# -----------------------------------------------------------------------


# User constants - EDITS REQUIRED
# The customer supplies its robot identifier at invocation time.
BOT_ID = ""
URDF_PATH = "urdf/modelone.urdf"
ROBOT_MAX_FREQ = 200  # [Hz]

# Fully stretched position of the robot for calibration.
FULL_STRETCH_XYZ = [1.28989, 0.36866, 0.171]  # {~.~} [m]
FULL_STRETCH_QUAT = [0.499, 0.499, 0.499, 0.499]  # {~.~} [1]
FULL_STRETCH_JOINTS = [0.0, np.pi / 2, 0.0, 0.0, 0.0, 0.0]  # {~.~} [rad]
FULL_STRETCH_POSE_OVERRIDE = None  # {~.~} list of home pose (xyz and quaternion) to override additional height not in base height

# General constants
IS_DEGREES = False  # {~.~} [CHANGE TO TRUE IF ROBOT USES DEGREES]
DATA_LOCATION_PREFIX = "src/robot/data"  # {~.~} [CHANGE TO LOCATION DESIRED - will be robot/DATA_LOCATION_PREFIX/*]
DEFAULT_TCP_PAYLOAD = 0.0  # {~.~} [CHANGE IF THE DEFAULT PAYLOAD IS NON_ZERO]

MAX_ROBOT_JOINTS_BANDWIDTH = 7.5  # Servo motor bandwidth [Hz].

HTTP_HEADER = "http://"
T = TypeVar("T")
ROS_SERVO_POINT_DURATION_S = 0.02  # [s] short relative horizon for streamed ROS points.
ROS_SERVO_DISCOVERY_TIMEOUT_S = 2.0
ROS_SERVO_DISCOVERY_POLL_S = 0.02

# {~.~} IMU information
USE_REFORGE_IMU = False
DEFAULT_IMU_COMM_MODE = "usb"
DEFAULT_IMU_RECORD_FREQUENCY_HZ = ROBOT_MAX_FREQ


def _standardbots_url(robot_ip: str) -> str:
    """Return a Standard Bots SDK URL for a robot host.

    Args:
        robot_ip: Robot hostname, IP address, or fully qualified URL.

    Returns:
        `str` URL with an HTTP scheme.

    Side Effects:
        None.

    Raises:
        ValueError: If `robot_ip` is empty.

    Preconditions:
        `robot_ip` is not the simulator sentinel `sim`.
    """
    if not robot_ip:
        raise ValueError("robot_ip must be non-empty for live Standard Bots mode.")
    if robot_ip.startswith(("http://", "https://")):
        return robot_ip
    return HTTP_HEADER + robot_ip


def _require(value: T | None, message: str) -> T:
    """Return `value` or raise when the Standard Bots SDK omitted telemetry.

    Args:
        value: Optional value returned by the SDK.
        message: Error message for missing values.

    Returns:
        `T` non-None value.

    Side Effects:
        None.

    Raises:
        RuntimeError: If `value` is `None`.

    Preconditions:
        None.
    """
    if value is None:
        raise RuntimeError(message)
    return value


def _joint_values_in_radians(joint_values: Sequence[float]) -> list[float]:
    """Return joint positions in radians.

    Args:
        joint_values: Joint values from the SDK [rad] or [deg], depending on `IS_DEGREES`.

    Returns:
        `list[float]` joint values [rad].

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        All joint values are finite numeric angles.
    """
    if IS_DEGREES:
        return [float(np.deg2rad(angle)) for angle in joint_values]
    return [float(angle) for angle in joint_values]


def _joint_values_for_sdk(joint_values: Sequence[float]) -> tuple[float, ...]:
    """Return joint positions in the angular units expected by the SDK.

    Args:
        joint_values: Joint positions [rad].

    Returns:
        `tuple[float, ...]` joint positions [rad] or [deg], depending on `IS_DEGREES`.

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        All joint values are finite numeric angles.
    """
    if IS_DEGREES:
        return tuple(float(np.rad2deg(angle)) for angle in joint_values)
    return tuple(float(angle) for angle in joint_values)


class RobotInterface(ArmClient):
    """Provide a concrete robot implementation for system identification and calibration.

    Args:
        robot_ip: Robot internet protocol address or `sim` for simulator mode.
        tcp_payload: Optional payload of the robot for NN prediction of
            payload changes.
        tcp_payload_com: Optional 3x1 center of mass location of the
            payload, defined relative to the origin of the TCP [meters].
        local_ip: Local internet protocol address for networked setups.
        sdk_token: Authentication token for the robot software development kit.
        robot_id: Identifier for the robot in the control stack.

    Side Effects:
        Loads the robot model from the configured Unified Robot Description Format file.
        Connects to the robot hardware when not in simulator mode.

    Raises:
        RuntimeError: If the robot connection fails or required telemetry is missing.
        ValueError: If reported joint counts do not match the loaded model.

    Preconditions:
        The robot software development kit is installed and the Unified Robot
        Description Format file path is valid.
    """

    def __init__(
        self,
        robot_ip: str,
        local_ip: str = "",
        sdk_token: str = "",
        api_token: str = "",
        robot_id: str = BOT_ID,
        use_reforge_imu: bool = USE_REFORGE_IMU,
        imu_recorder: ImuRecorder | None = None,
        tcp_payload: float = DEFAULT_TCP_PAYLOAD,
        tcp_payload_com: Sequence[float] | None = None,
    ) -> None:
        """Initialize the robot interface and load the URDF model.

        Args:
            robot_ip: Robot IP address or `sim` for simulator mode.
            tcp_payload: Payload of the tcp (default=0)
            tcp_payload_com: Optional 3x1 center of mass of the tcp payload.
            local_ip: Local machine IP address if required by the SDK.
            sdk_token: SDK authentication token.
            api_token: Reforge API token.
            robot_id: Reforge robot ID (most cases) or SDK identifier used by the control stack.
            use_reforge_imu: Whether to use the built-in Reforge IMU backend
                when `imu_recorder` is not supplied.
            imu_recorder: Optional vendor-specific recorder supplied directly
                by an application or integration test.
            tcp_payload: payload of end-effector [kg].
            tcp_payload_com: (N x 3) center of mass of end-effector payload [m].

        Side Effects:
            Loads the URDF model and may connect to robot hardware.

        Raises:
            RuntimeError: If the robot connection fails.
            ValueError: If reported joint counts do not match the URDF.

        Preconditions:
            The URDF file is available and the SDK is installed.
        """
        super().__init__(
            name="Standard Bots", recording_data_frequency_hz=ROBOT_MAX_FREQ
        )

        self.max_sampling_frequency_hz = ROBOT_MAX_FREQ
        self.data_folder_prefix = DATA_LOCATION_PREFIX
        self.servo_bandwidth_hz = MAX_ROBOT_JOINTS_BANDWIDTH
        self.calibration_start_joints = FULL_STRETCH_JOINTS
        self.calibration_start_quat = FULL_STRETCH_QUAT
        self.calibration_start_xyz = FULL_STRETCH_XYZ
        self.full_stretch_pose_override = FULL_STRETCH_POSE_OVERRIDE

        # Initialize URDF location
        self.module_dir = files("robot")
        resource = self.module_dir.joinpath(URDF_PATH)
        with as_file(resource) as p:
            self._urdf_path = str(p)
        print(f"URDF Path: {self._urdf_path}")

        # Is robot in simulation mode?
        self._in_sim_mode = robot_ip == "sim"

        # Load robot model from URDF
        if not self.model_is_loaded:
            self.model = self.initialize_model_from_urdf(
                urdf_path=self.urdf_path,
                tcp_payload=tcp_payload,
                tcp_payload_com=tcp_payload_com,
            )
            # Use the model joint count as the ground truth for downstream
            # dynamics calls (the hardware may report extra fixed joints/grippers).
            self.num_joints = self.model.num_joints

        self.use_reforge_imu = use_reforge_imu
        self._ros_initialized_by_interface = False
        self._ros_servo_node: Any | None = None
        self._ros_joint_publisher: Any | None = None
        self._ros_servo_executor: Any | None = None
        self._ros_servo_spin_thread: threading.Thread | None = None
        self._ros_servo_topic = ""
        self._ros_servo_missing_subscriber_warning_printed = False

        # Reforge API and robot ID token is needed for "joint_tracker" product
        # Add it in the CLI with `--identify`
        self.reforge_api_token = api_token
        try:
            if robot_ip != "sim":
                StandardBotsRobot, models = self._load_standardbots_sdk()
                self._standardbots_models = models
                self.robot = StandardBotsRobot(
                    url=_standardbots_url(robot_ip),
                    token=sdk_token,
                    robot_kind=StandardBotsRobot.RobotKind.Live,
                )

                self.enter_position_mode()
                self.robot.movement.brakes.unbrake().ok()

                # Set ID for robot
                self.id = robot_id

                # Should be equivalent to Dynamics model joints
                num_joints_sdk = len(self._get_joint_positions())
                if num_joints_sdk != self.num_joints:
                    raise RuntimeError(
                        f"Number of robot joints in URDF ({self.num_joints}) is not equivalent to the number"
                        f"of joints returned by the robot SDK ({num_joints_sdk})."
                    )
                self.pose_length = len(self.get_tcp_pose())
            else:
                # Simulation mode
                self.id = robot_id
                self.num_joints = 6
                self.pose_length = 7
                self._standardbots_models = None

        except Exception as e:
            # Print exception error message
            raise RuntimeError(f"Error getting {robot_ip} operational: {str(e)}")

        if not self.imu_manager_is_loaded:
            selected_imu_recorder = imu_recorder
            if selected_imu_recorder is None and not self.use_reforge_imu:
                selected_imu_recorder = self.create_robot_imu_recorder()
            self.use_reforge_imu = selected_imu_recorder is None
            self.arm_imu_manager = self.initialize_arm_imu_manager(
                arm_sample_time_s=1.0 / ROBOT_MAX_FREQ,
                imu_comm_mode=DEFAULT_IMU_COMM_MODE,
                imu_record_frequency_hz=DEFAULT_IMU_RECORD_FREQUENCY_HZ,
                imu_recorder=selected_imu_recorder,
            )

    def create_robot_imu_recorder(self) -> ImuRecorder:
        """Create the robot-native IMU adapter used when Reforge IMU is disabled.

        Standard Bots exposes its internal end-effector IMU through ROS. This
        hook adapts that ROS topic to the shared `ImuRecorder` contract so
        `ArmImuManager` can keep coordinating arm recording, IMU recording, and
        sample alignment.

        Returns:
            `ImuRecorder` backed by the robot's ROS IMU topic.

        Raises:
            RuntimeError: If ROS dependencies are unavailable when the recorder
                is prepared for acquisition.
        """
        from robot.ros_imu_recorder import RosImuRecorder

        return RosImuRecorder(bot_id=self.id)

    @property
    def in_sim_mode(self) -> bool:
        """Return whether the interface is running in simulator mode.

        Returns:
            `bool` indicating simulator mode.

        Side Effects:
            None.

        Raises:
            None.

        Preconditions:
            None.
        """
        return self._in_sim_mode

    @property
    def urdf_path(self) -> str:
        """Return the absolute path to the URDF file.

        Returns:
            `str` path to the URDF file.

        Side Effects:
            None.

        Raises:
            None.

        Preconditions:
            The URDF path has been initialized.
        """
        return self._urdf_path

    # {~.~} REQUIRED METHODS
    @staticmethod
    def _load_standardbots_sdk() -> tuple[Any, Any]:
        """Load the Standard Bots SDK classes used by the live adapter.

        Returns:
            `tuple[Any, Any]` containing `StandardBotsRobot` and SDK models.

        Side Effects:
            Imports the Standard Bots SDK package.

        Raises:
            ImportError: If the `standardbots` package is unavailable.

        Preconditions:
            The robot package dependencies are installed for live Standard Bots mode.
        """
        from standardbots import StandardBotsRobot, models

        return StandardBotsRobot, models

    def _models(self) -> Any:
        """Return the loaded Standard Bots SDK models module.

        Returns:
            `Any` SDK models module.

        Side Effects:
            None.

        Raises:
            RuntimeError: If live Standard Bots mode has not loaded the SDK.

        Preconditions:
            The robot is not running in simulator mode.
        """
        models = self._standardbots_models
        if models is None:
            raise RuntimeError(
                "Standard Bots SDK models are unavailable in simulation mode."
            )
        return models

    def command_move_j(
        self,
        target_joints: np.ndarray | list[float],
        *,
        speed: float = 50.0,
        wait: bool = True,
    ) -> int:
        """Send a blocking/non-blocking point-to-point joint command using the
        robot's native position control interface.

        Args:
            target_joints: Target joint positions [rad] as a list or array.
            speed: Speed percentage for the motion, if supported by the robot. Default is 50%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.

        Returns:
            An integer status code from the robot's command interface, if applicable.
            If the robot does not provide a status code, return 0 for success or raise an exception for failure.
        """
        arm = self._require_connected_arm()
        models = self._models()
        self.enter_position_mode()

        joint_tuple = cast(
            Any,
            _joint_values_for_sdk(target_joints),
        )
        update_request = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
            joint_rotation=models.ArmJointRotations(joints=joint_tuple),
        )
        arm.movement.position.set_arm_position(body=update_request).ok()
        return 0

    def command_move_pose(
        self,
        target_quat: np.ndarray | list[float],
        target_xyz: np.ndarray | list[float],
        *,
        speed: float = 50.0,
        wait: bool = True,
    ) -> int:
        """Send a blocking/non-blocking point-to-point pose command using the
        robot's native position control interface.

        Args:
            target_quat: Target TCP orientation as a quaternion `[qx, qy, qz, qw]` [-] in the robot's base frame.
            target_xyz: Target TCP position `[x, y, z]` [m] in the robot's base frame.
            speed: Speed percentage for the motion, if supported by the robot. Default is 50%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.

        Returns:
            An integer status code from the robot's command interface, if applicable.
            If the robot does not provide a status code, return 0 for success or raise an exception for failure.
        """
        arm = self._require_connected_arm()
        models = self._models()
        self.enter_position_mode()

        quatx, quaty, quatz, quatw = [float(value) for value in target_quat]
        x, y, z = [float(value) for value in target_xyz]
        move_quat = models.Orientation(
            kind=models.OrientationKindEnum.Quaternion,
            quaternion=models.Quaternion(x=quatx, y=quaty, z=quatz, w=quatw),
        )
        move_xyz = models.Position(
            unit_kind=models.LinearUnitKind.Meters,
            x=x,
            y=y,
            z=z,
        )
        update_request = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.TooltipPosition,
            tooltip_position=models.PositionAndOrientation(
                position=move_xyz,
                orientation=move_quat,
            ),
        )

        arm.movement.position.set_arm_position(body=update_request).ok()
        return 0

    def command_servo_j(
        self,
        target_joints: np.ndarray | list[float],
        *,
        wait: bool = False,
    ) -> int:
        """Send one servo command in radians.

        This function will be used to stream a sequence of positions in a for loop in the calibration routine.

        Args:
            target_joints: Target joint position [rad] as a list or array.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.

        Returns:
            `int` status code, where `0` indicates the ROS command was published.

        Raises:
            RuntimeError: If the ROS servo publisher cannot be initialized.
        """
        if self._ros_joint_publisher is None:
            self.enter_servo_mode()
            self._ensure_ros_servo_publisher()

        from builtin_interfaces.msg import Duration as DurationMsg
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        target_joints_sdk_units = _joint_values_for_sdk(target_joints)
        point = JointTrajectoryPoint()
        point.positions = list(target_joints_sdk_units)
        point.time_from_start = DurationMsg(
            sec=0,
            nanosec=int(ROS_SERVO_POINT_DURATION_S * 1e9),
        )

        message = JointTrajectory()
        message.header.stamp = self._ros_servo_node.get_clock().now().to_msg()
        message.joint_names = [f"joint{i}" for i in range(len(point.positions))]
        message.points.append(point)
        self._ros_joint_publisher.publish(message)
        subscriber_count = self._ros_joint_publisher.get_subscription_count()
        if (
            subscriber_count == 0
            and not self._ros_servo_missing_subscriber_warning_printed
        ):
            print(
                "Warning: publishing ROS servo commands on "
                f"{self._ros_servo_topic}, but no subscribers are visible."
            )
            self._ros_servo_missing_subscriber_warning_printed = True
        return 0

    def command_joint_trajectory(
        self,
        time_data: Sequence[float],
        position_stream: Sequence[Sequence[float]],
        velocity_stream: Sequence[Sequence[float]] | None = None,
        acceleration_stream: Sequence[Sequence[float]] | None = None,
        Ts: float = 1 / ROBOT_MAX_FREQ,
    ) -> list[tuple[float, list[float]]]:
        """Publish a complete ROS joint trajectory and return command timing.

        Standard Bots' ROS trajectory controller expects trajectory-level
        timing and optional velocity/acceleration fields. This adapter keeps the
        base calibration pipeline in charge of recording while using the
        robot-specific ROS controller for command transport.

        Args:
            time_data: Command timestamps [s].
            position_stream: Joint position commands [rad].
            velocity_stream: Optional joint velocity commands [rad/s].
            acceleration_stream: Optional joint acceleration commands [rad/s^2].
            Ts: Sampling time [s].

        Returns:
            `list[tuple[float, list[float]]]` host publish timestamps [s] and
            joint position commands [rad].

        Raises:
            RuntimeError: If the robot is not connected or ROS publishing fails.

        Preconditions:
            The robot is connected, ROS topics are available, and `self.id` is non-empty.
        """
        if not self.id:
            raise RuntimeError(
                "Robot ID is required before publishing ROS trajectories."
            )

        from robot.ros_manager import JOINT_PUBLISHER, rclpy
        from rclpy.executors import MultiThreadedExecutor
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        ros_initialized_by_method = False
        if not rclpy.ok():
            rclpy.init()
            ros_initialized_by_method = True

        node = rclpy.create_node("standardbots_joint_trajectory_publisher")
        topic = JOINT_PUBLISHER.replace("<BOT_ID>", self.id)
        joint_publisher = node.create_publisher(
            JointTrajectory,
            topic,
            10,
        )
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(
            target=executor.spin,
            name="standardbots_joint_trajectory_publisher_spin",
            daemon=True,
        )
        spin_thread.start()

        try:
            discovery_deadline_s = time.time() + ROS_SERVO_DISCOVERY_TIMEOUT_S
            while (
                joint_publisher.get_subscription_count() == 0
                and time.time() < discovery_deadline_s
            ):
                time.sleep(ROS_SERVO_DISCOVERY_POLL_S)

            time_values = list(time_data)
            position_values = [list(row) for row in position_stream]
            velocity_values = (
                [list(row) for row in velocity_stream]
                if velocity_stream is not None
                else []
            )
            acceleration_values = (
                [list(row) for row in acceleration_stream]
                if acceleration_stream is not None
                else []
            )
            if len(time_values) != len(position_values):
                raise ValueError(
                    "time_data and position_stream must have the same length"
                )
            if not time_values:
                raise ValueError("time_data and position_stream must be non-empty")

            total_cmds = len(time_values)
            progress_every = max(1, total_cmds // 10)
            print(
                f"[StandardBots] Publishing {total_cmds} ROS trajectory commands at {1.0 / Ts:.1f} Hz."
            )
            tref0 = float(time_values[0])
            t0 = time.perf_counter()
            command_records: list[tuple[float, list[float]]] = []

            for cmd_idx, (tref, positions) in enumerate(
                zip(time_values, position_values),
                start=1,
            ):
                target_time = t0 + (float(tref) - tref0)
                while True:
                    now = time.perf_counter()
                    delta = target_time - now
                    if delta <= 0.0:
                        break
                    time.sleep(min(delta, Ts / 2.0))

                ros_now = node.get_clock().now()
                point = JointTrajectoryPoint()
                point.positions = positions
                if velocity_values:
                    point.velocities = velocity_values[cmd_idx - 1]
                if acceleration_values:
                    point.accelerations = acceleration_values[cmd_idx - 1]
                point.time_from_start.sec = int(ros_now.nanoseconds * 1e-9)
                point.time_from_start.nanosec = int(ros_now.nanoseconds % 1e9)

                message = JointTrajectory()
                message.joint_names = [f"joint{i}" for i in range(len(positions))]
                message.points.append(point)
                joint_publisher.publish(message)

                cmd_time = time.time()
                command_records.append((cmd_time, positions))
                if (
                    cmd_idx == 1
                    or cmd_idx == total_cmds
                    or (cmd_idx % progress_every) == 0
                ):
                    progress_pct = 100.0 * cmd_idx / total_cmds
                    print(
                        f"[StandardBots] Command progress: {cmd_idx}/{total_cmds} ({progress_pct:.1f}%)"
                    )
            return command_records
        finally:
            executor.shutdown()
            spin_thread.join(timeout=1.0)
            node.destroy_node()
            if ros_initialized_by_method and rclpy.ok():
                rclpy.shutdown()

    def enter_position_mode(self) -> Optional[int | None]:
        """
        Ensure the controller is in point-to-point position mode before issuing queued P2P moves.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()
        models = self._models()
        self._teardown_ros_servo_publisher()
        with arm.connection():
            arm.ros.control.update_ros_control_state(
                models.ROSControlUpdateRequest(
                    action=models.ROSControlStateEnum.Disabled,
                )
            ).ok()
            self.state = arm.ros.status.get_ros_control_state().ok()
        return 0

    def enter_servo_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to servo control mode.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()
        models = self._models()
        with arm.connection():
            arm.ros.control.update_ros_control_state(
                models.ROSControlUpdateRequest(
                    action=models.ROSControlStateEnum.Enabled,
                )
            ).ok()
            self.state = arm.ros.status.get_ros_control_state().ok()
        return 0

    def get_joint_state(self) -> tuple[list[float], list[float], list[float]]:
        """Return one joint state sample as ``(q, qd, tau)``.

        Returns:
            Tuple of three lists: joint positions `q` [rad], velocities `qd` [rad/s],
            and efforts/currents `tau` [SDK units].
        """
        arm = self._require_connected_arm()
        position = arm.movement.position.get_arm_position().ok()
        joint_rotations = _require(
            position.joint_rotations, "Robot returned no joint rotations."
        )
        q = _joint_values_in_radians(list(joint_rotations))
        if len(q) > self.num_joints:
            q = q[: self.num_joints]
        elif len(q) < self.num_joints:
            raise ValueError(
                f"Received {len(q)} joint angles but expected {self.num_joints} "
                "from the URDF model."
            )
        qd = [0.0] * self.num_joints
        tau = [0.0] * self.num_joints
        return q, qd, tau

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        arm = self._require_connected_arm()
        position = arm.movement.position.get_arm_position().ok()
        tooltip_position = _require(
            position.tooltip_position, "Robot returned no tooltip position."
        )
        cartesian_position = _require(
            tooltip_position.position, "Robot returned no cartesian position."
        )
        orientation = _require(
            tooltip_position.orientation, "Robot returned no tooltip orientation."
        )
        quaternion = _require(
            orientation.quaternion, "Robot returned no tooltip quaternion."
        )

        return [
            float(cartesian_position.x),
            float(cartesian_position.y),
            float(cartesian_position.z),
            float(quaternion.x),
            float(quaternion.y),
            float(quaternion.z),
            float(quaternion.w),
        ]

    def _ensure_ros_servo_publisher(self) -> None:
        """Create the ROS publisher used by `command_servo_j`.

        Args:
            None.

        Returns:
            `None`.

        Side Effects:
            Initializes ROS, creates a node, and creates a joint trajectory publisher.

        Raises:
            RuntimeError: If the robot ID is not available.

        Preconditions:
            The Standard Bots controller has ROS control enabled.
        """
        if self._ros_joint_publisher is not None:
            return
        if not self.id:
            raise RuntimeError(
                "Robot ID is required before creating ROS servo publisher."
            )

        from robot.ros_manager import JOINT_PUBLISHER, rclpy
        from rclpy.executors import SingleThreadedExecutor
        from trajectory_msgs.msg import JointTrajectory

        if not rclpy.ok():
            rclpy.init()
            self._ros_initialized_by_interface = True
        self._ros_servo_node = rclpy.create_node("standardbots_servo_publisher")
        self._ros_servo_topic = JOINT_PUBLISHER.replace("<BOT_ID>", self.id)
        self._ros_joint_publisher = self._ros_servo_node.create_publisher(
            JointTrajectory,
            self._ros_servo_topic,
            10,
        )
        self._ros_servo_executor = SingleThreadedExecutor()
        self._ros_servo_executor.add_node(self._ros_servo_node)
        self._ros_servo_spin_thread = threading.Thread(
            target=self._spin_ros_servo_executor,
            name="standardbots_servo_publisher_spin",
            daemon=True,
        )
        self._ros_servo_spin_thread.start()
        discovery_deadline_s = time.time() + ROS_SERVO_DISCOVERY_TIMEOUT_S

        # Give DDS discovery a short window before the high-rate command loop.
        while (
            self._ros_joint_publisher.get_subscription_count() == 0
            and time.time() < discovery_deadline_s
        ):
            time.sleep(ROS_SERVO_DISCOVERY_POLL_S)

    def _spin_ros_servo_executor(self) -> None:
        """Spin the servo publisher executor until ROS shutdown.

        Returns:
            `None`.

        Raises:
            None.
        """
        if self._ros_servo_executor is None:
            return
        try:
            self._ros_servo_executor.spin()
        except Exception as exc:
            if exc.__class__.__name__ != "ExternalShutdownException":
                raise

    def _teardown_ros_servo_publisher(self) -> None:
        """Destroy the ROS servo publisher created by this adapter.

        Args:
            None.

        Returns:
            `None`.

        Side Effects:
            Destroys the ROS publisher node and shuts down ROS if this adapter
            initialized it.

        Raises:
            None.

        Preconditions:
            None.
        """
        if self._ros_servo_node is None and not self._ros_initialized_by_interface:
            return

        from robot.ros_manager import rclpy

        if self._ros_servo_executor is not None:
            self._ros_servo_executor.shutdown()
            self._ros_servo_executor = None
        if self._ros_servo_spin_thread is not None:
            self._ros_servo_spin_thread.join(timeout=1.0)
            self._ros_servo_spin_thread = None
        if self._ros_servo_node is not None:
            self._ros_servo_node.destroy_node()
        self._ros_joint_publisher = None
        self._ros_servo_node = None
        self._ros_servo_topic = ""
        self._ros_servo_missing_subscriber_warning_printed = False
        if self._ros_initialized_by_interface and rclpy.ok():
            rclpy.shutdown()
        self._ros_initialized_by_interface = False

    # {~.~} END OF REQUIRED METHODS
