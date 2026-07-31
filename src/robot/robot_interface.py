# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0

import numpy as np
from importlib.resources import files, as_file
from typing import Optional, Sequence
from robot.cobotta_bcap import BcapTransportFactory, CobottaBcapRobot
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
BOT_ID = "denso-cobotta-pro-900"
URDF_PATH = "urdf/cobotta_pro_900.urdf"
ROBOT_MAX_FREQ = 125  # CurJntEx is refreshed every 8 ms [Hz]

# Fully stretched position of the robot for calibration.
FULL_STRETCH_XYZ = [1.28989, 0.36866, 0.171]  # {~.~} [m]
FULL_STRETCH_QUAT = [0.499, 0.499, 0.499, 0.499]  # {~.~} [1]
FULL_STRETCH_JOINTS = [0.0, np.pi / 2, 0.0, 0.0, 0.0, 0.0]  # {~.~} [rad]
FULL_STRETCH_POSE_OVERRIDE = None  # {~.~} list of home pose (xyz and quaternion) to override additional height not in base height

# General constants
IS_DEGREES = False
DATA_LOCATION_PREFIX = "src/robot/data"  # {~.~} [CHANGE TO LOCATION DESIRED - will be robot/DATA_LOCATION_PREFIX/*]
DEFAULT_TCP_PAYLOAD = 0.0  # {~.~} [CHANGE IF THE DEFAULT PAYLOAD IS NON_ZERO]

MAX_ROBOT_JOINTS_BANDWIDTH = (
    5.0  # {~.~} Servo motor bandwidth. Leave as is if you don't know [Hz]
)

# {~.~} IMU information
USE_REFORGE_IMU = True
DEFAULT_IMU_COMM_MODE = "usb"
DEFAULT_IMU_RECORD_FREQUENCY_HZ = ROBOT_MAX_FREQ


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
        cobotta_transport_factory: BcapTransportFactory | None = None,
    ) -> None:
        """Initialize the robot interface and load the URDF model.

        Args:
            robot_ip: Robot IP address or `sim` for simulator mode.
            tcp_payload: Payload of the tcp (default=0)
            tcp_payload_com: Optional 3x1 center of mass of the tcp payload.
            local_ip: Local machine IP address if required by the SDK.
            sdk_token: SDK authentication token.
            api_token: Reforge API token.
            robot_id: Reforge robot ID (most cases) or SDK identifier _reforge_d by the control stack.
            use_reforge_imu: Whether to use the built-in Reforge IMU backend
                when `imu_recorder` is not supplied.
            imu_recorder: Optional vendor-specific recorder supplied directly
                by an application or integration test.
            tcp_payload: payload of end-effector [kg].
            tcp_payload_com: (N x 3) center of mass of end-effector payload [m].
            cobotta_transport_factory: Optional b-CAP transport factory for
                integration tests. Production uses the pinned TCP client.

        Side Effects:
            Loads the URDF model and may connect to robot hardware.

        Raises:
            RuntimeError: If the robot connection fails.
            ValueError: If reported joint counts do not match the URDF.

        Preconditions:
            The URDF file is available and the SDK is installed.
        """
        super().__init__(
            name="DENSO COBOTTA PRO 900",
            recording_data_frequency_hz=ROBOT_MAX_FREQ,
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

        # Reforge API and robot ID token is needed for "joint_tracker" product
        # Add it in the CLI with `--identify`
        self.reforge_api_token = api_token
        try:
            if robot_ip != "sim":
                if cobotta_transport_factory is None:
                    self.robot = CobottaBcapRobot(
                        robot_ip,
                        joint_count=self.num_joints,
                    )
                else:
                    self.robot = CobottaBcapRobot(
                        robot_ip,
                        joint_count=self.num_joints,
                        transport_factory=cobotta_transport_factory,
                    )
                self.robot.connect()

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

        except Exception as exc:
            if isinstance(self.robot, CobottaBcapRobot):
                try:
                    self.robot.disconnect()
                except BaseException:
                    pass
            raise RuntimeError(f"Error getting {robot_ip} operational: {exc}") from exc

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

        Robot integrations should override this method and return an
        `ImuRecorder` that converts SDK samples into `IMUState` values in SI
        units. The recorder must emit Unix epoch timestamps aligned with arm
        state timestamps, either because both originate from one controller
        clock or because `prepare()` estimates and applies their offset.

        Returns:
            `ImuRecorder` backed by the robot vendor's native IMU API.

        Raises:
            NotImplementedError: If this robot template has not implemented a
                native IMU adapter.
        """
        raise NotImplementedError(
            "use_reforge_imu=False requires RobotInterface."
            "create_robot_imu_recorder() to return a vendor-specific "
            "ImuRecorder."
        )

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
        self._cobotta_robot().move_j(
            self._joint_command(
                (
                    target_joints.tolist()
                    if isinstance(target_joints, np.ndarray)
                    else target_joints
                ),
                "target_joints",
            ),
            speed_percent=self._speed_percent(speed),
            wait=self._wait_flag(wait),
        )
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
        self._cobotta_robot().move_pose(
            self._fixed_length_values(
                (
                    target_quat.tolist()
                    if isinstance(target_quat, np.ndarray)
                    else target_quat
                ),
                4,
                "target_quat",
            ),
            self._fixed_length_values(
                (
                    target_xyz.tolist()
                    if isinstance(target_xyz, np.ndarray)
                    else target_xyz
                ),
                3,
                "target_xyz",
            ),
            speed_percent=self._speed_percent(speed),
            wait=self._wait_flag(wait),
        )
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
            An integer status code from the robot's command interface, if applicable.
            If the robot does not provide a status code, return 0 for success or raise an exception for failure.
        """
        if wait:
            raise ValueError("COBOTTA Slave Mode does not support a wait flag")
        self._cobotta_robot().command_servo_j(
            self._joint_command(
                (
                    target_joints.tolist()
                    if isinstance(target_joints, np.ndarray)
                    else target_joints
                ),
                "target_joints",
            )
        )
        return 0

    def enter_position_mode(self) -> Optional[int | None]:
        """
        Ensure the controller is in point-to-point position mode before issuing queued P2P moves.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        self._cobotta_robot().enter_position_mode()
        return 0

    def enter_servo_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to servo control mode.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        self._cobotta_robot().enter_servo_mode()
        return 0

    def supports_teaching_mode(self) -> bool:
        """Return whether b-CAP exposes CRC9 Direct Teaching control.

        Returns:
            `bool` indicating whether manual teaching mode is implemented.
        """
        return False

    def enter_teaching_mode(self) -> Optional[int | None]:
        """Leave Direct Teaching unchanged becaforge b-CAP does not control it.

        CRC9 Direct Teaching is an operator-facing feature. The RC9 b-CAP and
        Provider guides do not document a command to enter it remotely.

        Returns:
            `None`, because teaching mode is unsupported.
        """
        return None

    def supports_flange_nn(self) -> bool:
        """Return whether the robot exposes a readable flange button.

        Override this method when the robot SDK exposes a button or equivalent
        operator input near the tool flange.

        Returns:
            `bool` indicating whether flange-button reads are implemented.
        """
        return False

    def read_flange_button_pressed(self) -> bool:
        """Return whether the flange button is currently pressed.

        Override this method with the robot SDK's flange-button read.

        Returns:
            `bool` indicating the current flange-button state.
        """
        arm = self._require_connected_arm()  # noqa: F841

        # {~.~} Read the flange-button state using the robot SDK.
        # [YOUR CODE HERE]

        return False

    def get_joint_state(self) -> tuple[list[float], list[float], list[float]]:
        """Return one joint state sample as ``(q, qd, tau)``.

        Returns:
            Tuple of three lists: joint positions `q` [rad], velocities `qd` [rad/s],
            and efforts/currents `tau` [SDK units].
        """
        sample = self._cobotta_robot().get_joint_sample()
        return sample.positions_rad, sample.velocities_rad_s, sample.efforts

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        return self._cobotta_robot().get_current_pose()

    def disconnect(self) -> None:
        """Disconnect the live COBOTTA b-CAP session when one is active.

        This does not stop an IMU recorder; callers remain responsible for
        ending any active recording before releasing robot control.
        """
        if isinstance(self.robot, CobottaBcapRobot):
            self.robot.disconnect()

    def _cobotta_robot(self) -> CobottaBcapRobot:
        """Return the connected COBOTTA wrapper.

        Returns:
            Active COBOTTA b-CAP wrapper.

        Raises:
            RuntimeError: If the interface is simulated or misconfigured.
        """
        robot = self._require_connected_arm()
        if not isinstance(robot, CobottaBcapRobot):
            raise RuntimeError("Connected arm is not a COBOTTA b-CAP robot")
        return robot

    def _joint_command(self, values: Sequence[float], name: str) -> list[float]:
        """Validate one complete arm joint command in radians.

        Args:
            values: Joint positions [rad].
            name: Input name for error messages.

        Returns:
            Validated joint positions [rad].
        """
        return self._fixed_length_values(values, self.num_joints, name)

    @staticmethod
    def _fixed_length_values(
        values: Sequence[float], expected_length: int, name: str
    ) -> list[float]:
        """Validate a fixed-length sequence of finite numeric values.

        Args:
            values: Values to validate.
            expected_length: Required number of values.
            name: Input name for error messages.

        Returns:
            Validated float values.

        Raises:
            ValueError: If the values are malformed or non-finite.
        """
        if isinstance(values, (str, bytes)) or len(values) != expected_length:
            raise ValueError(f"{name} must contain exactly {expected_length} values")
        try:
            result = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must contain finite numeric values") from exc
        if not all(np.isfinite(result)):
            raise ValueError(f"{name} must contain finite numeric values")
        return result

    @staticmethod
    def _speed_percent(speed: float) -> float:
        """Validate the controller external speed percentage.

        Args:
            speed: Controller speed [%].

        Returns:
            Validated speed percentage.
        """
        return RobotInterface._fixed_length_values([speed], 1, "speed")[0]

    @staticmethod
    def _wait_flag(wait: bool) -> bool:
        """Validate a point-motion wait flag.

        Args:
            wait: Candidate motion wait flag.

        Returns:
            Validated wait flag.

        Raises:
            ValueError: If the value is not a boolean.
        """
        if not isinstance(wait, bool):
            raise ValueError("wait must be a boolean")
        return wait

    # {~.~} END OF REQUIRED METHODS

    # {~.~} OPTIONAL OVERRIDES
    def command_joint_trajectory(
        self,
        time_data: Sequence[float],
        position_stream: Sequence[Sequence[float]],
        velocity_stream: Sequence[Sequence[float]] | None = None,
        acceleration_stream: Sequence[Sequence[float]] | None = None,
        Ts: float = 1.0 / ROBOT_MAX_FREQ,
    ) -> list[tuple[float, list[float]]]:
        """Send one complete joint trajectory and return command timestamps.

        *OVERRIDE* this method when the robot SDK supports a native trajectory
        upload/stream API, requires velocity or acceleration feedforward, or
        needs controller-specific readiness checks before publishing a full
        trajectory. The default implementation delegates to `ArmClient`, which
        streams each sample through `command_servo_j()` at the requested timing.

        Args:
            time_data: Command timestamps [s].
            position_stream: Joint position commands [rad].
            velocity_stream: Optional joint velocity commands [rad/s].
            acceleration_stream: Optional joint acceleration commands [rad/s^2].
            Ts: Sampling time [s].

        Returns:
            `list[tuple[float, list[float]]]` host publish timestamps [s] and
            joint position commands [rad].
        """
        # {~.~} OPTIONAL: Only override this method if the robot SDK has a
        # native trajectory upload/stream API or requires special handling for
        # velocity/acceleration feedforward. Otherwise, the default
        # implementation in `ArmClient` will stream each sample using
        # `command_servo_j()` at the specified timing.

        min_command_period_s = 1.0 / ROBOT_MAX_FREQ
        if not np.isfinite(Ts) or Ts < min_command_period_s:
            raise ValueError(
                f"Ts must be finite and at least {min_command_period_s:.3f} seconds"
            )
        if len(time_data) != len(position_stream) or not time_data:
            raise ValueError(
                "time_data and position_stream must be non-empty and equal"
            )
        previous_time_s: float | None = None
        for time_s, positions_rad in zip(time_data, position_stream, strict=True):
            validated_time_s = self._fixed_length_values([time_s], 1, "time_data")[0]
            if (
                previous_time_s is not None
                and validated_time_s - previous_time_s < min_command_period_s
            ):
                raise ValueError("time_data samples must be at least 8 ms apart")
            previous_time_s = validated_time_s
            self._joint_command(positions_rad, "position_stream sample")

        for stream, name in (
            (velocity_stream, "velocity_stream"),
            (acceleration_stream, "acceleration_stream"),
        ):
            if stream is not None:
                if len(stream) != len(position_stream):
                    raise ValueError(f"{name} must match position_stream length")
                for sample in stream:
                    self._joint_command(sample, f"{name} sample")

        return super().command_joint_trajectory(
            time_data=time_data,
            position_stream=position_stream,
            velocity_stream=velocity_stream,
            acceleration_stream=acceleration_stream,
            Ts=Ts,
        )

    # {~.~} END OF OPTIONAL OVERRIDES
