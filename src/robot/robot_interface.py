# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0

from collections.abc import Mapping
from importlib.resources import as_file, files
import os
from pathlib import Path
from typing import Literal, Optional, Sequence

import numpy as np

from reforge_core.hw_interfaces.arm_client import ArmClient
from reforge_core.hw_interfaces.imu_recorder import ImuRecorder

from .grpc_transport import YaskawaGrpcTransport
from .motion import YaskawaMotionClient
from .read_only import YaskawaReadOnlyClient

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


# Phase 1 contract defaults. Live motion remains gated on controller discovery,
# preflight, and an approved trajectory even though the supplied model is present.
BOT_ID = "yaskawa-nex7"
URDF_PATH = "urdf/NEX07C00/NEX07C00.urdf"
ROBOT_MAX_FREQ = 250  # ACU monitor request ceiling [Hz], not a proven stable rate.
DEFAULT_FEEDBACK_RATE_HZ = 25.0
SERVO_CONFIRMATION_TEXT = "I_CONFIRM_SERVOS_ARE_ON"
DEFAULT_QUEUED_MOVE_SPEED_PERCENT = 10.0
DEFAULT_QUEUED_ACCELERATION_PERCENT = 20.0
IK_POSITION_TOLERANCE_M = 1.0e-3
IK_ORIENTATION_TOLERANCE_RAD = np.deg2rad(1.0)
IK_MAX_ITERATIONS = 300
IK_SOLVER_TOLERANCE = 1.0e-6

# Fully stretched position of the robot for calibration.
FULL_STRETCH_XYZ = [0.0, 0.0, 0.0]  # Unverified until the NEX7 pose is approved [m].
FULL_STRETCH_QUAT = [0.0, 0.0, 0.0, 1.0]  # Identity diagnostic placeholder.
FULL_STRETCH_JOINTS = [
    0.0,
    1.5707963267948966,
    1.5707963267948966,
    0.0,
    0.0,
    0.0,
]  # [rad].
FULL_STRETCH_POSE_OVERRIDE = None

# General constants
IS_DEGREES = False  # ArmClient-facing values are radians.
DATA_LOCATION_PREFIX = "src/robot/data"  # {~.~} [CHANGE TO LOCATION DESIRED - will be robot/DATA_LOCATION_PREFIX/*]
SIM_DATA_LOCATION_PREFIX = str(Path(__file__).resolve().parent / "data" / "sim")
DEFAULT_TCP_PAYLOAD = 0.0  # {~.~} [CHANGE IF THE DEFAULT PAYLOAD IS NON_ZERO]

MAX_ROBOT_JOINTS_BANDWIDTH = (
    5.0  # {~.~} Servo motor bandwidth. Leave as is if you don't know [Hz]
)

# {~.~} IMU information
USE_REFORGE_IMU = True
DEFAULT_IMU_COMM_MODE: Literal["ble", "usb", "virtual"] = "usb"
DEFAULT_IMU_RECORD_MODE: Literal["streaming", "logging"] = "streaming"
DEFAULT_IMU_RECORD_FREQUENCY_HZ = ROBOT_MAX_FREQ


def _positive_environment_float(name: str, default: float) -> float:
    """Read one finite positive floating-point deployment setting.

    Args:
        name: Environment variable name.
        default: Value used when the variable is absent.

    Returns:
        Configured positive value.

    Raises:
        ValueError: If the configured value is not finite and positive.
    """

    raw_value = os.environ.get(name)
    value = default if raw_value is None else float(raw_value)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")
    return value


def _optional_positive_environment_float(name: str) -> float | None:
    """Read an optional finite positive floating-point deployment setting.

    Args:
        name: Environment variable name.

    Returns:
        Configured positive value, or ``None`` when the variable is absent.

    Raises:
        ValueError: If the configured value is not finite and positive.
    """

    raw_value = os.environ.get(name)
    if raw_value is None:
        return None
    return _positive_environment_float(name, 1.0)


def _operator_servo_confirmation() -> bool:
    """Return whether the operator set the exact live-motion acknowledgement."""

    return os.environ.get("YASKAWA_SERVO_POWER_CONFIRMED") == SERVO_CONFIRMATION_TEXT


class RobotInterface(ArmClient):
    """Provide a concrete robot implementation for system identification and calibration.

    Args:
        robot_ip: Live robot internet protocol address.
        tcp_payload: Optional payload of the robot for NN prediction of
            payload changes.
        tcp_payload_com: Optional 3x1 center of mass location of the
            payload, defined relative to the origin of the TCP [meters].
        local_ip: Local internet protocol address for networked setups.
        sdk_token: Authentication token for the robot software development kit.
        robot_id: Identifier for the robot in the control stack.

    Side Effects:
        Loads the robot model from the configured Unified Robot Description Format file.
        Connects to the robot hardware.

    Raises:
        ValueError: If the simulator sentinel is passed to the hardware adapter.
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
        imu_record_mode: Literal["streaming", "logging"] = DEFAULT_IMU_RECORD_MODE,
        imu_comm_mode: Literal["ble", "usb", "virtual"] = DEFAULT_IMU_COMM_MODE,
        imu_record_frequency_hz: float | int = DEFAULT_IMU_RECORD_FREQUENCY_HZ,
        imu_recorder: ImuRecorder | None = None,
        tcp_payload: float = DEFAULT_TCP_PAYLOAD,
        tcp_payload_com: Sequence[float] | None = None,
        read_only_client: YaskawaReadOnlyClient | None = None,
        motion_client: YaskawaMotionClient | None = None,
    ) -> None:
        """Initialize the robot interface and load the URDF model.

        Args:
            robot_ip: Live robot IP address.
            local_ip: Local machine IP address if required by the SDK.
            sdk_token: SDK authentication token.
            api_token: Reforge API token.
            robot_id: Reforge robot ID (most cases) or SDK identifier used by the control stack.
            use_reforge_imu: Whether to use the built-in Reforge IMU backend
                when `imu_recorder` is not supplied.
            imu_record_mode: Reforge IMU acquisition backend used when
                `imu_recorder` is not supplied.
            imu_comm_mode: Reforge IMU communication backend used when
                `imu_recorder` is not supplied.
            imu_record_frequency_hz: Reforge IMU recording frequency [Hz] used
                when `imu_recorder` is not supplied.
            imu_recorder: Optional vendor-specific recorder supplied directly
                by an application or integration test.
            tcp_payload: Payload mass attached at the TCP [kg].
            tcp_payload_com: Optional payload center of mass in TCP coordinates [m].
            read_only_client: Configured validated state client. The caller
                owns construction of the local ACU bridge transport.
            motion_client: Optional Phase 4 motion client. Its state client
                must be the same validated read-only client supplied through
                ``read_only_client`` when both are provided.

        Side Effects:
            Loads the URDF model and connects to robot hardware.

        Raises:
            ValueError: If the simulator sentinel is passed to the hardware adapter.
            RuntimeError: If the robot connection fails.
            ValueError: If reported joint counts do not match the URDF.

        Preconditions:
            The URDF file is available and the SDK is installed.
        """
        if robot_ip == "sim":
            raise ValueError(
                "RobotInterface is hardware-only; construct simulator mode "
                "through reforge_core.calibration.run_helpers."
            )

        super().__init__(
            name="My Robot", recording_data_frequency_hz=ROBOT_MAX_FREQ
        )  # {~.~} [Edit with your robot's name and sampling frequency]

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
        self.robot: YaskawaReadOnlyClient | None = None
        selected_read_only_client: YaskawaReadOnlyClient | None = None
        try:
            if read_only_client is None and motion_client is None:
                feedback_rate_hz = _positive_environment_float(
                    "YASKAWA_FEEDBACK_RATE_HZ", DEFAULT_FEEDBACK_RATE_HZ
                )
                command_rate_hz = _positive_environment_float(
                    "YASKAWA_COMMAND_RATE_HZ", feedback_rate_hz
                )
                controller_tick_period_s = _optional_positive_environment_float(
                    "YASKAWA_CONTROLLER_TICK_PERIOD_S"
                )
                transport = YaskawaGrpcTransport(robot_ip)
                read_only_client = YaskawaReadOnlyClient(
                    transport,
                    rate_hz=feedback_rate_hz,
                    controller_tick_period_s=controller_tick_period_s,
                )
                motion_client = YaskawaMotionClient(
                    read_only_client,
                    transport,
                    cycle_period_s=1.0 / command_rate_hz,
                    servo_power_confirmation=_operator_servo_confirmation,
                )
            selected_read_only_client = (
                motion_client.state_client
                if motion_client is not None
                else read_only_client
            )
            if selected_read_only_client is None:
                raise RuntimeError(
                    "Yaskawa requires an injected read-only ACU bridge "
                    "client; no default controller connection is enabled."
                )
            if (
                read_only_client is not None
                and motion_client is not None
                and (motion_client.state_client is not read_only_client)
            ):
                raise ValueError(
                    "motion_client.state_client must be the supplied read_only_client."
                )
            selected_read_only_client.connect()
            selected_read_only_client.start()
            selected_read_only_client.latest(timeout_s=2.0)
            self.robot = selected_read_only_client
            self._motion_client = motion_client

            # ------------------- EXAMPLE --------------------
            # self.robot = StandardBotsRobot(
            #     url=robot_ip,
            #     token=sdk_token,
            #     robot_kind=StandardBotsRobot.RobotKind.Live,
            # )
            # ------------------------------------------------

            # {~.~} Enable ROS control, if necessary
            # [YOUR CODE HERE]

            # ------------------------------ EXAMPLE -------------------------------
            # with self.robot.connection():
            #     ## Set teleoperation/ROS control state
            #     self.robot.ros.control.update_ros_control_state(
            #         models.ROSControlUpdateRequest(
            #             action=models.ROSControlStateEnum.Enabled,
            #             # to disable: action=models.ROSControlStateEnum.Disabled,
            #         )
            #     )

            #     # Get teleoperation state
            #     self.state = self.robot.ros.status.get_ros_control_state().ok()
            #     # Enable the robot, make sure the E-stop is released before enabling
            #     print("Enabling live robot...")
            # -----------------------------------------------------------------------

            # {~.~} Unbrake the robot if not operational
            # [YOUR CODE HERE]

            # --------------- EXAMPLE -----------------
            # self.robot.movement.brakes.unbrake().ok()
            # -----------------------------------------

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

        except Exception as e:
            if selected_read_only_client is not None:
                selected_read_only_client.close()
            # Print exception error message
            raise RuntimeError(f"Error getting {robot_ip} operational: {str(e)}")

        if not self.imu_manager_is_loaded:
            selected_imu_recorder = imu_recorder
            if selected_imu_recorder is None and not self.use_reforge_imu:
                selected_imu_recorder = self.create_robot_imu_recorder()
            self.use_reforge_imu = selected_imu_recorder is None
            self.arm_imu_manager = self.initialize_arm_imu_manager(
                arm_sample_time_s=1.0 / ROBOT_MAX_FREQ,
                imu_record_mode=imu_record_mode,
                imu_comm_mode=imu_comm_mode,
                imu_record_frequency_hz=imu_record_frequency_hz,
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
            `bool` always false for the hardware adapter.
        """
        return False

    @property
    def urdf_path(self) -> str:
        """Return the absolute path to the URDF file.

        Returns:
            `str` path to the URDF file.
        """
        return self._urdf_path

    # {~.~} REQUIRED METHODS
    def command_move_j(
        self,
        target_joints: np.ndarray | list[float] | tuple[float, ...],
        *,
        speed: float = DEFAULT_QUEUED_MOVE_SPEED_PERCENT,
        wait: bool = True,
    ) -> int:
        """Queue point-to-point joint motion through an explicit motion client.

        Args:
            target_joints: Target joint positions [rad] as a list or array.
            speed: Controller-rated joint speed percentage. Defaults to the
                conservative initial Yaskawa test limit of 10%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.

        Raises:
            NotImplementedError: If no explicit motion client was injected.
        """
        motion_client = getattr(self, "_motion_client", None)
        if isinstance(motion_client, YaskawaMotionClient):
            # Queued and incremental motion may not overlap. This is a no-op
            # when no incremental task exists and fail-closed when cleanup fails.
            motion_client.stop_servo_session()
            return motion_client.move_j(
                list(target_joints),
                speed_percent=speed,
                acceleration_percent=DEFAULT_QUEUED_ACCELERATION_PERCENT,
                deceleration_percent=DEFAULT_QUEUED_ACCELERATION_PERCENT,
                wait=wait,
            )
        raise NotImplementedError(
            "Yaskawa motion is disabled unless an explicit Phase 4 motion client "
            "is injected."
        )

    def command_move_pose(
        self,
        target_quat: np.ndarray | list[float],
        target_xyz: np.ndarray | list[float],
        *,
        speed: float = DEFAULT_QUEUED_MOVE_SPEED_PERCENT,
        wait: bool = True,
        locked_joints: Mapping[int, float] | None = None,
    ) -> int:
        """Solve a Cartesian target with URDF IK and queue the joint solution.

        Args:
            target_quat: Target TCP orientation as a quaternion `[qx, qy, qz, qw]` [-] in the robot's base frame.
            target_xyz: Target TCP position `[x, y, z]` [m] in the robot's base frame.
            speed: Controller-rated joint speed percentage. Defaults to the
                conservative initial Yaskawa test limit of 10%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.
            locked_joints: Simulator-only joint-index to fixed position map [rad].

        Returns:
            Zero after the validated joint target completes or is accepted.

        Raises:
            RuntimeError: If IK is unavailable or does not satisfy the reviewed
                position and orientation tolerances.
            ValueError: If a target is malformed or simulator-only joint locks
                are requested on hardware.
        """
        if locked_joints is not None:
            raise ValueError("locked_joints is only supported in simulator mode.")
        motion_client = getattr(self, "_motion_client", None)
        if not isinstance(motion_client, YaskawaMotionClient):
            raise NotImplementedError(
                "Yaskawa motion is disabled unless an explicit Phase 4 motion "
                "client is injected."
            )
        model = getattr(self, "model", None)
        if model is None:
            raise RuntimeError(
                "Yaskawa Cartesian motion requires the loaded URDF model."
            )

        xyz = np.asarray(target_xyz, dtype=float)
        quat = np.asarray(target_quat, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError("target_xyz must be a finite three-vector in meters.")
        if quat.shape != (4,) or not np.all(np.isfinite(quat)):
            raise ValueError("target_quat must be a finite quaternion [x, y, z, w].")
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm <= np.finfo(float).eps:
            raise ValueError("target_quat must have non-zero magnitude.")

        current_joints = np.asarray(
            motion_client.state_client.latest().joint_positions_rad,
            dtype=float,
        )
        target_pose = np.concatenate((xyz, quat / quat_norm))
        solved_joints, report = model.get_inverse_kinematics(
            target_pose=target_pose,
            initial_angles=current_joints,
            max_iters=IK_MAX_ITERATIONS,
            tol=IK_SOLVER_TOLERANCE,
            get_report=True,
        )
        if (
            not report.converged
            or report.position_error_mag > IK_POSITION_TOLERANCE_M
            or report.rotation_error_mag > IK_ORIENTATION_TOLERANCE_RAD
        ):
            raise RuntimeError(
                "Yaskawa Cartesian target has no validated URDF IK solution: "
                f"converged={report.converged}, "
                f"position_error_m={report.position_error_mag:.6g}, "
                f"orientation_error_rad={report.rotation_error_mag:.6g}."
            )
        return self.command_move_j(
            np.asarray(solved_joints, dtype=float),
            speed=speed,
            wait=wait,
        )

    def command_servo_j(
        self,
        target_joints: np.ndarray | list[float],
        *,
        wait: bool = False,
    ) -> int:
        """Publish one incremental servo target through an explicit motion client.

        Args:
            target_joints: Target joint position [rad] as a list or array.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.

        Raises:
            NotImplementedError: If no explicit motion client was injected.
        """
        motion_client = getattr(self, "_motion_client", None)
        if isinstance(motion_client, YaskawaMotionClient):
            return motion_client.command_servo_j(list(target_joints), wait=wait)
        raise NotImplementedError(
            "Yaskawa motion is disabled unless an explicit Phase 4 motion client "
            "is injected."
        )

    def enter_position_mode(self) -> Optional[int | None]:
        """Stop incremental motion before queued point-to-point commands.

        Returns:
            Zero after no incremental task remains active.

        Raises:
            NotImplementedError: If no explicit motion client was injected.
        """
        motion_client = getattr(self, "_motion_client", None)
        if isinstance(motion_client, YaskawaMotionClient):
            motion_client.stop_servo_session()
            return 0
        raise NotImplementedError(
            "Yaskawa motion is disabled unless an explicit Phase 4 motion client "
            "is injected."
        )

    def enter_servo_mode(self) -> Optional[int | None]:
        """Start an incremental session after an operator enables the servos.

        Raises:
            NotImplementedError: If no explicit motion client was injected.
        """
        motion_client = getattr(self, "_motion_client", None)
        if isinstance(motion_client, YaskawaMotionClient):
            motion_client.start_servo_session()
            return 0
        raise NotImplementedError(
            "Yaskawa motion is disabled unless an explicit Phase 4 motion client "
            "is injected."
        )

    def supports_teaching_mode(self) -> bool:
        """Return whether the robot supports manual teaching mode.

        Override this method when the robot SDK supports hand-guided teaching.

        Returns:
            `bool` indicating whether manual teaching mode is implemented.
        """
        return False

    def enter_teaching_mode(self) -> Optional[int | None]:
        """Reject Teach-mode changes unsupported by the ACU SDK.

        Raises:
            NotImplementedError: Always; only mode reads are documented.
        """
        raise NotImplementedError(
            "The Yaskawa ACU SDK reports Teach/Play state but cannot change it."
        )

    def supports_flange_button(self) -> bool:
        """Return whether the robot exposes a readable flange button.

        Override this method when the robot SDK exposes a button or equivalent
        operator input near the tool flange.

        Returns:
            `bool` indicating whether flange-button reads are implemented.
        """
        return False

    def read_flange_button_pressed(self) -> bool:
        """Reject flange-button reads because NEX7 has no exposed button.

        Raises:
            NotImplementedError: Always; no flange input is available.
        """
        raise NotImplementedError("Yaskawa NEX7 exposes no readable flange button.")

    def get_joint_state(self) -> tuple[list[float], list[float], list[float]]:
        """Return one joint state sample as ``(q, qd, tau)``.

        Returns:
            Tuple of three lists: joint positions `q` [rad], velocities `qd` [rad/s],
            and efforts/currents `tau` [SDK units].
        """
        client = self._require_connected_arm()
        if not isinstance(client, YaskawaReadOnlyClient):
            raise RuntimeError("Connected Yaskawa client has an invalid type.")
        state = client.latest()
        if state.joint_velocities_rad_s is None:
            raise RuntimeError(
                "Yaskawa joint velocity is unavailable until the controller "
                "timestamp tick period is measured and configured."
            )
        return (
            list(state.joint_positions_rad),
            list(state.joint_velocities_rad_s),
            list(state.joint_torques_nm),
        )

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        client = self._require_connected_arm()
        if not isinstance(client, YaskawaReadOnlyClient):
            raise RuntimeError("Connected Yaskawa client has an invalid type.")
        return list(client.latest().tcp_pose)

    def close(self) -> None:
        """Close read-only acquisition and the local bridge transport."""

        cleanup_error: BaseException | None = None
        motion_client = getattr(self, "_motion_client", None)
        if isinstance(motion_client, YaskawaMotionClient):
            try:
                motion_client.close()
            except BaseException as exc:  # noqa: BLE001
                cleanup_error = exc
        client = getattr(self, "robot", None)
        try:
            if isinstance(client, YaskawaReadOnlyClient):
                client.close()
        finally:
            self.robot = None
            self._motion_client = None
        if cleanup_error is not None:
            raise cleanup_error

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
        motion_client = getattr(self, "_motion_client", None)
        try:
            return super().command_joint_trajectory(
                time_data=time_data,
                position_stream=position_stream,
                velocity_stream=velocity_stream,
                acceleration_stream=acceleration_stream,
                Ts=Ts,
            )
        finally:
            if isinstance(motion_client, YaskawaMotionClient):
                motion_client.stop_servo_session()

    # {~.~} END OF OPTIONAL OVERRIDES
