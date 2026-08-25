# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0

from collections.abc import Mapping
from dataclasses import dataclass
from importlib.resources import as_file, files
from pathlib import Path
from typing import Literal, Optional, Sequence

import numpy as np
from reforge_core.hw_interfaces.arm_client import ArmClient
from reforge_core.hw_interfaces.imu_recorder import ImuRecorder
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from scipy.spatial.transform import Rotation

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
# 3. This integration targets the Universal Robots UR5e via `ur_rtde`
#    (`rtde_control.RTDEControlInterface` / `rtde_receive.RTDEReceiveInterface`).
#    `ur_rtde` is a third-party binding, not an official UR SDK; see
#    https://sdurobotics.gitlab.io/ur_rtde/ and the UR RTDE protocol guide.
# 4. If you opt to use ROS for publishing joint positions, you can use the ros_manager.py file
# in the robots folder. See detailed instructions in that file.


# User constants - EDITS REQUIRED
BOT_ID = ""  # can also enter as CLI argument (see run.py --help)
URDF_PATH = "urdf/ur5e.urdf"
# RTDE's own interface is capped at 125 Hz regardless of the controller's internal
# 500 Hz servo loop (Universal Robots RTDE guide: "frequency must be between 1 and
# 125 Hz"), so 125 Hz is this adapter's real achievable command/telemetry rate.
ROBOT_MAX_FREQ = 125  # [Hz]

# Fully stretched position of the robot for calibration: shoulder bent -90 deg
# (arm pointing straight up, elbow/wrists at 0), all other joints at 0. XYZ/quat were
# computed by forward-kinematics against src/robot/urdf/ur5e.urdf via
# reforge_core.util.robot_dynamics.Dynamics (Pinocchio), not hand-derived, so they
# stay self-consistent with FULL_STRETCH_JOINTS below.
FULL_STRETCH_XYZ = [0.0997, 0.2329, 0.9797]  # [m]
FULL_STRETCH_QUAT = [-0.5, 0.5, 0.5, 0.5]  # [qx, qy, qz, qw]
FULL_STRETCH_JOINTS = [0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0]  # [rad]
FULL_STRETCH_POSE_OVERRIDE = None  # list of home pose (xyz and quaternion) to override additional height not in base height

# General constants
# UR RTDE reports joint/TCP telemetry in SI units (radians, m) natively -- no
# degrees<->radians conversion is needed for this robot.
IS_DEGREES = False
DATA_LOCATION_PREFIX = "src/robot/data"  # {~.~} [CHANGE TO LOCATION DESIRED - will be robot/DATA_LOCATION_PREFIX/*]
SIM_DATA_LOCATION_PREFIX = str(Path(__file__).resolve().parent / "data" / "sim")
DEFAULT_TCP_PAYLOAD = 0.0  # {~.~} [CHANGE IF THE DEFAULT PAYLOAD IS NON_ZERO]

MAX_ROBOT_JOINTS_BANDWIDTH = (
    5.0  # {~.~} Servo motor bandwidth. Leave as is if you don't know [Hz]
)

# Conservative speed/acceleration ceiling for Reforge's 0-100 percentage-based
# `speed` argument. These equal ur_rtde's own documented default moveJ/moveL
# arguments (https://sdurobotics.gitlab.io/ur_rtde/api/api.html), well under
# the UR5e's hard joint-velocity limit of pi rad/s (180 deg/s, from Universal
# Robots' published joint_limits.yaml for the ur5e). speed=100 maps to this
# ceiling; speed=0 maps to no motion; values are clamped into [0, 100].
MAX_JOINT_SPEED_RAD_S = 1.05  # [rad/s] leading-axis joint speed, moveJ's default
MAX_JOINT_ACCEL_RAD_S2 = 1.4  # [rad/s^2] leading-axis joint acceleration, moveJ's default
MAX_LINEAR_SPEED_M_S = 0.25  # [m/s] TCP speed, moveL's default
MAX_LINEAR_ACCEL_M_S2 = 1.2  # [m/s^2] TCP acceleration, moveL's default

# servoJ's speed/acceleration parameters are documented as unused by the
# current RTDE protocol version, so 0.0 is passed for both -- matching
# ur_rtde's own examples/py/servoj_example.py. lookahead_time and gain are
# also taken directly from that same official example.
SERVO_LOOKAHEAD_TIME_S = 0.1  # [s], range [0.03, 0.2] per the ur_rtde API reference
SERVO_GAIN = 800  # [-], range [100, 2000] per the ur_rtde API reference

# {~.~} IMU information
USE_REFORGE_IMU = True
DEFAULT_IMU_COMM_MODE: Literal["ble", "usb", "virtual"] = "usb"
DEFAULT_IMU_RECORD_MODE: Literal["streaming", "logging"] = "streaming"
DEFAULT_IMU_RECORD_FREQUENCY_HZ = ROBOT_MAX_FREQ


@dataclass
class _RtdeClients:
    """Bundle the two `ur_rtde` client objects behind `ArmClient.robot`.

    `_require_connected_arm()` only checks `self.robot is not None`, so this
    wrapper is what lets every required method reach both the control and
    receive interfaces through the one `arm = self._require_connected_arm()`
    call already used throughout this file.
    """

    control: RTDEControlInterface
    receive: RTDEReceiveInterface


def _speed_pct_to_joint_sa(speed: float) -> tuple[float, float]:
    """Map a Reforge 0-100 speed percentage to moveJ's `(speed, acceleration)`.

    Args:
        speed: Speed percentage. Clamped into `[0, 100]`.
    Returns:
        `(speed_rad_s, acceleration_rad_s2)` scaled linearly against
        `MAX_JOINT_SPEED_RAD_S`/`MAX_JOINT_ACCEL_RAD_S2`.
    """
    fraction = max(0.0, min(100.0, speed)) / 100.0
    return fraction * MAX_JOINT_SPEED_RAD_S, fraction * MAX_JOINT_ACCEL_RAD_S2


def _speed_pct_to_linear_sa(speed: float) -> tuple[float, float]:
    """Map a Reforge 0-100 speed percentage to moveL's `(speed, acceleration)`.

    Args:
        speed: Speed percentage. Clamped into `[0, 100]`.

    Returns:
        `(speed_m_s, acceleration_m_s2)` scaled linearly against
        `MAX_LINEAR_SPEED_M_S`/`MAX_LINEAR_ACCEL_M_S2`.
    """
    fraction = max(0.0, min(100.0, speed)) / 100.0
    return fraction * MAX_LINEAR_SPEED_M_S, fraction * MAX_LINEAR_ACCEL_M_S2


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
        rtde_c: RTDEControlInterface | None = None,
        rtde_r: RTDEReceiveInterface | None = None,
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
            rtde_c: Optional pre-built `RTDEControlInterface`, injected in place
                of connecting to `robot_ip`. Tests supply a fake here so no
                socket is ever opened.
            rtde_r: Optional pre-built `RTDEReceiveInterface`, injected in place
                of connecting to `robot_ip`. Tests supply a fake here so no
                socket is ever opened.

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
        try:
            # Establish both RTDE clients. Each constructor call connects
            # synchronously and raises on failure (e.g. unreachable robot_ip,
            # rejected connection); those failures surface via the `except`
            # block below. Tests inject fakes via rtde_c/rtde_r so no socket
            # is ever opened.
            self.robot = _RtdeClients(
                control=rtde_c if rtde_c is not None else RTDEControlInterface(robot_ip),
                receive=rtde_r if rtde_r is not None else RTDEReceiveInterface(robot_ip),
            )

            # UR5e has no separate "unbrake"/ROS-handoff step over RTDE: a
            # successful RTDEControlInterface connection means the controller
            # already accepted remote control of the arm.

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

    def close(self) -> None:
        """Stop any active servo streaming and disconnect both RTDE clients.

        Not part of the `ArmClient` abstract contract; call this when done
        with the robot (e.g. at the end of a calibration run, or in a
        `finally` block) to leave the controller in a safe, disconnected
        state rather than relying on socket teardown at process exit.

        Side Effects:
            Decelerates and stops any in-progress servo motion, terminates
            the uploaded RTDE control script, and disconnects both clients.

        Returns:
            `None`.
        """
        if self.robot is None:
            return
        try:
            self.robot.control.servoStop()
            self.robot.control.stopScript()
        finally:
            self.robot.control.disconnect()
            self.robot.receive.disconnect()
            self.robot = None

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
        if IS_DEGREES:
            target_joints = list(np.rad2deg(angle) for angle in target_joints)

        arm = self._require_connected_arm()
        speed_rad_s, accel_rad_s2 = _speed_pct_to_joint_sa(speed)
        q = [float(v) for v in target_joints]

        ok = arm.control.moveJ(q, speed_rad_s, accel_rad_s2, not wait)
        if not ok:
            raise RuntimeError(f"moveJ to {q} failed")
        return 0

    def command_move_pose(
        self,
        target_quat: np.ndarray | list[float],
        target_xyz: np.ndarray | list[float],
        *,
        speed: float = 50.0,
        wait: bool = True,
        locked_joints: Mapping[int, float] | None = None,
    ) -> int:
        """Send a blocking/non-blocking point-to-point pose command using the
        robot's native position control interface.

        Args:
            target_quat: Target TCP orientation as a quaternion `[qx, qy, qz, qw]` [-] in the robot's base frame.
            target_xyz: Target TCP position `[x, y, z]` [m] in the robot's base frame.
            speed: Speed percentage for the motion, if supported by the robot. Default is 50%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.
            locked_joints: Simulator-only joint-index to fixed position map [rad].

        Returns:
            An integer status code from the robot's command interface, if applicable.
            If the robot does not provide a status code, return 0 for success or raise an exception for failure.
        """
        if locked_joints is not None:
            raise RuntimeError("locked_joints is only supported in simulator mode.")

        arm = self._require_connected_arm()
        speed_m_s, accel_m_s2 = _speed_pct_to_linear_sa(speed)

        # RTDE poses are [x, y, z, rx, ry, rz] with orientation as a rotation
        # vector; Reforge supplies orientation as [qx, qy, qz, qw]. scipy's
        # Rotation.from_quat() expects that same scalar-last ordering, so no
        # component reordering is needed here (mirrors get_tcp_pose()'s
        # inverse conversion).
        rotvec = Rotation.from_quat(list(target_quat)).as_rotvec()
        pose = [*(float(v) for v in target_xyz), *rotvec.tolist()]

        ok = arm.control.moveL(pose, speed_m_s, accel_m_s2, not wait)
        if not ok:
            raise RuntimeError(f"moveL to {pose} failed")
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
        del wait  # servoJ blocks internally for `time` seconds regardless of
        # any Python-level wait flag -- ur_rtde has no separate async servoJ
        # variant the way moveJ/moveL do.

        arm = self._require_connected_arm()
        q = [float(v) for v in target_joints]
        # This adapter's own command period; the base class's streaming loop
        # (ArmClient.stream_joint_positions) already paces calls to this
        # method at that rate, so `time` here just needs to match it.
        command_period_s = 1.0 / self.max_sampling_frequency_hz

        # speed/acceleration args are unused by the current RTDE protocol
        # version (see SERVO_LOOKAHEAD_TIME_S/SERVO_GAIN above), hence 0.0.
        ok = arm.control.servoJ(
            q, 0.0, 0.0, command_period_s, SERVO_LOOKAHEAD_TIME_S, SERVO_GAIN
        )
        if not ok:
            raise RuntimeError(f"servoJ to {q} failed")
        return 0

    def enter_position_mode(self) -> Optional[int | None]:
        """
        Ensure the controller is in point-to-point position mode before issuing queued P2P moves.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()

        # RTDE has no separate "position mode" handshake; moveJ/moveL can be
        # called directly. What does matter is halting any in-progress
        # servoJ stream first -- RTDE does not accept an interleaved
        # moveJ/moveL command while one is active -- so this ensures a clean
        # handoff from servo streaming back to point-to-point motion.
        ok = arm.control.servoStop()
        return 0 if ok else 1

    def enter_servo_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to servo control mode.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        self._require_connected_arm()

        # RTDE has no explicit servo-mode enable call: servoJ() can be
        # called directly once connected, and command_servo_j() computes its
        # own command period. Nothing to do here beyond the liveness check.
        return 0

    def supports_teaching_mode(self) -> bool:
        """Return whether the robot supports manual teaching mode.

        Override this method when the robot SDK supports hand-guided teaching.

        Returns:
            `bool` indicating whether manual teaching mode is implemented.
        """
        return False

    def enter_teaching_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to manual teaching mode.

        Override this method with the robot SDK's teaching-mode command.

        Returns:
            Vendor-specific mode/state code when available.
        """
        arm = self._require_connected_arm()  # noqa: F841

        # {~.~} Enable manual teaching mode using the robot SDK.
        # [YOUR CODE HERE]

        # {~.~} Return 0 for success - edit after implementation and testing
        return 1

    def supports_flange_button(self) -> bool:
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
        arm = self._require_connected_arm()

        # UR RTDE already reports SI radians/rad-s natively (see IS_DEGREES
        # above), so no unit conversion is applied here.
        q = list(arm.receive.getActualQ())
        qd = list(arm.receive.getActualQd())
        # `getJointTorques()` is gravity/friction-corrected and documented in
        # N-m (`ur_rtde` docs: "[Base, Shoulder, Elbow, Wrist1, Wrist2,
        # Wrist3]"), matching Reforge's expected effort unit directly. This
        # lives on the control interface, not the receive interface.
        tau = list(arm.control.getJointTorques())

        return q, qd, tau

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        arm = self._require_connected_arm()

        # getActualTCPPose() returns [x, y, z, rx, ry, rz]: position in meters
        # already, orientation as a rotation vector (not axis-angle + separate
        # magnitude, and not degrees) per the ur_rtde API reference.
        pose = arm.receive.getActualTCPPose()
        position, rotvec = list(pose[:3]), pose[3:]

        # scipy's Rotation.as_quat() returns scalar-last [qx, qy, qz, qw] by
        # default, which is exactly Reforge's expected pose ordering -- no
        # component reordering needed.
        quat = Rotation.from_rotvec(rotvec).as_quat().tolist()

        # Return tooltip pose as a list
        return [*position, *quat]

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
        return super().command_joint_trajectory(
            time_data=time_data,
            position_stream=position_stream,
            velocity_stream=velocity_stream,
            acceleration_stream=acceleration_stream,
            Ts=Ts,
        )

    # {~.~} END OF OPTIONAL OVERRIDES
