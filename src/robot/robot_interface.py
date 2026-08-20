# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0

from collections.abc import Mapping
from importlib.resources import as_file, files
from pathlib import Path
from typing import Literal, Optional, Sequence

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
from xarm import XArmAPI
from scipy.spatial.transform import Rotation as R

# User constants - EDITS REQUIRED
BOT_ID = ""  # Need to update when I get the API working
URDF_PATH = "urdf/uf850.urdf"
ROBOT_MAX_FREQ = 250  # [Hz]

# Fully stretched position of the robot for calibration.
FULL_STRETCH_XYZ = [0.814, 0.0007, 0.425]  # [m]
FULL_STRETCH_QUAT = [0.0, 0.0, 0.0, 1.0]
FULL_STRETCH_JOINTS = [0.0, -np.pi / 2, -np.pi, 0.0, -np.pi / 2, 0.0]  # [rad]
FULL_STRETCH_POSE_OVERRIDE = [0.814, 0.0007, 0.364]

# General constants
IS_DEGREES = False
DATA_LOCATION_PREFIX = "src/robot/data"
SIM_DATA_LOCATION_PREFIX = str(Path(__file__).resolve().parent / "data" / "sim")
DEFAULT_TCP_PAYLOAD = 0.0

MAX_ROBOT_JOINTS_BANDWIDTH = 5.0
TEACHING_DETECTION_PARAM = 1
FLANGE_BUTTON_INPUT_INDEX = 2
FLANGE_BUTTON_PRESSED_VALUE = 1


# {~.~} IMU information
USE_REFORGE_IMU = True
DEFAULT_IMU_COMM_MODE: Literal["ble", "usb", "virtual"] = "usb"
DEFAULT_IMU_RECORD_MODE: Literal["streaming", "logging"] = "streaming"
DEFAULT_IMU_RECORD_FREQUENCY_HZ = ROBOT_MAX_FREQ


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

        super().__init__(name="UFACTORY", recording_data_frequency_hz=ROBOT_MAX_FREQ)

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
            # {~.~} Instantiate live robot mode
            self.robot = XArmAPI(robot_ip, is_radian=not IS_DEGREES)

            # Enable robot and set to "motion" state (0)
            self.robot.motion_enable(enable=True)
            self.robot.set_state(state=0)

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
            target_joints: Target joint positions [rad] as a list, tuple, or array.
            speed: Speed percentage for the motion, if supported by the robot. Default is 50%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.

        Returns:
            An integer status code from the robot's command interface, if applicable.
            If the robot does not provide a status code, return 0 for success or raise an exception for failure.
        """
        if IS_DEGREES:
            target_joints = list(np.rad2deg(angle) for angle in target_joints)

        arm = self._require_connected_arm()
        cmd_q = list(target_joints)
        if not arm._is_radian:
            cmd_q = [np.rad2deg(value) for value in cmd_q]

        self.enter_position_mode()
        code = arm.set_servo_angle(
            angle=cmd_q,
            speed=speed / 100,
            mvacc=None,
            mvtime=None,
            wait=wait,
        )
        return code

    def command_move_pose(
        self,
        target_quat: np.ndarray | list[float],
        target_xyz: np.ndarray | list[float],
        *,
        speed: float = 50.0,
        wait: bool = True,
        max_retries: int = 3,
        locked_joints: Mapping[int, float] | None = None,
    ) -> int:
        """Send a blocking/non-blocking point-to-point pose command using the
        robot's native position control interface.

        Args:
            target_quat: Target TCP orientation as a quaternion `[qx, qy, qz, qw]` [-] in the robot's base frame.
            target_xyz: Target TCP position `[x, y, z]` [m] in the robot's base frame.
            speed: Speed percentage for the motion, if supported by the robot. Default is 50%.
            wait: If `True`, block until the motion is complete. If `False`, return immediately after sending the command.
            max_retries: Number of error-clear-and-retry attempts on code-1 failures.
            locked_joints: Simulator-only joint-index to fixed position map [rad].

        Returns:
            An integer status code from the robot's command interface, if applicable.
            If the robot does not provide a status code, return 0 for success or raise an exception for failure.
        """
        if locked_joints is not None:
            raise RuntimeError("locked_joints is only supported in simulator mode.")

        arm = self._require_connected_arm()

        # Convert quaternion to axis-angle, as expected by API
        axang = R.from_quat(target_quat).as_rotvec().tolist()

        # Convert to degrees, if expected by API
        if not self.robot._is_radian:
            axang = [np.rad2deg(coord) for coord in axang]

        # Convert xyz to millimeters, as expected by API
        position = [coord * 1000.0 for coord in target_xyz]
        pose = [*position, *axang]

        # Servo mode from realtime streaming blocks set_position_* APIs; switch back to
        # position mode. enter_position_mode also clears latched hardware errors (e.g.
        # collision false-positives from high-torque servo trajectories) before each attempt.
        code = 1
        for attempt in range(max_retries):
            self.enter_position_mode()
            code = arm.set_position_aa(
                axis_angle_pose=pose,
                speed=None,
                mvacc=None,
                mvtime=None,
                motion_type=1,
                wait=wait,
            )
            print(
                f"[ufactory850] set_position_aa -> {code} (attempt {attempt + 1}/{max_retries})"
            )
            if code == 0:
                return code
            print(f"[ufactory850] set_position_aa failed with code {code}, retrying...")

        raise RuntimeError(
            f"Pose command failed with code {code} after {max_retries} attempts "
            f"for target quat={target_quat}, xyz={target_xyz}"
        )

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
        arm = self._require_connected_arm()
        cmd_q = list(target_joints)
        if not arm._is_radian:
            cmd_q = [np.rad2deg(value) for value in cmd_q]

        code = arm.set_servo_angle_j(
            angles=cmd_q,
            wait=wait,
        )
        return code

    def enter_position_mode(self) -> Optional[int | None]:
        """
        Ensure the controller is in point-to-point position mode before issuing queued P2P moves.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()
        # Clear any latched hardware errors (e.g. collision-detection false positives
        # from high servo-mode torques) so the subsequent set_mode/set_state are not
        # blocked and the next position command does not see a stale error_code.
        arm.clean_error()
        arm.clean_warn()
        code_mode = arm.set_mode(0)
        code_state = arm.set_state(0)
        print(f"[ufactory850] set_mode(0)->{code_mode}, set_state(0)->{code_state}")
        return code_mode

    def enter_teaching_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to manual teaching mode.

        Returns:
            The xArm teaching-mode status code when available.

        Raises:
            RuntimeError: If the controller reports the known xArm teaching-mode
                failure state after the mode switch.
        """
        arm = self._require_connected_arm()

        # Reset stale warnings before entering hand-guided teaching so the xArm
        # controller accepts the new mode transition.
        arm.clean_error()
        arm.clean_warn()
        code_mode_reset = arm.set_mode(0)
        code_mode_teaching = arm.set_mode(2, TEACHING_DETECTION_PARAM)
        code_state = arm.set_state(0)
        print(
            "[ufactory850] "
            f"set_mode(0)->{code_mode_reset}, "
            f"set_mode(2, {TEACHING_DETECTION_PARAM})->{code_mode_teaching}, "
            f"set_state(0)->{code_state}"
        )

        if getattr(arm, "error_code", 0) == 37:
            raise RuntimeError(
                "xArm entered error 37 after switching to teaching mode."
            )

        return code_mode_teaching

    def supports_teaching_mode(self) -> bool:
        """Return whether the xArm backend supports manual teaching mode.

        Returns:
            `bool` true because this backend implements `enter_teaching_mode()`.

        Raises:
            None.
        """
        return True

    def supports_flange_button(self) -> bool:
        """Return whether the xArm backend exposes a readable flange button.

        Returns:
            `bool` true because this backend implements flange-button reads.

        Raises:
            None.
        """
        return True

    def read_flange_button_pressed(self) -> bool:
        """Return whether the xArm flange button is currently pressed.

        Returns:
            `bool` true when the configured Tool GPIO input reports a pressed
            flange button.

        Raises:
            RuntimeError: If the Tool GPIO read fails or the configured input
                index is outside the returned digital-input vector.
        """
        arm = self._require_connected_arm()
        code, input_values = arm.get_tool_digital_input()
        if code != 0:
            raise RuntimeError(
                "xArm Tool GPIO read failed while checking the flange button "
                f"(code={code})."
            )

        digital_inputs = [int(value) for value in input_values]
        if FLANGE_BUTTON_INPUT_INDEX >= len(digital_inputs):
            raise RuntimeError(
                "Configured xArm flange-button input index "
                f"{FLANGE_BUTTON_INPUT_INDEX} is outside the Tool GPIO input "
                f"vector of length {len(digital_inputs)}."
            )

        return digital_inputs[FLANGE_BUTTON_INPUT_INDEX] == FLANGE_BUTTON_PRESSED_VALUE

    def enter_servo_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to servo control mode.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()  # noqa: F841

        print("Preparing arm for next move...")
        code_en = arm.motion_enable(enable=True)
        code_mode = arm.set_mode(1)  # servo mode
        code_state = arm.set_state(0)  # start
        print(
            f"[ArmImuManager] motion_enable->{code_en}, set_mode(1)->{code_mode}, set_state(0)->{code_state}"
        )
        return code_mode

    def get_joint_state(self) -> tuple[list[float], list[float], list[float]]:
        """Return one joint state sample as ``(q, qd, tau)``.

        Returns:
            Tuple of three lists: joint positions `q` [rad], velocities `qd` [rad/s],
            and efforts/currents `tau` [SDK units].
        """
        arm = self._require_connected_arm()  # noqa: F841
        q: list[float] = []
        qd: list[float] = []
        tau: list[float] = []

        code, payload = arm.get_joint_states()
        if code != 0:
            raise RuntimeError(f"get_joint_states returned code {code}")
        q, qd, tau = payload
        q = list(q[: self.num_joints])
        qd = list(qd[: self.num_joints])
        tau = list(tau[: self.num_joints])

        if not arm._is_radian:
            q = [np.deg2rad(value) for value in q]
            qd = [np.deg2rad(value) for value in qd]

        return q, qd, tau

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        position: list[float] = []
        quat: list[float] = []
        arm = self._require_connected_arm()  # noqa: F841

        # ------------------------------ EXAMPLE ------------------------------
        code, pose = arm.get_position_aa()
        if code != 0:
            raise Exception(f"Unreliable TCP pose! Return code {code}")

        # Additional operations may be needed depending on pose return style
        # Decompose pose if necessary
        position, axang = pose[:3], pose[3:]

        # Convert position coordinates to meters
        position = [coord / 1000.0 for coord in position]

        # Convert rotation coordinates to radians, if necessary
        if not arm._is_radian:
            axang = [np.deg2rad(coord) for coord in axang]

        # Convert axis-angle to quaternion
        quat = R.from_rotvec(axang).as_quat().tolist()

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
