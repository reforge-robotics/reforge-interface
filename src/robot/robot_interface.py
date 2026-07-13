# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0

from importlib.resources import files, as_file
from typing import Optional, Sequence

import numpy as np
import trossen_arm
from scipy.spatial.transform import Rotation

from reforge_core.hw_interfaces.arm_client import ArmClient
from reforge_core.hw_interfaces.imu_recorder import ImuRecorder

# Trossen WidowX AI follower configuration.
#BOT_ID = "dd65af8b-1ea9-47db-83c7-8a75c4d0d817"
BOT_ID = "78554eb2-209e-4075-af33-d1d5ede177a5"
URDF_PATH = "urdf/trossen/wxai_follower.urdf"
ROBOT_MAX_FREQ = 200  # Trossen's documented high-rate recording frequency [Hz].
TROSSEN_MODEL = trossen_arm.Model.wxai_v0
TROSSEN_END_EFFECTOR = trossen_arm.StandardEndEffector.wxai_v0_follower
TROSSEN_INTERPOLATION_SPACE = trossen_arm.InterpolationSpace.joint
TROSSEN_DEFAULT_MOVE_TIME_S = 2.0
TROSSEN_GRIPPER_JOINT_INDEX = 6
TROSSEN_GRIPPER_HOLD_POSITION_M = 0.018035

# Fully stretched position of the robot for calibration.
FULL_STRETCH_XYZ = [0.517762, 0.0, 0.4275]  # Fallback from the bundled URDF [m].
FULL_STRETCH_QUAT = [0.0, 0.0, 0.0, 1.0]
FULL_STRETCH_JOINTS = [0.0, 5*np.pi / 6, 2*np.pi / 3, np.pi / 6, 0.0, 0.0]
FULL_STRETCH_POSE_OVERRIDE = None

# General constants
IS_DEGREES = False
DATA_LOCATION_PREFIX = "src/robot/data"
DEFAULT_TCP_PAYLOAD = 0.0

MAX_ROBOT_JOINTS_BANDWIDTH = 2.5

USE_REFORGE_IMU = True
DEFAULT_IMU_COMM_MODE = "usb"
DEFAULT_IMU_RECORD_FREQUENCY_HZ = ROBOT_MAX_FREQ
DEFAULT_IMU_TO_TCP_X = 0.0
DEFAULT_IMU_TO_TCP_Y = 0.0
DEFAULT_IMU_TO_TCP_Z = 0.043


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
            name="Trossen WidowX AI Follower",
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

        # Reforge API and robot ID token is needed for "feedforward" product
        # Add it in the CLI with `--identify`
        self.reforge_api_token = api_token
        try:
            if robot_ip != "sim":
                # Trossen's driver does not use the local IP or SDK token.
                _ = local_ip, sdk_token
                self.robot = trossen_arm.TrossenArmDriver()
                self.robot.configure(
                    TROSSEN_MODEL,
                    TROSSEN_END_EFFECTOR,
                    robot_ip,
                    False,
                )
                self.robot.set_arm_modes(trossen_arm.Mode.position)
                print(
                    "Connected to Trossen arm "
                    f"(driver={self.robot.get_driver_version()}, "
                    f"controller={self.robot.get_controller_version()})."
                )

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
        """Create the robot SDK adapter used when the Reforge IMU is disabled.

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

    def _validate_joint_target(
        self, target_joints: np.ndarray | list[float]
    ) -> list[float]:
        """Return a finite six-joint target in Trossen's native radian units."""
        joints = np.asarray(target_joints, dtype=float)
        if joints.shape != (self.num_joints,):
            raise ValueError(
                f"Expected {self.num_joints} joint targets, received {joints.shape}."
            )
        if not np.all(np.isfinite(joints)):
            raise ValueError("Joint targets must contain only finite values.")
        if IS_DEGREES:
            joints = np.rad2deg(joints)
        return joints.tolist()

    @staticmethod
    def _goal_time_from_speed(speed: float) -> float:
        """Map the interface speed percentage to Trossen's goal duration."""
        speed_value = float(speed)
        if not 0.0 < speed_value <= 100.0:
            raise ValueError("speed must be in the range (0, 100].")
        return float(
            np.clip(TROSSEN_DEFAULT_MOVE_TIME_S * 50.0 / speed_value, 0.2, 10.0)
        )

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
        arm.set_arm_positions(
            self._validate_joint_target(target_joints),
            self._goal_time_from_speed(speed),
            wait,
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
        arm = self._require_connected_arm()
        xyz = np.asarray(target_xyz, dtype=float)
        quat = np.asarray(target_quat, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError("target_xyz must be a finite three-vector in meters.")
        if quat.shape != (4,) or not np.all(np.isfinite(quat)):
            raise ValueError("target_quat must be a finite quaternion [x, y, z, w].")
        quat_norm = float(np.linalg.norm(quat))
        if quat_norm <= np.finfo(float).eps:
            raise ValueError("target_quat must have non-zero magnitude.")
        rotvec = Rotation.from_quat(quat / quat_norm).as_rotvec()
        arm.set_cartesian_positions(
            np.concatenate((xyz, rotvec)).tolist(),
            TROSSEN_INTERPOLATION_SPACE,
            self._goal_time_from_speed(speed),
            wait,
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
        arm = self._require_connected_arm()
        arm.set_arm_positions(
            self._validate_joint_target(target_joints),
            0.0,
            wait,
        )
        return 0

    def enter_position_mode(self) -> Optional[int | None]:
        """
        Ensure the controller is in point-to-point position mode before issuing queued P2P moves.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()
        arm.set_arm_modes(trossen_arm.Mode.position)
        return 0

    def enter_servo_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to servo control mode.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        arm = self._require_connected_arm()
        # Immediate set_arm_positions calls are Trossen's streamed position path.
        arm.set_arm_modes(trossen_arm.Mode.position)
        return 0

    def enter_teach_mode(self) -> int:
        """Make the arm backdrivable using Trossen gravity compensation."""
        arm = self._require_connected_arm()
        arm.set_arm_modes(trossen_arm.Mode.external_effort)
        arm.set_arm_external_efforts([0.0] * self.num_joints, 0.0, False)
        return 0

    def exit_teach_mode(self) -> int:
        """Brake the arm joints after manual positioning."""
        arm = self._require_connected_arm()
        arm.set_arm_modes(trossen_arm.Mode.idle)
        return 0

    def command_gripper_position(
        self,
        position_m: float,
        *,
        move_time_s: float = TROSSEN_DEFAULT_MOVE_TIME_S,
        wait: bool = True,
    ) -> int:
        """Move the gripper to a fixed opening and hold it in position mode."""
        arm = self._require_connected_arm()
        position = float(position_m)
        move_time = float(move_time_s)
        if not np.isfinite(position):
            raise ValueError("Gripper position must be finite.")
        if not np.isfinite(move_time) or move_time <= 0.0:
            raise ValueError("Gripper move time must be finite and greater than zero.")

        limits = arm.get_joint_limits()
        if len(limits) <= TROSSEN_GRIPPER_JOINT_INDEX:
            raise RuntimeError("Trossen controller did not report a gripper joint limit.")
        gripper_limit = limits[TROSSEN_GRIPPER_JOINT_INDEX]
        lower = float(gripper_limit.position_min)
        upper = float(gripper_limit.position_max)
        if not lower <= position <= upper:
            raise ValueError(
                f"Gripper position must be within [{lower}, {upper}] m; "
                f"received {position} m."
            )

        arm.set_gripper_mode(trossen_arm.Mode.position)
        arm.set_gripper_position(position, move_time, wait)
        return 0

    def get_gripper_position(self) -> float:
        """Return the current gripper opening in meters."""
        return float(self._require_connected_arm().get_gripper_position())

    def hold_gripper(self) -> int:
        """Hold the installed IMU at the measured fixed gripper position."""
        return self.command_gripper_position(TROSSEN_GRIPPER_HOLD_POSITION_M)

    def get_joint_state(self) -> tuple[list[float], list[float], list[float]]:
        """Return one joint state sample as ``(q, qd, tau)``.

        Returns:
            Tuple of three lists: joint positions `q` [rad], velocities `qd` [rad/s],
            and efforts/currents `tau` [SDK units].
        """
        arm = self._require_connected_arm()
        output = arm.get_robot_output()
        q = list(output.joint.arm.positions)
        qd = list(output.joint.arm.velocities)
        tau = list(output.joint.arm.efforts)
        lengths = (len(q), len(qd), len(tau))
        if lengths != (self.num_joints, self.num_joints, self.num_joints):
            raise RuntimeError(
                "Trossen joint-state dimensions do not match the URDF model: "
                f"q/qd/tau={lengths}, expected {self.num_joints}."
            )
        if IS_DEGREES:
            q = np.deg2rad(q).tolist()
            qd = np.deg2rad(qd).tolist()
        return q, qd, tau

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        arm = self._require_connected_arm()
        pose = np.asarray(arm.get_cartesian_positions(), dtype=float)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise RuntimeError(
                f"Trossen returned an invalid Cartesian pose with shape {pose.shape}."
            )
        quat = Rotation.from_rotvec(pose[3:]).as_quat()
        return [*pose[:3].tolist(), *quat.tolist()]
