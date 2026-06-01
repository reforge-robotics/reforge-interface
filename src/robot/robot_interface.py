# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 1.0

import datetime
import os
import numpy as np
from collections import deque
from importlib.resources import files, as_file
from reforge_core.util.utility import TrajParams, SystemIdParams, polar_to_cartesian
from reforge_core.util.trajectory import Trajectory
from typing import TYPE_CHECKING, Any, Deque, List, Dict, Tuple, Sequence
from robot.robot_base import Robot, DataRecorder

from robot.robot_base import (
    get_polar_coordinates,
    store_parameters_in_data_folder,
    store_recorder_data_in_data_folder,
)
from reforge_core.util.robot_dynamics import Dynamics

from robot.robot_base import (
    DEFAULT_HOME_SIGN,
    DEFAULT_FIRST_AXIS,
    DEFAULT_MIN_CALIBRATION_ANGLE,
    DEFAULT_MAX_CALIBRATION_ANGLE,
    DEFAULT_MIN_CALIBRATION_RADIUS_SCALE,
    DEFAULT_MAX_CALIBRATION_RADIUS_SCALE,
    DEFAULT_TCP_PAYLOAD,
    DEFAULT_IMU_TO_TCP_X,
    DEFAULT_IMU_TO_TCP_Y,
    DEFAULT_IMU_TO_TCP_Z,
)


from reforge_core.util.utility import (
    DEFAULT_CONFIG,
    DEFAULT_BCB_RUNTIME,
    DEFAULT_SYSID_TYPE,
    DEFAULT_SINE_MIN_FREQ,
    DEFAULT_SINE_MAX_FREQ,
    DEFAULT_FREQ_SPACING,
    DEFAULT_SINE_CYCLES,
    DEFAULT_DWELL_TIME,
    DEFAULT_MAX_DISP,
    DEFAULT_MAX_VEL,
    DEFAULT_MAX_ACC,
    DEFAULT_SYSID_ANGLES,
    DEFAULT_SYSID_RADII,
    DEFAULT_FIRST_POSE,
    SysIdType,
)

if TYPE_CHECKING:
    from reforge_core.control.python.joint_control import JointInversionController

# ------NOTES-----
# 1. Where you see the #{~.~} symbol, you need to make a change. Use Ctrl+F to find all instances.
# The general flow will be the following:
#   a. Import the robot's Python SDK
#   b. Change the BOT_ID, URDF_PATH, ROBOT_MAX_FREQ, and
#      HOME_SHOULDER_ANGLE, HOME_XYZ, HOME_QUAT, and HOME_JOINTS constants
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
BOT_ID = ""  # {~.~} [CHANGE TO ROBOT's ID, IF NECESSARY] - can also enter as CLI argument (see run.py --help)
URDF_PATH = "urdf/test_robot.urdf"  # {~.~} [CHANGE TO YOUR ROBOT'S URDF FILE PATH]
ROBOT_MAX_FREQ = 1000  # {~.~} [CHANGE TO ROBOT'S MAX SAMPLING FREQUENCY] in [Hz]

# Home position of the robot
HOME_SHOULDER_ANGLE = np.pi / 2  # {~.~} [rad]
HOME_XYZ = [1.28989, 0.36866, 0.171]  # {~.~} [m]
HOME_QUAT = [0.499, 0.499, 0.499, 0.499]  # {~.~} [1]
HOME_JOINTS = [0.0, np.pi / 2, 0.0, 0.0, 0.0, 0.0]  # {~.~} [rad]
HOME_POSE_OVERRIDE = None  # {~.~} list of home pose (xyz and quaternion) to override additional height not in base height

# General constants
IS_DEGREES = False  # {~.~} [CHANGE TO TRUE IF ROBOT USES DEGREES]
DATA_LOCATION_PREFIX = "src/robot/data"  # {~.~} [CHANGE TO LOCATION DESIRED - will be robot/DATA_LOCATION_PREFIX/*]

MAX_ROBOT_JOINTS_BANDWIDTH = (
    7.5  # {~.~} Servo motor bandwidth. Leave as is if you don't know [Hz]
)


# TODO: Rethink RobotInterface. It is mixing a lot of responsibility.
# Shared functionality that isn't user implementation-specific is
# supposed to go in Robot.
class RobotInterface(Robot):
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
        tcp_payload: float = DEFAULT_TCP_PAYLOAD,
        tcp_payload_com: Sequence[float] | None = None,
        local_ip: str = "",
        sdk_token: str = "",
        api_token: str = "",
        robot_id: str = BOT_ID,
    ) -> None:
        """Initialize the robot interface and load the URDF model.

        Args:
            robot_ip: Robot IP address or `sim` for simulator mode.
            tcp_payload: Payload of the tcp (default=0)
            tcp_payload_com: Optional 3x1 center of mass of the tcp payload.
            local_ip: Local machine IP address if required by the SDK.
            sdk_token: SDK authentication token.
            robot_id: Identifier used by the control stack.

        Side Effects:
            Loads the URDF model and may connect to robot hardware.

        Raises:
            RuntimeError: If the robot connection fails.
            ValueError: If reported joint counts do not match the URDF.

        Preconditions:
            The URDF file is available and the SDK is installed.
        """
        super().__init__("My Robot")  # {~.~} [Edit to your robot's name, if desired]
        # Initialize Robot attributes
        self.recorder = DataRecorder()

        # Initialize URDF location
        self.module_dir = files("robot")
        resource = self.module_dir.joinpath(URDF_PATH)
        with as_file(resource) as p:
            self._urdf_path = str(p)
        print(f"URDF Path: {self._urdf_path}")

        # Dynamic model loaded flag
        self.model_is_loaded = False

        # Check if robot is in simulation mode
        self._in_sim_mode = robot_ip == "sim"

        # Load robot model from URDF
        if not self.model_is_loaded:
            self.initialize_model_from_urdf(
                urdf_path=self.urdf_path,
                tcp_payload=tcp_payload,
                tcp_payload_com=tcp_payload_com,
            )
            # Use the model joint count as the ground truth for downstream
            # dynamics calls (the hardware may report extra fixed joints/grippers).
            self.num_joints = self.model.num_joints

        self.reforge_api_token = api_token
        try:
            if robot_ip != "sim":
                # {~.~} Instantiate robot
                self.robot = None  # [CHANGE THIS LINE]

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

                self.num_joints = len(self.__get_joint_positions())
                self.pose_length = len(self.__get_tcp_pose())
            else:
                self.id = robot_id
                self.num_joints = 6
                self.pose_length = 7

        except Exception as e:
            # Print exception error message
            raise RuntimeError(f"Error getting {robot_ip} operational: {str(e)}")

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
    def __get_joint_positions(self) -> List:
        """Get the current joint positions of the robot.

        Returns:
            `list[float]` joint positions in radians.

        Side Effects:
            Queries the robot hardware or simulator state.

        Raises:
            RuntimeError: If the robot returns no joint rotations.
            ValueError: If the number of joints does not match the URDF.

        Preconditions:
            The robot connection is active.
        """
        joint_angles: list[float] = []

        # --------------------------- EXAMPLE ---------------------------
        # position = self.robot.movement.position.get_arm_position().ok()
        # joint_angles = list(position.joint_rotations)
        # ---------------------------------------------------------------

        if IS_DEGREES:
            joint_angles = [np.deg2rad(angle) for angle in joint_angles]

        # Hardware may return more joints than the URDF dynamics model (e.g., gripper).
        if len(joint_angles) > self.num_joints:
            joint_angles = joint_angles[: self.num_joints]
        elif len(joint_angles) < self.num_joints:
            raise ValueError(
                f"Received {len(joint_angles)} joint angles but expected {self.num_joints} from URDF model"
            )

        # Return joint positions [rad] as a list
        return joint_angles

    def __get_tcp_pose(self) -> List:
        """Get the current tool center point pose.

        Returns:
            `list[float]` pose as [x, y, z, qx, qy, qz, qw].

        Side Effects:
            Queries the robot hardware or simulator state.

        Raises:
            RuntimeError: If the robot returns missing pose data.

        Preconditions:
            The robot connection is active.
        """
        pose: list[float] = []

        # --------------------------- EXAMPLE ---------------------------
        # position = self.robot.movement.position.get_arm_position().ok()
        # pose = [position.tooltip_position.position.x,
        #         position.tooltip_position.position.y,
        #         position.tooltip_position.position.z,
        #         position.tooltip_position.orientation.quaternion.x,
        #         position.tooltip_position.orientation.quaternion.y,
        #         position.tooltip_position.orientation.quaternion.z,
        #         position.tooltip_position.orientation.quaternion.w]
        # ---------------------------------------------------------------

        # Return tooltip pose as a list
        return pose

    def move_to_joint(self, target_joint: Tuple[float, ...]) -> None:
        """Move the robot to the specified joint positions.

        Args:
            target_joint: Joint positions in radians.

        Returns:
            `None`.

        Side Effects:
            Commands the robot to move.

        Raises:
            ValueError: If the number of joints does not match the URDF.

        Preconditions:
            The robot connection is active.
        """
        if IS_DEGREES:
            target_joint = tuple(np.rad2deg(angle) for angle in target_joint)

        # ------------------------------------ EXAMPLE -------------------------------------
        # update_request = models.ArmPositionUpdateRequest(
        #     kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
        #     joint_rotation=models.ArmJointRotations(
        #         joints=target_joint)
        # )

        # response = self.robot.movement.position.set_arm_position(body=update_request).ok()
        # ----------------------------------------------------------------------------------

    def move_to_pose(self, target_quat: List[float], target_xyz: List[float]) -> None:
        """Move the robot to the specified Cartesian pose.

        Args:
            target_quat: Quaternion [qx, qy, qz, qw].
            target_xyz: Position [x, y, z] in meters.

        Returns:
            `None`.

        Side Effects:
            Commands the robot to move.

        Raises:
            ValueError: If input lists are the wrong length.

        Preconditions:
            The robot connection is active.
        """
        # ---------------------------------- EXAMPLE ------------------------------------
        # quatx, quaty, quatz, quatw = target_quat
        # move_quat = models.Orientation(
        #                 kind=models.OrientationKindEnum.Quaternion,
        #                 quaternion=models.Quaternion(x=quatx,
        #                                              y=quaty,
        #                                              z=quatz,
        #                                              w=quatw
        #                                              ),
        #             )
        # x, y, z = target_xyz
        # move_xyz = models.Position(
        #                 unit_kind=models.LinearUnitKind.Meters,
        #                 x=x,
        #                 y=y,
        #                 z=z
        #             )

        # update_request = models.ArmPositionUpdateRequest(
        #     kind=models.ArmPositionUpdateRequestKindEnum.TooltipPosition,
        #     tooltip_position=models.PositionAndOrientation(
        #         position=move_xyz,
        #         orientation=move_quat)
        # )

        # response = self.robot.movement.position.set_arm_position(body=update_request).ok()
        # ----------------------------------------------------------------------------------

    def publish_and_record_joint_positions(
        self,
        time_data: List,
        position_stream: List,
        velocity_stream: List = [],
        acceleration_stream: List = [],
        Ts: float = 1 / ROBOT_MAX_FREQ,
    ) -> Deque[Dict]:
        """Publish joint trajectories and record streamed data.

        Args:
            time_data: Time stamps for each command.
            position_stream: Joint position commands (N x num_joints).
            velocity_stream: Joint velocity commands (N x num_joints).
            acceleration_stream: Joint acceleration commands (N x num_joints).
            Ts: Sampling time [s].

        Returns:
            `collections.deque[dict]` log of recorded data entries.

        Side Effects:
            Sends commands over ROS, starts threads, and records data.

        Raises:
            RuntimeError: If ROS or robot communication fails.

        Preconditions:
            Robot connection is active and ROS control is enabled.
        """
        # {~.~} Publish joint positions to the robot
        # [YOUR CODE HERE -- see example with ros_manager.py below]

        # ------------------------------ EXAMPLE (joint pub) ------------------------------
        # from robot.ros_manager import (
        #     JointTrajectoryController,
        #     rclpy,
        #     threading,
        # )

        # rclpy.init()

        # publish_complete_event = threading.Event()

        # joint_controller = JointTrajectoryController(
        #     f"{self.id}/ro1/hardware", # ROS topic
        #     Ts,
        #     time_data,
        #     position_stream,
        #     velocity_stream,
        #     acceleration_stream,
        #     publish_complete_event
        # )

        # thread = threading.Thread(target=self.spin_thread, args=(joint_controller, ), daemon=True)
        # thread.start()

        # # wait until trajectory publishing finishes
        # publish_complete_event.wait()
        # rclpy.shutdown()
        # thread.join()
        # END of joint pub example
        # -------------------------------------------------------------------------------

        # {~.~} Store the recorded data in a deque and return
        # Data log must have the following structure:
        # {'cmd_time','input_positions','output_positions','velocities','efforts',
        # 'imu_time','linear_acceleration','angular_velocity','orientation'}

        # STRUCTURE:
        # {'cmd_time': servo_timestamp,
        # 'input_positions': [j0_pos_cmd,j1_pos_cmd,j2_pos_cmd,
        #                     j3_pos_cmd,j4_pos_cmd,j5_pos_cmd],
        # 'output_positions': [j0_pos_enc,j1_pos_enc,j2_pos_enc,
        #                      j3_pos_enc,j4_pos_enc,j5_pos_enc],
        # 'velocities': [j0_vel,j1_vel,j2_vel,j3_vel,j4_vel,j5_vel],
        # 'efforts': [j0_current,j1_current,j2_current,j3_current,j4_current,j5_current],
        # 'imu_time': imu_timestamp,
        # 'linear_acceleration': [acc_x,acc_y,acc_z],
        # 'angular_velocity': [gyro_x,gyro_y,gyro_z],
        # 'orientation': [w,x,y,z]}

        # Leave the field empty if not available, but have a row entry for each time step (see ros_manager.py for example)
        data_log: Deque[Dict] = (
            deque()
        )  # {'cmd_time','input_positions','output_positions','velocities','efforts','imu_time','linear_acceleration','angular_velocity','orientation'}

        # ------- EXAMPLE (record joint and imu data) ------
        # data_log = joint_controller.aligned_log
        #
        # Destroy the node explicitly after recording data
        # joint_controller.destroy_node()
        #
        # END of record data example
        # --------------------------------------------------

        return (
            data_log  # {'cmd_time','input_positions','output_positions','velocities',
        )
        # 'efforts','imu_time','linear_acceleration','angular_velocity','orientation'}

    # {~.~} END OF REQUIRED METHODS

    # PRE-DEFINED METHODS
    def calibrate_robot(
        self,
        Ts: float,
        axes_to_command: int,
        max_disp: float = DEFAULT_MAX_DISP,
        max_vel: float = DEFAULT_MAX_VEL,
        max_acc: float = DEFAULT_MAX_ACC,
        bcb_runtime: float = DEFAULT_BCB_RUNTIME,
        ctrl_config: str = DEFAULT_CONFIG,
        sysid_type: SysIdType | str = DEFAULT_SYSID_TYPE,
        nV: int = DEFAULT_SYSID_ANGLES,
        nR: int = DEFAULT_SYSID_RADII,
        min_angle: float = DEFAULT_MIN_CALIBRATION_ANGLE,
        max_angle: float = DEFAULT_MAX_CALIBRATION_ANGLE,
        min_radius_scale: float = DEFAULT_MIN_CALIBRATION_RADIUS_SCALE,
        max_radius_scale: float = DEFAULT_MAX_CALIBRATION_RADIUS_SCALE,
        min_sine_freq: float = DEFAULT_SINE_MIN_FREQ,
        max_sine_freq: float = DEFAULT_SINE_MAX_FREQ,
        sine_freq_spacing: float = DEFAULT_FREQ_SPACING,
        num_sine_cycles: int = DEFAULT_SINE_CYCLES,
        dwell_btw_sine: float = DEFAULT_DWELL_TIME,
        start_pose: int = DEFAULT_FIRST_POSE,
        start_axis: int = DEFAULT_FIRST_AXIS,
        home_sign: int = DEFAULT_HOME_SIGN,
        imu_to_tcp_x: float = DEFAULT_IMU_TO_TCP_X,
        imu_to_tcp_y: float = DEFAULT_IMU_TO_TCP_Y,
        imu_to_tcp_z: float = DEFAULT_IMU_TO_TCP_Z,
    ) -> str:
        """Run the system identification trajectory and record calibration data.

        Args:
            Ts: Sampling time [s].
            axes_to_command: Number of axes to command.
            max_disp: Maximum displacement [rad].
            max_vel: Maximum velocity [rad/s].
            max_acc: Maximum acceleration [rad/s^2].
            bcb_runtime: Runtime for bang-coast-bang system ID [s].
            ctrl_config: Control configuration (`task` or `joint`).
            sysid_type: System identification workflow (`shaper`, `feedforward`, or `bcb`).
            nV: Number of angle positions.
            nR: Number of radius positions.
            min_angle: Minimum calibration angle from horizontal [rad].
            max_angle: Maximum calibration angle from horizontal [rad].
            min_radius_scale: Minimum calibration radius scale.
            max_radius_scale: Maximum calibration radius scale.
            min_sine_freq: Minimum sine sweep frequency [Hz].
            max_sine_freq: Maximum sine sweep frequency [Hz].
            sine_freq_spacing: Frequency spacing [Hz].
            num_sine_cycles: Number of sine cycles per pose.
            dwell_btw_sine: Dwell time between sweeps [s].
            start_pose: Starting pose index.
            start_axis: Starting joint index for commanded-axis block.
            home_sign: Sign of shoulder joint angle at home position.
            imu_to_tcp_x: X component of IMU->TCP translation [m].
            imu_to_tcp_y: Y component of IMU->TCP translation [m].
            imu_to_tcp_z: Z component of IMU->TCP translation [m].

        Returns:
            `str` folder where calibration data is stored.

        Side Effects:
            Commands robot motion, collects sensor data, and writes files to disk.

        Raises:
            RuntimeError: If sampling frequency exceeds robot limits.

        Preconditions:
            Robot connection is active and calibration data directory is writable.
        """
        # Check to make sure the sampling frequency is not larger than the max sampling frequency
        if ROBOT_MAX_FREQ < (1 / Ts):
            raise RuntimeError(
                f"The sample time exceeds the maximum robot\
                                sampling frequency of {ROBOT_MAX_FREQ} Hz."
            )
        sysid_type_enum = SysIdType(sysid_type)
        if start_axis < 0:
            raise ValueError("start_axis must be >= 0.")
        if axes_to_command <= 0:
            raise ValueError("axes_to_command must be > 0.")
        if (start_axis + axes_to_command) > self.num_joints:
            raise ValueError(
                "Requested commanded-axis range exceeds robot joints: "
                f"start_axis={start_axis}, axes_to_command={axes_to_command}, "
                f"num_joints={self.num_joints}."
            )
        imu_to_tcp_xyz = [
            float(imu_to_tcp_x),
            float(imu_to_tcp_y),
            float(imu_to_tcp_z),
        ]
        self.move_home(home_sign=home_sign)
        traj_params, sysid_params = self._build_calibration_params(
            max_disp=max_disp,
            max_vel=max_vel,
            max_acc=max_acc,
            bcb_runtime=bcb_runtime,
            ctrl_config=ctrl_config,
            sysid_type=sysid_type_enum,
            nV=nV,
            nR=nR,
            min_sine_freq=min_sine_freq,
            max_sine_freq=max_sine_freq,
            sine_freq_spacing=sine_freq_spacing,
            num_sine_cycles=num_sine_cycles,
            dwell_btw_sine=dwell_btw_sine,
        )
        data_folder = self._create_calibration_data_folder()
        self._store_calibration_parameters(
            traj_params=traj_params,
            sysid_params=sysid_params,
            axes_to_command=axes_to_command,
            Ts=Ts,
            start_pose=start_pose,
            start_axis=start_axis,
            imu_to_tcp_x=imu_to_tcp_x,
            imu_to_tcp_y=imu_to_tcp_y,
            imu_to_tcp_z=imu_to_tcp_z,
            data_folder=data_folder,
            joint_controller_id_completed=False,
        )
        v_list, r_list = get_polar_coordinates(
            sysid_params.nV,
            sysid_params.nR,
            self.max_reach,
            min_angle=min_angle,
            max_angle=max_angle,
            min_radius_scale=min_radius_scale,
            max_radius_scale=max_radius_scale,
        )
        if sysid_type_enum == SysIdType.SHAPER:
            return self._run_shaper_calibration(
                Ts=Ts,
                axes_to_command=axes_to_command,
                traj_params=traj_params,
                sysid_params=sysid_params,
                v_list=v_list,
                r_list=r_list,
                start_pose=start_pose,
                start_axis=start_axis,
                data_folder=data_folder,
            )
        if sysid_type_enum == SysIdType.FEEDFORWARD:
            return self._run_feedforward_calibration(
                Ts=Ts,
                axes_to_command=axes_to_command,
                traj_params=traj_params,
                sysid_params=sysid_params,
                v_list=v_list,
                r_list=r_list,
                start_pose=start_pose,
                start_axis=start_axis,
                imu_to_tcp_xyz=imu_to_tcp_xyz,
                data_folder=data_folder,
            )
        raise ValueError(
            "Unsupported sysid_type. Expected one of: 'shaper', 'feedforward'."
        )

    def _build_calibration_params(
        self,
        max_disp: float,
        max_vel: float,
        max_acc: float,
        bcb_runtime: float,
        ctrl_config: str,
        sysid_type: SysIdType | str,
        nV: int,
        nR: int,
        min_sine_freq: float,
        max_sine_freq: float,
        sine_freq_spacing: float,
        num_sine_cycles: int,
        dwell_btw_sine: float,
    ) -> tuple[TrajParams, SystemIdParams]:
        """Build the trajectory and identification parameter objects for one calibration run.

        Args:
            max_disp: Maximum commanded displacement [rad].
            max_vel: Maximum commanded velocity [rad/s].
            max_acc: Maximum commanded acceleration [rad/s^2].
            bcb_runtime: Point-to-point runtime used by trajectory generation [s].
            ctrl_config: Control configuration string (`task` or `joint`).
            sysid_type: Calibration workflow type used to build trajectory parameters.
            nV: Number of angle samples in the pose grid [count].
            nR: Number of radius samples in the pose grid [count].
            min_sine_freq: Minimum sine-sweep frequency [Hz].
            max_sine_freq: Maximum sine-sweep frequency [Hz].
            sine_freq_spacing: Frequency spacing between sweep points [Hz].
            num_sine_cycles: Number of cycles per sine frequency [count].
            dwell_btw_sine: Dwell time after each sine segment [s].
        Returns:
            `tuple[TrajParams, SystemIdParams]`:
            1) trajectory parameters for motion generation.
            2) identification parameters for sweep generation.
        Side Effects:
            None.
        Raises:
            None.
        Preconditions:
            The numeric inputs are expressed in the units expected by the trajectory generator.
        """
        traj_params = TrajParams(
            configuration=ctrl_config,
            max_displacement=max_disp,
            max_velocity=max_vel,
            max_acceleration=max_acc,
            sysid_type=sysid_type,
            single_pt_run_time=bcb_runtime,
        )
        sysid_params = SystemIdParams(
            nV=nV,
            nR=nR,
            min_freq=min_sine_freq,
            max_freq=max_sine_freq,
            freq_space=sine_freq_spacing,
            num_sine_cycles=num_sine_cycles,
            dwell_time=dwell_btw_sine,
        )
        return traj_params, sysid_params

    def _create_calibration_data_folder(self) -> str:
        """Create the root data folder path for one calibration run.

        Args:
            None.
        Returns:
            `str` root folder path for the current-date calibration run.
        Side Effects:
            None.
        Raises:
            None.
        Preconditions:
            None.
        """
        now = datetime.datetime.now()
        return f"{DATA_LOCATION_PREFIX}/{now.year}-{now.month}-{now.day}"

    def _store_calibration_parameters(
        self,
        traj_params: TrajParams,
        sysid_params: SystemIdParams,
        axes_to_command: int,
        Ts: float,
        start_pose: int,
        start_axis: int,
        imu_to_tcp_x: float,
        imu_to_tcp_y: float,
        imu_to_tcp_z: float,
        data_folder: str,
        joint_controller_id_completed: bool,
    ) -> None:
        """Persist the top-level calibration metadata for one run.

        Args:
            traj_params: Trajectory-generation parameters for the run.
            sysid_params: Identification parameters for the run.
            axes_to_command: Number of commanded axes [count].
            Ts: Sample time [s].
            start_pose: Starting pose index for resumable workflows [count].
            start_axis: Starting joint index for commanded-axis block [count].
            imu_to_tcp_x: X component of IMU-to-TCP translation [m].
            imu_to_tcp_y: Y component of IMU-to-TCP translation [m].
            imu_to_tcp_z: Z component of IMU-to-TCP translation [m].
            data_folder: Output data folder for `identification_parameters.csv` [path string].
            joint_controller_id_completed: Whether joint-controller ID has completed for this run [flag].
        Returns:
            `None`.
        Side Effects:
            Writes `identification_parameters.csv` to disk.
        Raises:
            OSError: Propagated if the parameter file cannot be written.
        Preconditions:
            `data_folder` is writable.
        """
        tcp_payload = float(getattr(self.model, "tcp_payload", 0.0))
        tcp_payload_com = getattr(self.model, "tcp_payload_com", np.zeros(3))
        store_parameters_in_data_folder(
            traj_params=traj_params,
            sysid_params=sysid_params,
            axes_commanded=axes_to_command,
            num_joints=self.num_joints,
            sample_time=Ts,
            start_pose=start_pose,
            start_axis=start_axis,
            shoulder_len=self.side_length,
            base_height=self.initial_height,
            robot_name=self.name,
            data_folder=data_folder,
            tcp_payload=tcp_payload,
            tcp_payload_com_x=float(tcp_payload_com[0]),
            tcp_payload_com_y=float(tcp_payload_com[1]),
            tcp_payload_com_z=float(tcp_payload_com[2]),
            imu_to_tcp_x=imu_to_tcp_x,
            imu_to_tcp_y=imu_to_tcp_y,
            imu_to_tcp_z=imu_to_tcp_z,
            joint_controller_id_completed=joint_controller_id_completed,
        )

    def _run_shaper_calibration(
        self,
        Ts: float,
        axes_to_command: int,
        traj_params: TrajParams,
        sysid_params: SystemIdParams,
        v_list: np.ndarray,
        r_list: np.ndarray,
        start_pose: int,
        start_axis: int,
        data_folder: str,
        joint_controller: "JointInversionController | None" = None,
    ) -> str:
        """Run the existing pose-grid sine-sweep calibration workflow.

        Args:
            Ts: Sample time [s].
            axes_to_command: Number of commanded axes [count].
            traj_params: Trajectory parameters for sine-sweep generation.
            sysid_params: Identification parameters for the sine sweep.
            v_list: Array of angle coordinates for the pose grid [rad].
            r_list: Array of radius coordinates for the pose grid [m].
            start_pose: Starting pose index for resumable runs [count].
            start_axis: Starting joint index for commanded-axis block [count].
            data_folder: Output folder for recorded calibration data [path string].
            joint_controller: Optional joint compensation controller for generating compensated sweeps.
        Returns:
            `str` root calibration data folder.
        Side Effects:
            Commands robot motion, records sweep data, and writes CSV files to disk.
        Raises:
            RuntimeError: Propagated if robot motion or recording fails.
        Preconditions:
            The robot is connected and the calibration folder is writable.
        """
        reach_sign = np.sign(self.robot_home[self.reach_axis])
        height_sign = np.sign(self.robot_home[self.height_axis])
        depth_sign = np.sign(self.robot_home[self.depth_axis])
        base_angle = self.__get_joint_positions()[0]
        current_pose = self.__get_tcp_pose()
        trajectory = Trajectory(traj_params, sysid_params)
        run_index = 0

        # Each loop iteration visits one pose in the V/R grid and records one
        # sine sweep per commanded axis from that pose.
        for angle_index in range(sysid_params.nV):
            for radius_index in range(sysid_params.nR):
                if run_index < start_pose:
                    run_index += 1
                    continue
                v_value = v_list[angle_index]
                r_value = r_list[radius_index]
                length_value, depth_value, height_value = polar_to_cartesian(
                    v=v_value,
                    r=r_value,
                    side_arm=self.side_length,
                    base_height=self.initial_height,
                    base_angle=base_angle,
                )
                target_xyz = np.zeros(3)
                target_xyz[self.reach_axis] = length_value * reach_sign
                target_xyz[self.depth_axis] = depth_value * depth_sign
                target_xyz[self.height_axis] = height_value * height_sign

                for move_axis in range(start_axis, start_axis + axes_to_command):
                    self.move_point_to_point_xyz(current_pose, target_xyz.tolist())
                    current_q = self.__get_joint_positions()
                    trajectory.generate_sine_sweep_trajectory(current_q, move_axis, Ts)
                    original_positions = np.asarray(
                        trajectory.positionTrajectory, dtype=float
                    )
                    if joint_controller is not None:
                        # This function mutates 'trajectory' in-place to apply the compensation to the position trajectory,
                        joint_controller.compensate_axis_sine(
                            trajectory=trajectory,
                            move_axis=move_axis,
                        )
                        compensated_positions = np.asarray(
                            trajectory.positionTrajectory, dtype=float
                        )
                        trajectory.velocityTrajectory = np.gradient(
                            compensated_positions, Ts, axis=0
                        ).tolist()
                        trajectory.accelerationTrajectory = np.gradient(
                            np.asarray(trajectory.velocityTrajectory, dtype=float),
                            Ts,
                            axis=0,
                        ).tolist()
                    self._execute_and_store_trajectory(
                        Ts=Ts,
                        trajectory=trajectory,
                        current_q=current_q,
                        run_index=run_index,
                        move_axis=move_axis,
                        data_folder=data_folder,
                        static_v=v_value,
                        static_r=r_value,
                        logged_position_stream=original_positions,
                    )
                run_index += 1
        return data_folder

    def _run_feedforward_calibration(
        self,
        Ts: float,
        axes_to_command: int,
        traj_params: TrajParams,
        sysid_params: SystemIdParams,
        v_list: np.ndarray,
        r_list: np.ndarray,
        data_folder: str,
        start_pose: int,
        start_axis: int,
        imu_to_tcp_xyz: Sequence[float],
    ) -> str:
        """Run the joint-controller ID phase followed by compensated sine sweeps.

        Args:
            Ts: Sample time [s].
            axes_to_command: Number of commanded axes [count].
            traj_params: Trajectory parameters.
            sysid_params: Identification parameters reused for the controller-ID sine sweeps.
            v_list: Array of angle coordinates for the pose grid [rad].
            r_list: Array of radius coordinates for the pose grid [m].
            data_folder: Root calibration data folder [path string].
            start_pose: Starting pose index recorded in top-level metadata [count].
            start_axis: Starting joint index for commanded-axis block [count].
            imu_to_tcp_xyz: IMU-to-TCP translation vector `[x, y, z]` [m].
        Returns:
            `str` root calibration data folder.
        Side Effects:
            Records joint-controller ID data, builds a joint compensation controller,
            and commands robot sine sweeps.
        Raises:
            RuntimeError: Propagated if motion, compensation, or recording fails.
        Preconditions:
            The robot is connected and the requested fixed joint poses have one value per joint.
        """

        joint_id_folder = f"{data_folder}/joint_controller_id"
        joint_controller = self._collect_joint_controller_id_data(
            Ts=Ts,
            axes_to_command=axes_to_command,
            traj_params=traj_params,
            sysid_params=sysid_params,
            joint_id_folder=joint_id_folder,
            start_axis=start_axis,
            first_v=v_list[0],
            first_r=r_list[0],
            imu_to_tcp_xyz=imu_to_tcp_xyz,
        )
        self._run_shaper_calibration(
            Ts=Ts,
            axes_to_command=axes_to_command,
            traj_params=traj_params,
            sysid_params=sysid_params,
            start_axis=start_axis,
            v_list=v_list,
            r_list=r_list,
            start_pose=start_pose,
            joint_controller=joint_controller,
            data_folder=data_folder,
        )
        return data_folder

    def _collect_joint_controller_id_data(
        self,
        Ts: float,
        axes_to_command: int,
        traj_params: TrajParams,
        sysid_params: SystemIdParams,
        joint_id_folder: str,
        start_axis: int,
        first_v: float,
        first_r: float,
        imu_to_tcp_xyz: Sequence[float],
    ) -> "JointInversionController":
        """Collect sine-sweep data for joint-controller identification and return the controller.

        Args:
            Ts: Sample time [s].
            axes_to_command: Number of commanded axes [count].
            traj_params: Parent calibration trajectory parameters.
            sysid_params: Identification parameters used for the ID sine sweeps.
            joint_id_folder: Output folder for joint-controller ID data [path string].
            start_axis: Starting joint index for commanded-axis block [count].
            first_v: Angle coordinate for the first pose used in joint-controller ID [rad].
            first_r: Radius coordinate for the first pose used in joint-controller ID [m].
            imu_to_tcp_xyz: IMU-to-TCP translation vector `[x, y, z]` [m].
        Returns:
            `JointInversionController` loaded from the recorded ID dataset.
        Side Effects:
            Writes joint-controller ID metadata and motion files to disk and may run local model generation.
        Raises:
            RuntimeError: Propagated if controller generation fails.
            OSError: Propagated if the ID data cannot be written.
        Preconditions:
            The robot is connected and `joint_id_folder` is writable.
        """
        from reforge_core.control.python.joint_control import JointInversionController

        # Codex: force the joint-controller ID phase to use `shaper` metadata even
        # when the parent calibration is `feedforward`, because the downstream model
        # fitter expects a sine-sweep dataset in the `joint_controller_id` folder.
        joint_id_traj_params = TrajParams(
            configuration=traj_params.configuration,
            max_displacement=traj_params.max_displacement,
            max_velocity=traj_params.max_velocity,
            max_acceleration=traj_params.max_acceleration,
            sysid_type="shaper",
            single_pt_run_time=traj_params.single_pt_run_time,
            output_sensor="JointPos",
        )
        # Keep joint-ID sweeps inside the measurable band without mutating the
        # shared params object used by other calibration stages.
        joint_sysid_params = self._clone_sysid_params(
            sysid_params=sysid_params,
            max_freq_override=2 * MAX_ROBOT_JOINTS_BANDWIDTH,
            nv_override=1,  # [count]
            nr_override=1,  # [count; not used for sine
        )

        # Move to the first pose so the joint-controller ID sweeps are seeded from a consistent configuration.
        first_target_xyz = polar_to_cartesian(
            v=first_v,
            r=first_r,
            side_arm=self.side_length,
            base_height=self.initial_height,
            base_angle=self.__get_joint_positions()[0],
        )
        self.move_point_to_point_xyz(
            current_pose=self.__get_tcp_pose(),
            target_xyz=list(first_target_xyz),
        )
        current_q = self.__get_joint_positions()
        # Store the joint-controller ID metadata in a dedicated subfolder
        self._store_calibration_parameters(
            traj_params=joint_id_traj_params,
            sysid_params=joint_sysid_params,
            axes_to_command=axes_to_command,
            Ts=Ts,
            start_pose=0,  # we hardcode above to use the first pose
            start_axis=start_axis,
            imu_to_tcp_x=float(imu_to_tcp_xyz[0]),
            imu_to_tcp_y=float(imu_to_tcp_xyz[1]),
            imu_to_tcp_z=float(imu_to_tcp_xyz[2]),
            data_folder=joint_id_folder,
            joint_controller_id_completed=True,
        )
        pose_indices = list(range(len(current_q)))
        if self._has_pose_axis_dataset(
            data_folder=joint_id_folder,
            pose_indices=pose_indices,
            start_axis=start_axis,
            axes_to_command=axes_to_command,
        ):
            print(
                "[JointID] Found existing joint_controller_id files. "
                "Skipping sine sweep acquisition and reusing recorded pose/axis data."
            )
        else:
            trajectory = Trajectory(joint_id_traj_params, joint_sysid_params)
            # Record the controller-ID sweeps from the first pose to
            # get the motor dynamics
            self.move_to_joint(tuple(current_q))
            for move_axis in range(start_axis, start_axis + axes_to_command):
                current_joint_positions = self.__get_joint_positions()
                trajectory.generate_sine_sweep_trajectory(
                    current_joint_positions, move_axis, Ts
                )
                self._execute_and_store_trajectory(
                    Ts=Ts,
                    trajectory=trajectory,
                    current_q=current_joint_positions,
                    run_index=0,
                    move_axis=move_axis,
                    data_folder=joint_id_folder,
                    static_v=first_v,
                    static_r=first_r,
                )
        return JointInversionController(
            reforge_api_token=self.reforge_api_token,
            reforge_robot_id=self.id,
            data_folder=joint_id_folder,
            num_axes=axes_to_command,
            robot_name=self.name,
            num_joints=self.num_joints,
            robot_model=self.model,
            Ts=Ts,
            sParams=joint_sysid_params,
            tParams=joint_id_traj_params,
            axis_indices=list(range(start_axis, start_axis + axes_to_command)),
            saved_dynamics=False,
        )

    def _clone_sysid_params(
        self,
        sysid_params: SystemIdParams,
        max_freq_override: float | None = None,
        nv_override: int | None = None,
        nr_override: int | None = None,
    ) -> SystemIdParams:
        """Clone system-ID sweep parameters with an optional max-frequency override.
        Args:
            sysid_params: Source sweep parameters.
            max_freq_override: Optional max sweep frequency override [Hz].
        Returns:
            `SystemIdParams` independent copy.
        Side Effects:
            None.
        Raises:
            None.
        Preconditions:
            `sysid_params` fields are initialized.
        """
        dwell_value = (
            float(sysid_params.dwell)
            if hasattr(sysid_params, "dwell")
            else (float(DEFAULT_DWELL_TIME))
        )
        return SystemIdParams(
            nV=(int(nv_override) if nv_override is not None else int(sysid_params.nV)),
            nR=(int(nr_override) if nr_override is not None else int(sysid_params.nR)),
            min_freq=float(sysid_params.min_freq),
            max_freq=(
                float(max_freq_override)
                if max_freq_override is not None
                else float(sysid_params.max_freq)
            ),
            freq_space=float(sysid_params.freq_space),
            num_sine_cycles=int(sysid_params.num_sine_cycles),
            dwell_time=dwell_value,
        )

    def _has_pose_axis_dataset(
        self,
        data_folder: str,
        pose_indices: Sequence[int],
        start_axis: int,
        axes_to_command: int,
    ) -> bool:
        """Check whether motion/static files exist for all requested poses and commanded axes.
        Args:
            data_folder: Data folder containing motion/static run files [path string].
            pose_indices: Pose indices that must be present in the folder [count list].
            start_axis: First commanded axis index [count].
            axes_to_command: Number of commanded axes [count].
        Returns:
            `bool` true when both motion and static CSV files exist for every
            requested pose/axis combination.
        Side Effects:
            Reads file existence from disk.
        Raises:
            None.
        Preconditions:
            `data_folder` may or may not already exist.
        """
        if not os.path.isdir(data_folder):
            return False
        for pose_index in pose_indices:
            for axis in range(start_axis, start_axis + axes_to_command):
                motion_file = (
                    f"{data_folder}/robotData_motion_pose{pose_index}_axis{axis}.csv"
                )
                static_file = (
                    f"{data_folder}/robotData_static_pose{pose_index}_axis{axis}.csv"
                )
                if not (os.path.isfile(motion_file) and os.path.isfile(static_file)):
                    return False
        return True

    def _execute_and_store_trajectory(
        self,
        Ts: float,
        trajectory: Trajectory,
        current_q: list[float],
        run_index: int,
        move_axis: int,
        data_folder: str,
        static_v: float,
        static_r: float,
        compensation_enabled: bool = False,
        logged_position_stream: Sequence[Sequence[float]] | np.ndarray | None = None,
    ) -> None:
        """Execute one trajectory, capture recorder data, and persist the run.

        Args:
            Ts: Sample time [s].
            trajectory: Trajectory to execute and record.
            current_q: Joint-space configuration used for inertia evaluation [rad].
            run_index: Pose index or fixed-pose index used in the file name [count].
            move_axis: Zero-based active axis index used in the file name [count].
            data_folder: Output folder for the recorded run [path string].
            static_v: Calibration pose angle stored in the static file [rad].
            static_r: Calibration pose radius stored in the static file [m].
            compensation_enabled: Whether the executed command was compensation-modified [flag].
            logged_position_stream: Optional position stream to save as the
                commanded input instead of the executed trajectory [rad].
        Returns:
            `None`.
        Side Effects:
            Mutates `self.recorder`, executes robot motion, and writes motion/static CSV files.
        Raises:
            RuntimeError: Propagated if the robot motion or recorder pipeline fails.
            OSError: Propagated if the output files cannot be written.
        Preconditions:
            `trajectory` buffers are populated and `data_folder` is writable.
        """
        self.recorder.reset()
        current_mass_matrix = self.model.get_mass_matrix(joint_angles=current_q)
        current_mass_diagonals = np.zeros((self.num_joints))

        # Store the diagonal inertia terms once per run so the offline fitting
        # stage can pair every recorded trace with the local robot inertia.
        for joint_num in range(self.num_joints):
            current_mass_diagonals[joint_num] = current_mass_matrix[joint_num][
                joint_num
            ]
        self.rt_periodic_task(
            Ts=Ts,
            trajectory=trajectory,
            logged_position_stream=logged_position_stream,
        )
        self.recorder.outputMassDiagonals = current_mass_diagonals.tolist()
        self.recorder.endIndices = trajectory.endIndices.copy()
        self.recorder.inputV.append(static_v)
        self.recorder.inputR.append(static_r)
        self.recorder.compensationEnabled = compensation_enabled
        store_recorder_data_in_data_folder(
            recorder=self.recorder,
            run_index=run_index,
            move_axis=move_axis,
            data_folder=data_folder,
        )

    def initialize_model_from_urdf(
        self,
        urdf_path: str,
        tcp_payload: float | None = DEFAULT_TCP_PAYLOAD,
        tcp_payload_com: Sequence[float] | None = None,
    ) -> None:
        """Initialize the robot dynamics model from a URDF file.

        Args:
            urdf_path: Path to the URDF file.

        Returns:
            `None`.

        Side Effects:
            Loads the dynamics model and updates `self.model` and flags.

        Raises:
            FileNotFoundError: If the URDF file does not exist.
            RuntimeError: If the dynamics model cannot be constructed.

        Preconditions:
            The URDF file is accessible on disk.
        """
        print(
            f"Loading robot's kinematic and dynamic models from URDF: {self.urdf_path}"
        )

        # Load the robot model from the URDF file
        self.model = Dynamics(
            urdf_file=urdf_path,
            initialize=True,
            tcp_payload=tcp_payload,
            tcp_payload_com=tcp_payload_com,
        )
        self.model_is_loaded = True

        print("Robot model loaded successfully.")

    def move_home(self, home_sign: int = 1, joint_move: bool = True) -> None:
        """Move the robot to the home position.

        Args:
            home_sign: Sign of the shoulder joint angle at home.
            joint_move: If True, move using joint angles; otherwise use pose.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion and updates cached home properties.

        Raises:
            RuntimeError: If robot motion fails.

        Preconditions:
            Robot connection is active.
        """
        print("Moving to home position...")
        if joint_move:
            self.move_home_joint(home_sign=home_sign)
        else:
            self.move_home_pose()
        print("Home position reached.")

        # Store the home x-, y-, and z-axis (and r,p,y) parameters of the robot
        self.robot_home = (
            self.__get_tcp_pose() if HOME_POSE_OVERRIDE is None else HOME_POSE_OVERRIDE
        )
        self.home_xyz = np.array(self.robot_home, dtype=np.float32)[0:3]

        self.max_reach = max(
            [abs(self.robot_home[0]), abs(self.robot_home[1]), abs(self.robot_home[2])]
        )
        self.initial_height = self.home_xyz[2]

        self.reach_axis = np.where(np.abs(self.home_xyz) == self.max_reach)[0][0]
        self.height_axis = np.where(np.abs(self.home_xyz) == self.initial_height)[0][0]
        self.depth_axis = np.where(
            np.logical_and(
                np.not_equal(np.abs(self.home_xyz), self.initial_height),
                np.not_equal(np.abs(self.home_xyz), self.max_reach),
            )
        )[0][0]

        self.side_length = abs(self.robot_home[self.depth_axis])

    def move_home_pose(self) -> None:
        """Move the robot to the configured home pose.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            RuntimeError: If robot motion fails.

        Preconditions:
            Home pose is configured and robot connection is active.
        """
        self.move_to_pose(HOME_QUAT, HOME_XYZ)

    def move_home_joint(self, home_sign: int = 1) -> None:
        """Move the robot to the home joint configuration.

        Args:
            home_sign: Sign for the shoulder joint angle.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            RuntimeError: If robot motion fails.

        Preconditions:
            Robot connection is active.
        """
        home_joints = [j for j in HOME_JOINTS]
        home_joints[1] = home_sign * home_joints[1]
        self.move_to_joint(target_joint=tuple(home_joints))

    def move_point_to_point_xyz(
        self, current_pose: List[float], target_xyz: List[float]
    ) -> None:
        """Move the robot to a Cartesian position while keeping orientation.

        Args:
            current_pose: Current pose [x, y, z, qx, qy, qz, qw].
            target_xyz: Target position [x, y, z].

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            RuntimeError: If current pose does not include orientation.

        Preconditions:
            Robot connection is active.
        """
        # Check if the orientation is included in the current pose
        if len(current_pose) < self.pose_length:
            raise RuntimeError(
                "Not enough elements in currentPose to get the orientation \
                                of the current pose."
            )

        # Get quaternion from current pose
        target_quat = current_pose[3:]

        # Move to pose
        self.move_to_pose(target_quat, target_xyz)

    def process_motion_data(self, entry: Dict) -> None:
        """Process a single motion data entry into the recorder.

        Args:
            entry: Dictionary containing motion data fields.

        Returns:
            `None`.

        Side Effects:
            Appends data to `self.recorder` buffers.

        Raises:
            KeyError: If required fields are missing from `entry`.

        Preconditions:
            `entry` follows the expected data log schema.
        """
        # entry: {'cmd_time','input_positions','output_positions',
        #       'velocities','efforts','imu_time','linear_acceleration',
        #       'angular_velocity','orientation'}
        self.recorder.servoTime.append(entry["cmd_time"])
        self.recorder.inputJointPositions.append(list(entry["input_positions"]))
        self.recorder.outputJointPositions.append(entry["output_positions"])
        self.recorder.outputCurrents.append(entry["efforts"])
        self.recorder.imuTime.append(entry["imu_time"])
        self.recorder.outputTcpAccelerations.append(
            entry["linear_acceleration"] + entry["angular_velocity"]
        )
        self.recorder.quaternionTime.append(entry["imu_time"])
        self.recorder.quaternion.append(entry["orientation"])

    def rt_periodic_task(
        self,
        Ts: float,
        trajectory: Trajectory,
        logged_position_stream: Sequence[Sequence[float]] | np.ndarray | None = None,
    ) -> None:
        """Publish a trajectory in real time and record data.

        Args:
            Ts: Sampling time [s].
            trajectory: Trajectory containing position/velocity/acceleration data.
            logged_position_stream: Optional position stream to store in the
                recorder as the commanded input instead of the executed
                position stream [rad].

        Returns:
            `None`.

        Side Effects:
            Commands robot motion and appends data to `self.recorder`.

        Raises:
            RuntimeError: If publishing or recording fails.

        Preconditions:
            Robot connection is active and `trajectory` is populated.
        """
        # Collect trajectory data into lists
        position_data = trajectory.positionTrajectory  # list
        velocity_data = trajectory.velocityTrajectory  # list
        acceleration_data = trajectory.accelerationTrajectory  # list
        time_data = trajectory.trajectoryTime  # list

        # Publish joint positions to the robot and record data
        data_log = self.publish_and_record_joint_positions(
            time_data=time_data,
            position_stream=position_data,
            velocity_stream=velocity_data,
            acceleration_stream=acceleration_data,
            Ts=Ts,
        )

        logged_positions = None
        if logged_position_stream is not None:
            logged_positions = np.asarray(logged_position_stream, dtype=float)
            if logged_positions.shape != np.asarray(position_data, dtype=float).shape:
                raise ValueError(
                    "logged_position_stream must match the executed trajectory "
                    "shape."
                )
        # Reset recorder for storing data
        self.recorder.reset()

        # Data post processing -- adding data to recorder
        sample_index = 0
        while (
            data_log
        ):  # {'cmd_time','input_positions','output_positions','velocities','efforts','imu_time','linear_acceleration','angular_velocity','orientation'}
            log_entry = data_log.popleft()
            if logged_positions is not None:
                log_entry["input_positions"] = logged_positions[sample_index].tolist()
            self.process_motion_data(entry=log_entry)
            sample_index += 1

    # END OF PRE-DEFINED METHODS

    # OPTIONAL METHODS
    def spin_thread(self, node: Any) -> None:
        """Spin a ROS node in a background thread.

        Args:
            node: ROS node to spin.

        Returns:
            `None`.

        Side Effects:
            Runs a ROS event loop.

        Raises:
            None.

        Preconditions:
            ROS is initialized.
        """
        from robot.ros_manager import rclpy

        rclpy.spin(node=node)
