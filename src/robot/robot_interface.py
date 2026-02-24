# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 1.0

import datetime
import numpy as np
from collections import deque
from typing import Any, Deque, List, Dict, Tuple, Sequence
from importlib.resources import files, as_file

from robot.robot_base import Robot, DataRecorder
from robot.robot_base import (
    get_polar_coordinates,
    store_parameters_in_data_folder,
    store_recorder_data_in_data_folder,
)

from reforge_core.util.utility import TrajParams, SystemIdParams, polar_to_cartesian
from reforge_core.util.trajectory import Trajectory
from reforge_core.util.robot_dynamics import Dynamics

from robot.robot_base import (
    DEFAULT_MAX_DISP,
    DEFAULT_MAX_VEL,
    DEFAULT_MAX_ACC,
    DEFAULT_SYSID_ANGLES,
    DEFAULT_SYSID_RADII,
    DEFAULT_HOME_SIGN,
    DEFAULT_FIRST_POSE,
    DEFAULT_TCP_PAYLOAD,
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
)

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

                self.num_joints = len(self._get_joint_positions())
                self.pose_length = len(self._get_tcp_pose())
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
    def _get_joint_positions(self) -> List:
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

    def _get_tcp_pose(self) -> List:
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
        sysid_type: str = DEFAULT_SYSID_TYPE,
        nV: int = DEFAULT_SYSID_ANGLES,
        nR: int = DEFAULT_SYSID_RADII,
        min_sine_freq: float = DEFAULT_SINE_MIN_FREQ,
        max_sine_freq: float = DEFAULT_SINE_MAX_FREQ,
        sine_freq_spacing: float = DEFAULT_FREQ_SPACING,
        num_sine_cycles: int = DEFAULT_SINE_CYCLES,
        dwell_btw_sine: float = DEFAULT_DWELL_TIME,
        start_pose: int = DEFAULT_FIRST_POSE,
        home_sign: int = DEFAULT_HOME_SIGN,
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
            sysid_type: System identification type (`bcb` or `sine`).
            nV: Number of angle positions.
            nR: Number of radius positions.
            min_sine_freq: Minimum sine sweep frequency [Hz].
            max_sine_freq: Maximum sine sweep frequency [Hz].
            sine_freq_spacing: Frequency spacing [Hz].
            num_sine_cycles: Number of sine cycles per pose.
            dwell_btw_sine: Dwell time between sweeps [s].
            start_pose: Starting pose index.
            home_sign: Sign of shoulder joint angle at home position.

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

        # Initialize parameters for robot trajectory generation
        # ==================================================================
        # Move the robot to the home position
        self.move_home(home_sign=home_sign)

        # Store sign information for each Cartesian axis
        reach_sign = np.sign(self.robot_home[self.reach_axis])
        height_sign = np.sign(self.robot_home[self.height_axis])
        depth_sign = np.sign(self.robot_home[self.depth_axis])

        # Create trajectory and system ID parameters
        t_params = TrajParams(
            configuration=ctrl_config,
            max_displacement=max_disp,
            max_velocity=max_vel,
            max_acceleration=max_acc,
            sysid_type=sysid_type,
            single_pt_run_time=bcb_runtime,
        )

        s_params = SystemIdParams(
            nV=nV,
            nR=nR,
            min_freq=min_sine_freq,
            max_freq=max_sine_freq,
            freq_space=sine_freq_spacing,
            num_sine_cycles=num_sine_cycles,
            dwell_time=dwell_btw_sine,
        )

        # Generate V (angles from horizontal) and R (radius from base) positions
        # in base frame
        V, R = get_polar_coordinates(s_params.nV, s_params.nR, self.max_reach)

        # Get starting joint positions - need base joint angle
        # for polar to cartesian computation
        initial_q = self._get_joint_positions()
        base_angle = initial_q[0]

        # Initialize system ID variables for experiment
        # ===============================================================
        # Current x-, y-, and z-axis (and r,p,y) location of the robot
        current_pose = self._get_tcp_pose()

        # Create a trajectory with the trajectory and system id parameters
        trajectory = Trajectory(t_params, s_params)

        # Keep track of the number of runs (positions visited) and use to store values
        run_index = 0

        # Keep track of the inertia diagonals to store on each run
        current_mass_diagonals = np.zeros((self.num_joints))

        # Run system identification on robot through all positions
        # ==============================================================================================
        # Create directory name where data will be stored
        now = datetime.datetime.now()
        data_folder = f"{DATA_LOCATION_PREFIX}/{now.year}-{now.month}-{now.day}"

        # Store trajectory and system ID parameters in data folder
        tcp_payload = float(getattr(self.model, "tcp_payload", 0.0))
        tcp_payload_com = getattr(self.model, "tcp_payload_com", np.zeros(3))
        store_parameters_in_data_folder(
            traj_params=t_params,
            sysid_params=s_params,
            axes_commanded=axes_to_command,
            num_joints=self.num_joints,
            sample_time=Ts,
            start_pose=start_pose,
            shoulder_len=self.side_length,
            base_height=self.initial_height,
            robot_name=self.name,
            data_folder=data_folder,
            tcp_payload=tcp_payload,
            tcp_payload_com_x=float(tcp_payload_com[0]),
            tcp_payload_com_y=float(tcp_payload_com[1]),
            tcp_payload_com_z=float(tcp_payload_com[2]),
        )

        # Loop through angles and radii
        for i in range(s_params.nV):
            for j in range(s_params.nR):
                # Skip until we reach start_pose
                if run_index < start_pose:
                    run_index += 1
                    continue

                # Determine length, width, and height positions from polar coordinates
                v = V[i]
                r = R[j]
                l, d, h = polar_to_cartesian(
                    v=v,
                    r=r,
                    side_arm=self.side_length,
                    base_height=self.initial_height,
                    base_angle=base_angle,
                )

                new_xyz = np.zeros(3)
                new_xyz[self.reach_axis] = l * reach_sign
                new_xyz[self.depth_axis] = d * depth_sign
                new_xyz[self.height_axis] = h * height_sign

                for move_axis in range(axes_to_command):
                    # Clear the recorder for starting a new run
                    self.recorder.reset()

                    # Move to the initial position with point-to-point pose motion
                    self.move_point_to_point_xyz(current_pose, new_xyz.tolist())

                    # Get the initial joint positions after the robot move
                    current_q = self._get_joint_positions()

                    # Compute the current mass matrix and update current mass diagonals
                    current_mass_matrix = self.model.get_mass_matrix(
                        joint_angles=current_q
                    )  # numpy matrix
                    for joint_num in range(self.num_joints):
                        current_mass_diagonals[joint_num] = current_mass_matrix[
                            joint_num
                        ][joint_num]

                    # Generate system ID trajectory
                    trajectory.generate_sine_sweep_trajectory(current_q, move_axis, Ts)

                    # Start real-time joint control with trajectory
                    self.rt_periodic_task(Ts, trajectory)

                    # Store mass data
                    self.recorder.outputMassDiagonals = current_mass_diagonals.tolist()

                    # Store end indices data
                    self.recorder.endIndices = trajectory.endIndices.copy()

                    # R and V
                    self.recorder.inputV.append(v)
                    self.recorder.inputR.append(r)

                    # Save all recorder data in CSVs
                    store_recorder_data_in_data_folder(
                        recorder=self.recorder,
                        run_index=run_index,
                        move_axis=move_axis,
                        data_folder=data_folder,
                    )

                # Increment run index
                run_index += 1

        return data_folder

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
            self._get_tcp_pose() if HOME_POSE_OVERRIDE is None else HOME_POSE_OVERRIDE
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

    def rt_periodic_task(self, Ts: float, trajectory: Trajectory) -> None:
        """Publish a trajectory in real time and record data.

        Args:
            Ts: Sampling time [s].
            trajectory: Trajectory containing position/velocity/acceleration data.

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
        # Reset recorder for storing data
        self.recorder.reset()

        # Data post processing -- adding data to recorder
        while (
            data_log
        ):  # {'cmd_time','input_positions','output_positions','velocities','efforts','imu_time','linear_acceleration','angular_velocity','orientation'}
            log_entry = data_log.popleft()
            self.process_motion_data(entry=log_entry)

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