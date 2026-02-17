# src/util/Robot.py
# This module defines an abstract base class for robots, which can be extended by specific robot implementations
# to provide functionality for movement, command execution, and trajectory handling.

import csv
import math
import os
import numpy as np
from abc import ABC, abstractmethod
from typing import Deque, List, Dict, Tuple
from util.Utility import TrajParams, SystemIdParams
from util.Utility import (
    DEFAULT_CONFIG,
    DEFAULT_BCB_RUNTIME,
    DEFAULT_SYSID_TYPE,
    DEFAULT_SINE_MIN_FREQ,
    DEFAULT_SINE_MAX_FREQ,
    DEFAULT_FREQ_SPACING,
    DEFAULT_DWELL_TIME,
    DEFAULT_SINE_CYCLES,
)
from util.Trajectory import Trajectory


M_PI = math.pi
DEFAULT_MAX_DISP = M_PI / 18.0  # [rad]
DEFAULT_MAX_VEL = 18.0  # [rad/s]
DEFAULT_MAX_ACC = 2.0  # [rad/s^2]
DEFAULT_SYSID_ANGLES = (
    8 + 2
)  # number of joint angles (from the stretched horizontal position)
DEFAULT_SYSID_RADII = (
    4 * 2
)  # number of radii (from the center of the robot base to the end-effector)
DEFAULT_HOME_SIGN = 1  # sign of shoulder joint angle at home position
DEFAULT_FIRST_POSE = 0  # starting pose index for calibration (enables resuming calibration from a specific pose)
DEFAULT_AXES_COMMANDED = 3  # number of axes to command during calibration
PLACEHOLDER_IP = "sim"  # placeholder IP for systems not requiring IP addresses
DEFAULT_ROBOT_FREQ = 200  # [Hz] - Default frequency for robot control

# Other constants - edit as necessary
DEFAULT_MIN_CALIBRATION_ANGLE = 0.0  # [rad] - minimum angle from horizontal for the calibration pose with minimum angle
DEFAULT_MAX_CALIBRATION_ANGLE = (
    M_PI / 3
)  # [rad] - maximum angle from horizontal for the calibration pose with maximum angle
DEFAULT_MIN_CALIBRATION_RADIUS_SCALE = (
    0.4  # [%] of robot's max reach for the calibration pose with minimum radius
)
DEFAULT_MAX_CALIBRATION_RADIUS_SCALE = (
    0.8  # [%] of robot's max reach for the calibration pose with minimum radius
)


class DataRecorder:
    """Store time-series data collected during calibration and system ID.

    Args:
        None.

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        None.
    """

    def __init__(self) -> None:
        """Initialize empty data buffers for recording.

        Side Effects:
            Allocates empty lists for all data fields.

        Raises:
            None.

        Preconditions:
            None.
        """
        self.inputJointPositions: list[list[float]] = []
        self.outputJointPositions: list[list[float]] = []
        self.outputCurrents: list[list[float]] = []
        self.outputTcpAccelerations: list[list[float]] = []
        self.servoTime: list[float] = []
        self.imuTime: list[float] = []
        self.quaternionTime: list[float] = []
        self.quaternion: list[list[float]] = []

        # Static parameters
        self.outputMassDiagonals: list[list[float]] = []
        self.endIndices: list[int] = []
        self.inputV: list[float] = []
        self.inputR: list[float] = []

    def reset(self) -> None:
        """Clear all recorded data buffers.

        Returns:
            `None`.

        Side Effects:
            Reinitializes internal data lists.

        Raises:
            None.

        Preconditions:
            None.
        """
        self.inputJointPositions.clear()
        self.outputJointPositions.clear()
        self.outputCurrents.clear()
        self.outputTcpAccelerations.clear()
        self.servoTime.clear()
        self.imuTime.clear()
        self.quaternionTime.clear()
        self.quaternion.clear()
        self.outputMassDiagonals.clear()
        self.endIndices.clear()
        self.inputV.clear()
        self.inputR.clear()


class Robot(ABC):
    """Abstract base class defining the robot control interface.

    Args:
        name: Human-readable robot name.

    Side Effects:
        Initializes recorder and default joint/pose dimensions.

    Raises:
        None.

    Preconditions:
        None.
    """

    def __init__(self, name: str) -> None:
        """Initialize the robot base class.

        Args:
            name: Human-readable robot name.

        Side Effects:
            Creates a `DataRecorder` and sets default joint/pose sizes.

        Raises:
            None.

        Preconditions:
            None.
        """
        self.name = name
        self.recorder = DataRecorder()
        # Number of joints in the robot
        # default [j0, j1, j2, j3, j4, j5]
        self.num_joints = 6
        # Pose vector length
        # default [x, y, z, qx, qy, qz, qw]
        self.pose_length = 7

    @property
    @abstractmethod
    def in_sim_mode(self) -> bool:
        """Return whether the robot is in simulation mode.

        Returns:
            `bool` indicating simulation mode.

        Side Effects:
            None.

        Raises:
            None.

        Preconditions:
            None.
        """
        pass

    @property
    @abstractmethod
    def urdf_path(self) -> str:
        """Return the URDF path for the robot.

        Returns:
            `str` path to the URDF file.

        Side Effects:
            None.

        Raises:
            None.

        Preconditions:
            None.
        """
        pass

    # Required robot methods to be implemented by subclasses
    @abstractmethod
    def move_to_joint(self, target_joint: Tuple[float, ...]) -> None:
        """Move the robot to the specified joint positions.

        Args:
            target_joint: Joint positions in radians.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    @abstractmethod
    def move_to_pose(self, target_quat: List[float], target_xyz: List[float]) -> None:
        """Move the robot to the specified Cartesian pose.

        Args:
            target_quat: Quaternion [qx, qy, qz, qw].
            target_xyz: Position [x, y, z].

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    @abstractmethod
    def publish_and_record_joint_positions(
        self,
        time_data: List,
        position_stream: List,
        velocity_stream: List = [],
        acceleration_stream: List = [],
        Ts: float = 1 / DEFAULT_ROBOT_FREQ,
    ) -> Deque[Dict]:
        """Publish joint trajectories and record data.

        Args:
            time_data: Time stamps for each command.
            position_stream: Joint position commands (N x num_joints).
            velocity_stream: Joint velocity commands (N x num_joints).
            acceleration_stream: Joint acceleration commands (N x num_joints).
            Ts: Sampling time [s].

        Returns:
            `collections.deque[dict]` log of recorded data entries.

        Side Effects:
            Commands robot motion and records data.

        Raises:
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    # Pre-defined robot methods - do not modify
    @abstractmethod
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
        start_pose: int = 0,
        home_sign: int = 1,
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
            start_pose: Starting pose index.
            home_sign: Sign of shoulder joint angle at home position.

        Returns:
            `str` folder where calibration data is stored.

        Side Effects:
            Commands robot motion and writes calibration data to disk.

        Raises:
            None.

        Preconditions:
            Robot connection is active and data directory is writable.
        """
        pass

    @abstractmethod
    def initialize_model_from_urdf(self, urdf_path: str) -> None:
        """Load the robot kinematic and dynamic model from a URDF file.

        Args:
            urdf_path: Path to the URDF file.

        Returns:
            `None`.

        Side Effects:
            Loads a dynamics model and updates internal state.

        Raises:
            FileNotFoundError: If the URDF file does not exist.
            RuntimeError: If the dynamics model cannot be constructed.

        Preconditions:
            URDF file is accessible.
        """
        pass

    @abstractmethod
    def move_home(self, home_sign: int = 1, joint_move: bool = True) -> None:
        """Move the robot to the home position.

        Args:
            home_sign: Sign of the shoulder joint angle.
            joint_move: If True, move using joint angles; otherwise use pose.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    @abstractmethod
    def move_home_joint(self, home_sign: int = 1) -> None:
        """Move the robot to the home joint configuration.

        Args:
            home_sign: Sign of the shoulder joint angle.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    @abstractmethod
    def move_home_pose(self) -> None:
        """Move the robot to the configured home pose.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion.

        Raises:
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    @abstractmethod
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
            None.

        Preconditions:
            Robot connection is active.
        """
        pass

    @abstractmethod
    def process_motion_data(self, entry: Dict) -> None:
        """Process a single motion data entry into the recorder.

        Args:
            entry: Dictionary containing motion data fields.

        Returns:
            `None`.

        Side Effects:
            Appends data to a recorder implementation.

        Raises:
            None.

        Preconditions:
            `entry` follows the expected data log schema.
        """
        pass

    @abstractmethod
    def rt_periodic_task(self, Ts: float, trajectory: Trajectory) -> None:
        """Publish a trajectory in real time and record data.

        Args:
            Ts: Sampling time [s].
            trajectory: Trajectory containing position/velocity/acceleration data.

        Returns:
            `None`.

        Side Effects:
            Commands robot motion and records data.

        Raises:
            None.

        Preconditions:
            Robot connection is active and `trajectory` is populated.
        """
        pass


# -----------------------
# Utilites
# -----------------------
def get_polar_coordinates(
    num_angles: int,
    num_radii: int,
    max_reach: float,
    min_angle: float = DEFAULT_MIN_CALIBRATION_ANGLE,
    max_angle: float = DEFAULT_MAX_CALIBRATION_ANGLE,
    min_radius_scale: float = DEFAULT_MIN_CALIBRATION_RADIUS_SCALE,
    max_radius_scale: float = DEFAULT_MAX_CALIBRATION_RADIUS_SCALE,
) -> Tuple[np.ndarray, np.ndarray]:
    """Generate polar coordinates (R and V) grid for calibration poses.

    Args:
        num_angles: Number of angle samples.
        num_radii: Number of radius samples.
        max_reach: Maximum reach of the robot [m].
        min_angle: Minimum angle from horizontal [rad].
        max_angle: Maximum angle from horizontal [rad].
        min_radius_scale: Minimum radius scale (fraction of max reach).
        max_radius_scale: Maximum radius scale (fraction of max reach).

    Returns:
        `tuple[np.ndarray, np.ndarray]` containing angle list `V` [rad] and
        radius list `R` [m].

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        `num_angles` and `num_radii` are positive.
    """
    V = np.linspace(min_angle, max_angle, num_angles).tolist()
    R = np.linspace(
        min_radius_scale * max_reach, max_radius_scale * max_reach, num_radii
    ).tolist()
    return V, R


def store_recorder_data_in_data_folder(
    recorder: DataRecorder, run_index: int, move_axis: int, data_folder: str
) -> None:
    """Persist recorder data to motion and static CSV files.

    Args:
        recorder: DataRecorder containing recorded streams.
        run_index: Pose index for file naming.
        move_axis: Axis index for file naming.
        data_folder: Output directory for CSV files.

    Returns:
        `None`.

    Side Effects:
        Creates directories and writes CSV files to disk.

    Raises:
        OSError: If files cannot be created or written.

    Preconditions:
        `recorder` contains at least one recorded sample.
    """
    # Motion parameters
    motion_headers = [
        "index",
        "servo_period",
        "servo_timestamp",
        *[f"j{i}_pos_cmd" for i in range(len(recorder.inputJointPositions[0]))],
        *[f"j{i}_pos_enc" for i in range(len(recorder.inputJointPositions[0]))],
        *[f"j{i}_current" for i in range(len(recorder.inputJointPositions[0]))],
        "imu_timestamp",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "quat_timestamp",
        "w",
        "x",
        "y",
        "z",
    ]

    motion_data_rows = []
    for i in range(len(recorder.servoTime)):
        row = [
            i,
            recorder.servoTime[i] - recorder.servoTime[i - 1] if i > 0 else 0.0,
            recorder.servoTime[i],
            *recorder.inputJointPositions[i],
            *recorder.outputJointPositions[i][
                : len(recorder.inputJointPositions[i])
            ],  # enforce same length between input num_joints and output num_joints
            *recorder.outputCurrents[i][: len(recorder.inputJointPositions[i])],
            recorder.imuTime[i],
            *recorder.outputTcpAccelerations[i],
            recorder.quaternionTime[i],
            *recorder.quaternion[i],
        ]

        motion_data_rows.append(row)

    # Create filename
    motion_filename = (
        f"{data_folder}/robotData_motion_pose{run_index}_axis{move_axis}.csv"
    )

    # Create directories if they don't exist
    motion_directory = os.path.dirname(motion_filename)
    os.makedirs(motion_directory, exist_ok=True)

    # Store motion data in csv
    with open(file=motion_filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(motion_headers)  # header row
        writer.writerows(motion_data_rows)  # all data rows

    # Static parameters
    static_headers = [
        "input_v",
        "input_r",
        *[f"j{i}_inertia" for i in range(len(recorder.outputMassDiagonals))],
        *[
            f"end_index_{i}" for i in range(len(recorder.endIndices))
        ],  # (unknown size) -- must be the last elements
    ]

    static_data_rows = []
    static_data_rows.append(
        [
            *recorder.inputV,
            *recorder.inputR,
            *recorder.outputMassDiagonals,
            *recorder.endIndices,
        ]
    )

    # Create filename
    static_filename = (
        f"{data_folder}/robotData_static_pose{run_index}_axis{move_axis}.csv"
    )

    # Create directories if they don't exist
    static_directory = os.path.dirname(static_filename)
    os.makedirs(static_directory, exist_ok=True)

    # Store static data in csv
    with open(file=static_filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(static_headers)  # header row
        writer.writerows(static_data_rows)  # all data rows


def store_parameters_in_data_folder(
    traj_params: TrajParams,
    sysid_params: SystemIdParams,
    axes_commanded: int,
    num_joints: int,
    sample_time: float,
    start_pose: int,
    shoulder_len: float,
    base_height: float,
    robot_name: str,
    data_folder: str,
) -> None:
    """Persist identification parameters to a CSV file.

    Args:
        traj_params: Trajectory parameters.
        sysid_params: System identification parameters.
        axes_commanded: Number of axes commanded.
        num_joints: Number of robot joints.
        sample_time: Sampling time [s].
        start_pose: Starting pose index.
        shoulder_len: Shoulder link length [m].
        base_height: Base height [m].
        robot_name: Robot name identifier.
        data_folder: Output directory for the parameters file.

    Returns:
        `None`.

    Side Effects:
        Creates directories and writes `identification_parameters.csv` to disk.

    Raises:
        OSError: If the file cannot be created or written.

    Preconditions:
        `data_folder` is writable.
    """

    parameter_headers = [
        "max_disp",
        "max_vel",
        "max_acc",
        "sysid_type",
        "ctrl_config",
        "single_pt_run_time",
        "output_sensor",
        "nV",
        "nR",
        "sine_cycles",
        "min_freq",
        "max_freq",
        "freq_space",
        "dwell",
        "axes",
        "num_joints",
        "sample_time",
        "start_pose",
        "shoulder_len",
        "base_height",
        "robot_name",
    ]

    parameter_data_rows = []
    parameter_data_rows.append(
        [
            traj_params.max_displacement,
            traj_params.max_velocity,
            traj_params.max_acceleration,
            traj_params.sysid_type,  # string
            traj_params.configuration,  # string
            traj_params.single_pt_run_time,
            traj_params.output_sensor,  # string
            sysid_params.nV,
            sysid_params.nR,
            sysid_params.num_sine_cycles,
            sysid_params.min_freq,
            sysid_params.max_freq,
            sysid_params.freq_space,
            sysid_params.dwell,
            axes_commanded,
            num_joints,
            sample_time,
            start_pose,
            shoulder_len,
            base_height,
            robot_name,  # string
        ]
    )

    # Create filename
    parameter_filename = f"{data_folder}/identification_parameters.csv"

    # Create directories if they don't exist
    parameter_directory = os.path.dirname(parameter_filename)
    os.makedirs(parameter_directory, exist_ok=True)

    # Store parameters
    with open(file=parameter_filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(parameter_headers)  # header row
        writer.writerows(parameter_data_rows)  # all data rows
