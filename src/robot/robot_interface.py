# src/robot/robot_interface.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Specific code to create calibration interface for any Python Robot.
# Version: 2.0


import sys
from collections.abc import Mapping
from importlib.resources import as_file, files
from pathlib import Path
from typing import Literal, Optional, Sequence

import numpy as np

# almond-axol requires Python 3.12+. On 3.11, asyncio.wait_for can swallow a
# task cancellation, so the SDK's motor telemetry loops never stop and
# disconnect()/enable() hang.
if sys.version_info < (3, 12):
    raise RuntimeError(
        "The Axol interface requires Python 3.12+ (almond-axol's minimum); "
        f"this is Python {sys.version.split()[0]}."
    )

# Almond SDK imports including async libraries
from almond_axol.constants import ARM_JOINTS, CAN_LEFT, CAN_RIGHT, urdf_arm_joint_names
from almond_axol.kinematics import KinematicsSolver
from almond_axol.robot import Axol
from almond_axol.teleop.config import VRTeleopConfig
from almond_axol.teleop.trajectory import plan_collision_aware_trajectory
import asyncio
from concurrent.futures import Future
import threading

from reforge_core.hw_interfaces.arm_client import ArmClient
from reforge_core.hw_interfaces.imu_recorder import ImuRecorder
from reforge_core.util.utility import rotation_matrix_to_quaternion  # {~.~}

# Select an arm (left or right)
# Changing this flag automatically updates all downstream variables -- poses, urdfs, etc.
USE_LEFT = False

# ========== PER-ARM PARAMETERS =============
AXOL_SIDE = "left" if USE_LEFT else "right"
AXOL_CAN_CHANNEL = CAN_LEFT if USE_LEFT else CAN_RIGHT
AXOL_SDK_ARM_ATTRIBUTE = AXOL_SIDE
AXOL_TCP_LINK = f"{AXOL_SIDE}_gripper"
AXOL_JOINT_NAMES = tuple(joint.value for joint in ARM_JOINTS)
AXOL_URDF_JOINT_NAMES = tuple(urdf_arm_joint_names(is_left=USE_LEFT))

BOT_ID = "" if USE_LEFT else ""
# Source bimanual model used to generate both per-arm URDFs during calibration.
BASE_URDF_PATH = "urdf/axol.urdf"
URDF_PATH = f"urdf/axol-{AXOL_SIDE}.urdf"
# Arm straight out in front at shoulder height. The split URDF's base +z is
# the shoulder_1 axis (horizontal in the world), so the TCP must be stretched
# perpendicular to it for the calibration geometry to find a reach, height and
# depth axis. The elbow is bent 3 deg off its straight-arm URDF limit (0 rad) so
# encoder noise never reads past it.
FULL_STRETCH_JOINTS = [
    np.pi / 2 if USE_LEFT else -np.pi / 2,
    0.0,
    0.0,
    0.05 if USE_LEFT else -0.05,
    0.0,
    0.0,
    0.0,
]
# TCP pose of FULL_STRETCH_JOINTS in the split URDF frame ([x, y, z], [qx, qy, qz, qw]).
FULL_STRETCH_XYZ = [-0.017453 if USE_LEFT else 0.017453, -0.71151, 0.06958]
FULL_STRETCH_QUAT = (
    [0.512342, 0.487345, 0.512342, -0.487345]
    if USE_LEFT
    else [0.512344, -0.487344, -0.512341, -0.487346]
)
DEFAULT_TCP_PAYLOAD = 0.0

# ========== COMMON PARAMETERS ==============
ROBOT_MAX_FREQ = 240  # {~.~} [CHANGE TO ROBOT'S MAX SAMPLING FREQUENCY] in [Hz]
FULL_STRETCH_POSE_OVERRIDE = None  # {~.~} list of home pose (xyz and quaternion) to override additional height not in base height
AXOL_MAX_JOINT_SPEED = 2.0 * np.pi
AXOL_MAX_JOINT_ACCELERATION = 3.5 * 2.0 * np.pi
# Rest pose and return speed match `axol teleop`'s reset (0.63 rad/s average).
AXOL_REST_JOINTS = (
    VRTeleopConfig().rest_pose_left if USE_LEFT else VRTeleopConfig().rest_pose_right
).tolist()
AXOL_REST_SPEED_PERCENT = 15.0
AXOL_REST_TOLERANCE = 0.02  # [rad]

# General constants
IS_DEGREES = False  # {~.~} [CHANGE TO TRUE IF ROBOT USES DEGREES]
# Anchored to this package so data lands in src/robot/data/<date> from any working directory.
DATA_LOCATION_PREFIX = str(Path(__file__).resolve().parent / "data")
SIM_DATA_LOCATION_PREFIX = str(Path(__file__).resolve().parent / "data" / "sim")

MAX_ROBOT_JOINTS_BANDWIDTH = (
    5.0  # {~.~} Servo motor bandwidth. Leave as is if you don't know [Hz]
)

# {~.~} IMU information
USE_REFORGE_IMU = True
DEFAULT_IMU_COMM_MODE: Literal["ble", "usb", "virtual"] = "virtual"
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

        super().__init__(
            name="My Robot", recording_data_frequency_hz=ROBOT_MAX_FREQ
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
        self.robot: Axol | None = None
        self._axol_loop: asyncio.AbstractEventLoop | None = None
        self._axol_thread: threading.Thread | None = None

        # define axol states - motion, teaching, P2P
        self._axol_motion_enabled = False

        self._axol_teaching_future: Future[None] | None = None  # {~.~}
        self._axol_teaching_stop = threading.Event()  # {~.~}

        self._axol_kinematics_solver: KinematicsSolver | None = None  # {~.~}
        self._axol_move_future: Future[None] | None = None  # {~.~}

        # Reforge API and robot ID token is needed for "joint_tracker" product
        # Add it in the CLI with `--identify`
        self.reforge_api_token = api_token
        try:
            # Instantiate live robot mode.
            self.robot = Axol(
                left_channel=AXOL_CAN_CHANNEL if USE_LEFT else None,
                right_channel=None if USE_LEFT else AXOL_CAN_CHANNEL,
                left_joints=ARM_JOINTS if USE_LEFT else None,
                right_joints=None if USE_LEFT else ARM_JOINTS,
                loop_hz=ROBOT_MAX_FREQ,
                max_vel=AXOL_MAX_JOINT_SPEED,
                max_accel=AXOL_MAX_JOINT_ACCELERATION,
            )
            self._axol_loop = asyncio.new_event_loop()
            self._axol_thread = threading.Thread(
                target=self._axol_loop.run_forever,
                name="axol-event-loop",
                daemon=True,
            )
            self._axol_thread.start()

            self._run_axol(self.robot.connect())
            self._run_axol(
                self.robot.start_telemetry(ROBOT_MAX_FREQ, torque=True)
            )
            self._run_axol(self.robot.wait_for_telemetry())
            active_arm = self.robot.left if USE_LEFT else self.robot.right
            inactive_arm = self.robot.right if USE_LEFT else self.robot.left
            if active_arm is None or inactive_arm is not None:
                raise RuntimeError("Axol did not isolate the selected arm.")

            self.id = robot_id
            num_joints_sdk = len(self._get_joint_positions())
            if num_joints_sdk != self.num_joints:
                raise RuntimeError(
                    f"Number of robot joints in URDF ({self.num_joints}) is not "
                    f"equivalent to the number returned by Axol ({num_joints_sdk})."
                )
            self.pose_length = 7

        except BaseException as error:
            self._cleanup_axol_after_error(error)
            if isinstance(error, Exception):
                raise RuntimeError(
                    f"Error getting {robot_ip} operational: {error}"
                ) from error
            raise

        try:
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
        except BaseException as error:
            self._cleanup_axol_after_error(error)
            raise

    def _cleanup_axol_after_error(self, error: BaseException) -> None:
        try:
            self.close()
        except BaseException as cleanup_error:
            error.add_note(f"Axol cleanup failed: {cleanup_error}")

    def _run_axol(self, coroutine) -> object:
        if self._axol_loop is None:
            raise RuntimeError("Axol event loop is not initialized.")
        # Submit one synchronous SDK call to the persistent Axol loop.
        return asyncio.run_coroutine_threadsafe(coroutine, self._axol_loop).result()

    # Axol point-to-point helpers.
    async def _axol_joint_move(self, robot: Axol, trajectory: np.ndarray) -> None:
        period_s = 1.0 / ROBOT_MAX_FREQ
        loop = asyncio.get_running_loop()
        for target in trajectory:
            started_s = loop.time()
            command = np.zeros(8, dtype=np.float32)
            command[: len(ARM_JOINTS)] = target
            await robot.motion_control(**{AXOL_SDK_ARM_ATTRIBUTE: command})
            if robot.fault is not None:
                raise RuntimeError(f"Axol realtime core faulted: {robot.fault}")
            if robot.limp is not None:
                raise RuntimeError(f"Axol realtime core is limp: {robot.limp}")
            await asyncio.sleep(max(0.0, period_s - (loop.time() - started_s)))

    def _wait_for_axol_move(self) -> None:
        future = self._axol_move_future
        if future is None:
            return
        try:
            future.result()
        finally:
            self._axol_move_future = None

    def _get_axol_kinematics_solver(self) -> KinematicsSolver:
        if self._axol_kinematics_solver is None:
            self._axol_kinematics_solver = KinematicsSolver()
        return self._axol_kinematics_solver

    @staticmethod
    def _validate_move_speed(speed: float) -> float:
        try:
            speed_percent = float(speed)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Axol move speed must be numeric.") from exc
        if not np.isfinite(speed_percent) or not 0.0 < speed_percent <= 100.0:
            raise ValueError("Axol move speed must be in (0, 100].")
        return speed_percent / 100.0

    def _submit_axol_trajectory(
        self, robot: Axol, trajectory: Sequence[Sequence[float]], wait: bool
    ) -> None:
        arm_trajectory = np.asarray(trajectory, dtype=np.float32)
        if arm_trajectory.ndim != 2 or arm_trajectory.shape[1] != len(ARM_JOINTS):
            raise RuntimeError("Axol planner returned an invalid arm trajectory.")
        if not np.all(np.isfinite(arm_trajectory)):
            raise RuntimeError("Axol planner returned a non-finite arm trajectory.")
        if self._axol_loop is None:
            raise RuntimeError("Axol event loop is not initialized.")
        self._run_axol(self._axol_joint_move(robot, arm_trajectory[:1]))
        self._axol_move_future = asyncio.run_coroutine_threadsafe(
            self._axol_joint_move(robot, arm_trajectory[1:]), self._axol_loop
        )
        if wait:
            self._wait_for_axol_move()

    # Axol teaching helpers.
    async def _axol_teaching_loop(self, robot: Axol) -> None:
        period_s = 1.0 / ROBOT_MAX_FREQ
        loop = asyncio.get_running_loop()
        while not self._axol_teaching_stop.is_set():
            started_s = loop.time()
            await robot.gravity_compensate(kd=0.5)
            await asyncio.sleep(max(0.0, period_s - (loop.time() - started_s)))

    def _stop_axol_teaching(self) -> bool:
        future = self._axol_teaching_future
        if future is None:
            return False
        self._axol_teaching_stop.set()
        try:
            future.result()
        finally:
            self._axol_teaching_future = None
        return True

    def _get_joint_positions(self) -> list[float]:
        if self.robot is None:
            raise RuntimeError("Axol robot is not initialized.")
        arm = self.robot.left if USE_LEFT else self.robot.right
        if arm is None:
            raise RuntimeError(f"No {AXOL_SIDE} Axol arm is available.")
        return np.asarray(arm.positions, dtype=float)[: self.num_joints].tolist()

    def _return_to_rest(self) -> bool:
        """Move the active arm to its rest pose; return whether it got there."""
        robot = self.robot
        if robot is None or robot.fault is not None or robot.limp is not None:
            return False
        rest = np.asarray(AXOL_REST_JOINTS, dtype=float)
        current = np.asarray(self._get_joint_positions(), dtype=float)
        if np.max(np.abs(current - rest)) > AXOL_REST_TOLERANCE:
            print("Returning the Axol arm to rest before disabling ...")
            self.command_move_j(rest, speed=AXOL_REST_SPEED_PERCENT, wait=True)
            current = np.asarray(self._get_joint_positions(), dtype=float)
        return bool(np.max(np.abs(current - rest)) <= 2.0 * AXOL_REST_TOLERANCE)

    def close(self) -> None:
        """Return the arm to rest, then disable it and close the Axol async loop.

        Disabling cuts torque, so it only happens once the arm is at rest. If
        the arm cannot get there the buses are released with the motors still
        holding (the realtime core's own behavior when a session ends), and the
        operator must support the arm before using the e-stop. A session that
        never enabled motion is released without touching motor torque.
        """
        if self.robot is None:
            return
        try:
            self._wait_for_axol_move()
        except Exception:
            pass  # Teardown must still run if a move failed.

        try:
            self._stop_axol_teaching()
        except Exception:
            pass  # Teardown must still run if the teaching stream failed.

        self.stop_recording()
        if not self._axol_motion_enabled:
            self._run_axol(self.robot.disconnect())
        elif self.robot.fault is not None or self.robot.limp is not None:
            # The core keeps a faulted arm holding and a limp arm limp.
            self._run_axol(self.robot.disable())
        else:
            try:
                at_rest = self._return_to_rest()
            except Exception as error:
                print(f"Axol return to rest failed: {error}")
                at_rest = False
            if at_rest:
                self._run_axol(self.robot.disable())
            else:
                print(
                    "WARNING: the Axol arm is not at rest, so it was left holding "
                    "its pose instead of being disabled. Support the arm before "
                    "using the e-stop."
                )
                self._run_axol(self.robot.disconnect())
        self._axol_motion_enabled = False

        if self._axol_loop is not None:
            self._axol_loop.call_soon_threadsafe(self._axol_loop.stop)
        if self._axol_thread is not None:
            self._axol_thread.join()
            if self._axol_thread.is_alive():
                raise RuntimeError("Axol event-loop thread did not stop.")
        if self._axol_loop is not None:
            self._axol_loop.close()

        self.robot = None
        self._axol_loop = None
        self._axol_thread = None

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

    # REQUIRED METHODS
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
        target = self._validate_joint_target(target_joints)
        speed_scale = self._validate_move_speed(speed)
        lower_limits, upper_limits = self.model.joint_limits
        if np.any(target < lower_limits) or np.any(target > upper_limits):
            raise ValueError("Axol joint target exceeds the URDF joint limits.")

        self.enter_position_mode()
        robot = self._require_connected_arm()
        solver = self._get_axol_kinematics_solver()
        start = np.asarray(self._get_joint_positions(), dtype=np.float32)
        indices = solver.left_indices if USE_LEFT else solver.right_indices
        q_from = np.zeros(solver.num_joints, dtype=np.float32)
        q_from[indices] = start
        q_to = q_from.copy()
        q_to[indices] = target
        full_trajectory = plan_collision_aware_trajectory(
            solver,
            q_from,
            q_to,
            speed=AXOL_MAX_JOINT_SPEED * speed_scale / 1.5,
            rate=ROBOT_MAX_FREQ,
            min_duration=1.0 / ROBOT_MAX_FREQ,
        )
        planned = np.asarray(full_trajectory, dtype=np.float32)
        if not np.all(np.isfinite(planned)):
            raise RuntimeError("Axol joint planner returned non-finite values.")
        inactive = solver.right_indices if USE_LEFT else solver.left_indices
        if np.max(np.abs(planned[:, inactive])) > 1e-4:
            raise RuntimeError("Axol planner could not keep the inactive arm fixed.")
        self._submit_axol_trajectory(robot, planned[:, indices], wait)
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
        self._validate_move_speed(speed)
        try:
            xyz = np.asarray(target_xyz, dtype=float)
            quat = np.asarray(target_quat, dtype=float)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Axol Cartesian targets must be numeric.") from exc
        
        if xyz.shape != (3,) or quat.shape != (4,):
            raise ValueError("Expected target_xyz shape (3,) and target_quat shape (4,).")
        if not np.all(np.isfinite(xyz)) or not np.all(np.isfinite(quat)):
            raise ValueError("Axol Cartesian targets must be finite.")
        quat_norm = float(np.linalg.norm(quat))
        if not np.isfinite(quat_norm) or quat_norm == 0.0:
            raise ValueError("Axol target quaternion must have a finite, nonzero norm.")
        
        self.enter_position_mode()
        start = np.asarray(self._get_joint_positions(), dtype=np.float32)
        target_pose = np.concatenate((xyz, quat / quat_norm))
        target, report = self.model.get_inverse_kinematics(
            target_pose=target_pose,
            initial_angles=start,
            tol=1e-4,
            link_name=AXOL_TCP_LINK,
            get_report=True,
        )
        if not report.converged:
            raise RuntimeError(
                "Axol pose IK did not converge "
                f"({report.position_error_mag:.6f} m, "
                f"{report.rotation_error_mag:.6f} rad error)."
            )
        target = self._validate_joint_target(target)
        lower_limits, upper_limits = self.model.joint_limits
        if np.any(target < lower_limits) or np.any(target > upper_limits):
            raise ValueError("Axol pose IK target exceeds the URDF joint limits.")
        # Point-to-point, not a straight line: a Cartesian path out of the
        # near-singular stretched pose cannot be tracked, so move in joint
        # space (collision-aware) to the IK solution.
        return self.command_move_j(target, speed=speed, wait=wait)

    @staticmethod
    def _validate_joint_target(
        target_joints: Sequence[float] | np.ndarray,
    ) -> np.ndarray:
        try:
            target = np.asarray(target_joints, dtype=np.float32)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("Axol joint targets must be numeric.") from exc
        if target.shape != (len(ARM_JOINTS),):
            raise ValueError(f"Expected {len(ARM_JOINTS)} joint targets.")
        if not np.all(np.isfinite(target)):
            raise ValueError("Axol joint targets must be finite.")
        return target

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
        self._wait_for_axol_move()  # do not overlap P2P and servo streams.
        robot = self._require_connected_arm()
        del wait  # Axol has no target-settled acknowledgement.
        target = self._validate_joint_target(target_joints)
        if robot.fault is not None:
            raise RuntimeError(f"Axol realtime core faulted: {robot.fault}")
        if robot.limp is not None:
            raise RuntimeError(f"Axol realtime core is limp: {robot.limp}")
        command = np.zeros(8, dtype=np.float32)
        command[: len(ARM_JOINTS)] = target
        self._run_axol(
            robot.motion_control(**{AXOL_SDK_ARM_ATTRIBUTE: command})
        )
        return 0

    def enter_position_mode(self) -> Optional[int | None]:
        """
        Ensure the controller is in point-to-point position mode before issuing queued P2P moves.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        self._wait_for_axol_move()  # only change mode after axol stops moving
        robot = self._require_connected_arm()
        if not self._axol_motion_enabled:
            # Axol uses the same mode for position and servo control
            self._run_axol(robot.enable())
            self._axol_motion_enabled = True
        if robot.fault is not None:
            raise RuntimeError(f"Axol realtime core faulted: {robot.fault}")  # {~.~}
        if robot.limp is not None:
            raise RuntimeError(f"Axol realtime core is limp: {robot.limp}")  # {~.~}
        was_teaching = self._axol_teaching_future is not None
        try:
            self._stop_axol_teaching()
        finally:
            if was_teaching:
                robot.reset_command_state()
                self.command_servo_j(self._get_joint_positions())
        return 0 

    def enter_servo_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to servo control mode.

        Returns:
            the mode/state codes so they can be inspected when debugging.
        """
        return self.enter_position_mode()  # Axol shares one realtime control mode.

    def supports_teaching_mode(self) -> bool:
        """Return whether the robot supports manual teaching mode.

        Override this method when the robot SDK supports hand-guided teaching.

        Returns:
            `bool` indicating whether manual teaching mode is implemented.
        """
        return True  # Axol streams gravity compensation for teaching.

    def enter_teaching_mode(self) -> Optional[int | None]:
        """Ensure the controller is set to manual teaching mode.

        Override this method with the robot SDK's teaching-mode command.

        Returns:
            Vendor-specific mode/state code when available.
        """
        future = self._axol_teaching_future
        if future is not None and not future.done():
            return 0

        # Run a coroutine which continually applies gravity compensation to realtime loop.
        self.enter_position_mode()
        robot = self._require_connected_arm()
        if self._axol_loop is None:
            raise RuntimeError("Axol event loop is not initialized.")
        self._axol_teaching_stop.clear()
        self._axol_teaching_future = asyncio.run_coroutine_threadsafe(
            self._axol_teaching_loop(robot), self._axol_loop
        )
        return 0 

    def supports_flange_button(self) -> bool:
        """Return whether the robot exposes a readable flange button.

        Override this method when the robot SDK exposes a button or equivalent
        operator input near the tool flange.

        Returns:
            `bool` indicating whether flange-button reads are implemented.
        """
        return False  # Axol exposes no documented flange-button input.

    def read_flange_button_pressed(self) -> bool:
        """Return whether the flange button is currently pressed.

        Override this method with the robot SDK's flange-button read.

        Returns:
            `bool` indicating the current flange-button state.
        """
        raise NotImplementedError("Axol does not expose a flange-button input.")

    def get_joint_state(self) -> tuple[list[float], list[float], list[float]]:
        """Return one joint state sample as ``(q, qd, tau)``.

        Returns:
            Tuple of three lists: joint positions `q` [rad], velocities `qd` [rad/s],
            and efforts/currents `tau` [SDK units].
        """
        robot = self._require_connected_arm()

        async def read_joint_state():
            return await asyncio.gather(
                robot.get_positions(),
                robot.get_velocities(),
                robot.get_torques(),
            )

        try:
            side_index = 0 if USE_LEFT else 1
            channels = tuple(
                np.asarray(states[side_index], dtype=float)[: self.num_joints]
                for states in self._run_axol(read_joint_state())
            )
        except Exception as exc:
            raise RuntimeError(
                f"Unable to read {AXOL_SIDE} Axol joint state."
            ) from exc
        if any(
            channel.shape != (self.num_joints,)
            or not np.all(np.isfinite(channel))
            for channel in channels
        ):
            raise RuntimeError(f"Axol {AXOL_SIDE} joint state is unavailable.")
        q, qd, tau = (channel.tolist() for channel in channels)
        return q, qd, tau

    def get_tcp_pose(self) -> list[float]:
        """Return TCP pose as ``[x, y, z, qx, qy, qz, qw]``.

        Returns:
            List of 7 floats representing the TCP pose in meters for positions
            and unitless normalized for quaternions.
        """
        joint_positions = self._get_joint_positions()
        transform = self.model.get_transformation_matrix(
            joint_angles=joint_positions,
            link_name=AXOL_TCP_LINK,
        )
        position = np.asarray(transform[:3, 3], dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise RuntimeError("Reforge model returned an invalid TCP position.")
        quaternion = rotation_matrix_to_quaternion(
            np.asarray(transform[:3, :3], dtype=float)
        )
        quaternion_norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(quaternion_norm) or quaternion_norm == 0.0:
            raise RuntimeError("Reforge model returned an invalid TCP quaternion.")
        quaternion = quaternion / quaternion_norm
        return [*position.tolist(), *quaternion.tolist()]


    # OPTIONAL OVERRIDES
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