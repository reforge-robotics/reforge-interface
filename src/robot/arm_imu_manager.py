"""Command the arm through a reference trajectory and record arm + IMU data."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

from reforge_core.imu.client import MuseIMUClient
from reforge_core.imu.data import IMUState
from robot.arm_client import ArmClient
from robot.arm_imu_time_calibration import run_arm_imu_time_calibration


MAX_IMU_CONNECTION_RETRIES = 10


@dataclass
class ArmData:
    recv_time: float
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


@dataclass
class CommandData:
    cmd_time: float
    cmd_positions: list[float]


class ArmImuManager:
    def __init__(
        self,
        arm: ArmClient,
        Ts: float,
        time_data: list,
        position_data: list,
        buffer_len_s: float = 3600,
    ):
        self.arm = arm
        self.imu: MuseIMUClient | None = None
        self.Ts = Ts
        self.calibration_result = None

        self.time_data = time_data
        self.position_data = position_data

        maxlen = int(buffer_len_s * (1.0 / Ts))
        self.arm_data: deque[ArmData] = deque(maxlen=maxlen)
        self.imu_data: deque[IMUState] = deque(maxlen=maxlen)
        self.cmd_data: deque[CommandData] = deque(maxlen=maxlen)

        self.aligned_log = deque(maxlen=maxlen)

        self.prepare_arm()
        self.prepare_imu()

    def prepare_arm(self):
        """Ensure arm is enabled and set to servo control mode."""
        print("Preparing arm for next move...")
        self.arm.set_servo_mode()

    def prepare_imu(self):
        """Ensure the Muse client is calibrated, streaming, and producing samples."""
        self.imu = MuseIMUClient(time_offset_sec=0.0)
        for i in range(MAX_IMU_CONNECTION_RETRIES):
            print(f"Connecting to IMU (attempt {i + 1}/{MAX_IMU_CONNECTION_RETRIES})")
            try:
                self.imu.connect()
            except Exception as exc:
                print(f"IMU connect attempt failed: {exc}")
            if self.imu.is_connected():
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"Failed to connect to IMU after {MAX_IMU_CONNECTION_RETRIES} attempts."
            )

        self.calibration_result, self.imu = run_arm_imu_time_calibration(
            imu_client=self.imu,
            arm_client=self.arm,
            arm_control_mode="servo",
        )

        self.imu.set_streaming_data_frequency(1.0 / self.Ts, rounding="up")
        self.imu.start_streaming()

        while True:
            try:
                self.imu.get_latest_state(timeout_s=1.0)
                break
            except Exception:
                print("IMU data not being received. Is Muse powered and connected?")
                time.sleep(0.1)

    def get_imu_data(self) -> IMUState:
        if self.imu is None:
            raise RuntimeError("IMU client has not been initialized.")
        return self.imu.get_latest_state(timeout_s=1.0)

    def get_arm_data(self) -> ArmData:
        state = self.arm.get_state()
        return ArmData(
            recv_time=float(state.ts),
            positions=state.q.tolist(),
            velocities=state.qd.tolist(),
            efforts=state.tau.tolist(),
        )

    def publish_and_record(self):
        if len(self.time_data) != len(self.position_data):
            raise ValueError("time_data and position_data must have the same length")
        if not self.time_data:
            return

        overall_t0 = time.time()
        total_cmds = len(self.time_data)
        progress_every = max(1, total_cmds // 10)
        print(
            f"[ArmImuManager] Starting publish_and_record with {total_cmds} commands at {1.0 / self.Ts:.1f} Hz."
        )
        print("[ArmImuManager] Stopping IMU streaming and starting arm/IMU recording...")
        if self.imu is None:
            raise RuntimeError("IMU client has not been initialized.")

        phase_t0 = time.time()
        self.imu.stop_streaming()
        self.arm.set_recording_data_frequency(1.0 / self.Ts, rounding="nearest")
        self.imu.set_recording_data_frequency(1.0 / self.Ts, rounding="up")
        self.imu.start_recording()
        self.arm.start_recording()
        print(
            f"[ArmImuManager] Recorder startup took {time.time() - phase_t0:.3f} s."
        )

        tref0 = self.time_data[0]
        t0 = time.time()
        command_loop_t0 = time.time()
        try:
            for cmd_idx, (tref, pref) in enumerate(
                zip(self.time_data, self.position_data),
                start=1,
            ):
                target_time = t0 + (tref - tref0)
                while True:
                    now = time.time()
                    delta = target_time - now
                    if delta <= 0.0:
                        break
                    time.sleep(min(delta, self.Ts / 2.0))

                cmd_time = time.time()
                code = self.arm.command_servo(pref)
                if code != 0:
                    print(
                        f"[ArmImuManager] command_servo returned code {code} "
                        f"for pref={pref}"
                    )
                self.cmd_data.append(
                    CommandData(cmd_time=cmd_time, cmd_positions=list(pref))
                )
                if cmd_idx == 1 or cmd_idx == total_cmds or (cmd_idx % progress_every) == 0:
                    progress_pct = 100.0 * cmd_idx / total_cmds
                    print(
                        f"[ArmImuManager] Command progress: {cmd_idx}/{total_cmds} ({progress_pct:.1f}%)"
                    )
        finally:
            print("[ArmImuManager] Stopping arm and IMU recording...")
            stop_t0 = time.time()
            self.arm.stop_recording()
            self.imu.stop_recording()
            print(
                f"[ArmImuManager] Recorder shutdown took {time.time() - stop_t0:.3f} s."
            )

        print(
            f"[ArmImuManager] Command loop took {time.time() - command_loop_t0:.3f} s."
        )
        print("[ArmImuManager] Trajectory complete. Fetching recorded trajectories...")

        fetch_t0 = time.time()
        arm_traj = self.arm.get_recording()
        imu_traj = self.imu.get_recording()
        arm_states = list(arm_traj.states)
        imu_states = list(imu_traj.states)
        print(
            f"[ArmImuManager] Trajectory fetch took {time.time() - fetch_t0:.3f} s."
        )

        print(
            f"[ArmImuManager] Retrieved {len(arm_states)} arm samples and {len(imu_states)} IMU samples."
        )

        if not arm_states:
            raise RuntimeError("Arm recording completed with no samples.")
        if not imu_states:
            raise RuntimeError("IMU recording completed with no samples.")

        print("[ArmImuManager] Matching recorded samples back to command timestamps...")
        align_t0 = time.time()
        arm_ts = [state.ts for state in arm_states]
        imu_ts = [state.ts for state in imu_states]
        tcp_quat = self.arm.get_tcp_pose()[3:]

        for cmd_idx, cmd_d in enumerate(self.cmd_data, start=1):
            arm_idx = min(
                range(len(arm_states)),
                key=lambda idx: abs(arm_ts[idx] - cmd_d.cmd_time),
            )
            imu_idx = min(
                range(len(imu_states)),
                key=lambda idx: abs(imu_ts[idx] - cmd_d.cmd_time),
            )
            arm_state = arm_states[arm_idx]
            imu_d = imu_states[imu_idx]
            accel = [imu_d.ax, imu_d.ay, imu_d.az]
            gyro = [imu_d.gx, imu_d.gy, imu_d.gz]

            self.aligned_log.append(
                {
                    "cmd_time": cmd_d.cmd_time,
                    "input_positions": cmd_d.cmd_positions,
                    "output_positions": arm_state.q.tolist(),
                    "velocities": arm_state.qd.tolist(),
                    "efforts": arm_state.tau.tolist(),
                    "imu_time": float(imu_d.ts),
                    "linear_acceleration": accel,
                    "angular_velocity": gyro,
                    "orientation": tcp_quat,
                }
            )
            if cmd_idx == 1 or cmd_idx == total_cmds or (cmd_idx % progress_every) == 0:
                progress_pct = 100.0 * cmd_idx / total_cmds
                print(
                    f"[ArmImuManager] Alignment progress: {cmd_idx}/{total_cmds} ({progress_pct:.1f}%)"
                )

        print(
            f"[ArmImuManager] Alignment took {time.time() - align_t0:.3f} s."
        )
        print("[ArmImuManager] Alignment complete. Returning arm to servo-ready mode.")
        reset_t0 = time.time()
        self.prepare_arm()
        print(
            f"[ArmImuManager] Servo reset took {time.time() - reset_t0:.3f} s."
        )
        print(
            f"[ArmImuManager] Total publish_and_record time: {time.time() - overall_t0:.3f} s."
        )
