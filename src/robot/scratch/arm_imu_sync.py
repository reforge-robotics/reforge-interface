"""Collect one arm/IMU synchronization dataset and save it to disk.

This script records:
- an arm trajectory captured by polling xArm joint states
- an IMU trajectory captured through MuseIMUClient's stream-backed recorder

Timestamp convention in this file:
- Arm timestamps are stored in UTC unix epoch seconds as ``float``.
- IMU timestamps are already public aligned UTC unix epoch seconds as ``float``.
"""

from __future__ import annotations

import datetime
import pickle
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from reforge_core.imu.client import MuseIMUClient

from robot.arm_client import ArmClient


ROBOT_IP = "192.168.1.208"
OUTPUT_DIR = Path(__file__).with_name("syncdata")
# Motion parameters
BASE_STEP_DEG = 5.0
POSITION_SPEED = 100.0
PRE_STEP_SETTLE_S = 0.5
# Recording parameters
IMU_RECORDING_HZ = 200.0


@dataclass(frozen=True, slots=True)
class StepTiming:
    name: str
    elapsed_s: float


def _make_output_path() -> Path:
    timestamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y%m%d_%H%M%S"
    )
    return OUTPUT_DIR / f"arm_imu_recording_{timestamp}.pkl"


def _run_timed_step(timings: list[StepTiming], name: str, fn):
    t0 = time.perf_counter()
    result = fn()
    elapsed_s = time.perf_counter() - t0
    timings.append(StepTiming(name=name, elapsed_s=elapsed_s))
    return result


def _print_timing_summary(timings: list[StepTiming], total_elapsed_s: float) -> None:
    print("\nTiming Summary")
    print("-" * 52)
    for timing in timings:
        print(f"{timing.name:<36} {timing.elapsed_s:7.3f} s")
    print("-" * 52)
    print(f"{'Total':<36} {total_elapsed_s:7.3f} s")


def main() -> None:
    arm = ArmClient(ROBOT_IP)
    imu = MuseIMUClient()
    output_path = _make_output_path()
    timings: list[StepTiming] = []
    total_t0 = time.perf_counter()

    arm_stop_status = None
    imu_stop_status = None

    try:
        _run_timed_step(timings, "Connect IMU", imu.connect)
        _run_timed_step(timings, "Synchronize IMU time", imu.set_time)
        configured_imu_hz = _run_timed_step(
            timings,
            "Configure IMU recording rate",
            lambda: imu.set_recording_data_frequency(IMU_RECORDING_HZ),
        )
        print(f"Configured IMU recording frequency: {configured_imu_hz:.1f} Hz")

        q0 = _run_timed_step(
            timings,
            "Read initial arm state",
            lambda: arm.get_latest_state().q,
        )
        q_plus = q0.copy()
        q_minus = q0.copy()
        q_plus[0] = q0[0] + np.deg2rad(BASE_STEP_DEG)
        q_minus[0] = q0[0] - np.deg2rad(BASE_STEP_DEG)

        _run_timed_step(timings, "Set arm position mode", arm.set_position_mode)

        _run_timed_step(timings, "Start IMU recording", imu.start_recording)
        _run_timed_step(timings, "Start arm recording", arm.start_recording)

        _run_timed_step(
            timings,
            f"Pre-step hold ({PRE_STEP_SETTLE_S:.1f} s)",
            lambda: time.sleep(PRE_STEP_SETTLE_S),
        )

        code_q_plus = _run_timed_step(
            timings,
            f"Move to +{BASE_STEP_DEG:.1f} deg",
            lambda: arm.command_position(q_plus, speed=POSITION_SPEED, wait=True),
        )
        if code_q_plus != 0:
            raise RuntimeError(f"command_position(q_plus) returned code {code_q_plus}")

        code_q_minus = _run_timed_step(
            timings,
            f"Move to -{BASE_STEP_DEG:.1f} deg",
            lambda: arm.command_position(q_minus, speed=POSITION_SPEED, wait=True),
        )
        if code_q_minus != 0:
            raise RuntimeError(
                f"command_position(q_minus) returned code {code_q_minus}"
            )

        code_q0 = _run_timed_step(
            timings,
            "Return to q0",
            lambda: arm.command_position(q0, speed=POSITION_SPEED, wait=True),
        )
        if code_q0 != 0:
            raise RuntimeError(f"command_position(q0) returned code {code_q0}")

        imu_stop_status = _run_timed_step(
            timings,
            "Stop IMU recording",
            imu.stop_recording,
        )
        arm_stop_status = _run_timed_step(
            timings,
            "Stop arm recording",
            arm.stop_recording,
        )

        arm_traj = _run_timed_step(timings, "Fetch arm trajectory", arm.get_recording)
        imu_traj = _run_timed_step(timings, "Fetch IMU trajectory", imu.get_recording)

        payload = {
            "arm_traj": arm_traj,
            "imu_traj": imu_traj,
            "q0": q0,
            "q_plus": q_plus,
            "q_minus": q_minus,
            "robot_ip": ROBOT_IP,
            "base_step_deg": BASE_STEP_DEG,
            "imu_recording_hz": configured_imu_hz,
            "pre_step_settle_s": PRE_STEP_SETTLE_S,
            "arm_stop_status": arm_stop_status,
            "imu_stop_status": imu_stop_status,
        }

        def _write_pickle() -> None:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with output_path.open("wb") as f:
                pickle.dump(payload, f)

        _run_timed_step(timings, "Write pickle", _write_pickle)

        print(f"Saved recording to {output_path}")
        print(f"Arm samples: {len(arm_traj)}")
        print(f"IMU samples: {len(imu_traj)}")
        _print_timing_summary(timings, time.perf_counter() - total_t0)

    finally:
        if arm_stop_status is None:
            try:
                arm.stop_recording()
            except Exception:
                pass
        if imu_stop_status is None:
            try:
                imu.stop_recording()
            except Exception:
                pass
        try:
            imu.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
