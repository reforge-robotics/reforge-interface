"""Parse and inspect the scratch arm/IMU recording pickle."""

from __future__ import annotations

import pickle
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from robot import arm_client as arm_client_module
from robot.arm_client import ArmState, ArmStateTraj
from robot.scratch.arm_imu_sync import PRE_STEP_SETTLE_S


# Preserve compatibility with older pickles that referenced these classes via
# either `arm_client` or `__main__`.
sys.modules.setdefault("arm_client", arm_client_module)
main_module = sys.modules.get("__main__")
if main_module is not None:
    setattr(main_module, "ArmState", ArmState)
    setattr(main_module, "ArmStateTraj", ArmStateTraj)


INPUT_DIR = Path(__file__).with_name("syncdata")
MAX_ALIGNMENT_SHIFT_S = 2.0
INITIAL_IMU_STATIONARY_S = PRE_STEP_SETTLE_S


@dataclass(frozen=True, slots=True)
class ProcessedSignals:
    ts_arm: np.ndarray
    arm_rate_mag: np.ndarray
    ts_imu: np.ndarray
    imu_gyro_mag: np.ndarray


@dataclass(frozen=True, slots=True)
class AlignmentResult:
    method: str
    lag_s: float
    ts_imu_in_arm_time: np.ndarray
    robot_signal: np.ndarray
    imu_signal: np.ndarray
    robot_label: str
    imu_label: str
    score_name: str
    score_value: float
    scale_factor: float | None = None
    baseline_offset: float | None = None


def _find_latest_input_path() -> Path:
    candidate_paths = sorted(INPUT_DIR.glob("arm_imu_recording_*.pkl"))
    if not candidate_paths:
        raise FileNotFoundError(f"No recording pickle found in {INPUT_DIR}")
    return candidate_paths[-1]


def _extract_processed_signals(payload: dict) -> ProcessedSignals:
    arm_traj = payload["arm_traj"]
    imu_traj = payload["imu_traj"]

    ts_arm, _, qd_arm, _ = arm_traj.unpack()
    ts_imu, _, gyro = imu_traj.unpack()

    if ts_arm.size < 2:
        raise RuntimeError("Need at least 2 arm samples to estimate lag.")
    if ts_imu.size < 2:
        raise RuntimeError("Need at least 2 IMU samples to estimate lag.")

    return ProcessedSignals(
        ts_arm=ts_arm,
        arm_rate_mag=np.abs(qd_arm[:, 0]),
        ts_imu=ts_imu,
        imu_gyro_mag=np.linalg.norm(gyro, axis=1),
    )


def estimate_alignment_calibrated_shift(signals: ProcessedSignals) -> AlignmentResult:
    """Estimate lag after separating stationary calibration from motion fitting.

    Assumptions:
    - The first ~1 second of IMU samples is stationary.
    - One scalar baseline correction on IMU angular-rate magnitude is sufficient.
    - One scalar gain is sufficient after baseline correction.
    - Time shift and gain should be fit only on the informative motion segment,
      not on the full trajectory dominated by stationary samples.
    """

    stationary_mask = signals.ts_imu <= float(
        signals.ts_imu[0] + INITIAL_IMU_STATIONARY_S
    )
    if np.count_nonzero(stationary_mask) < 2:
        raise RuntimeError(
            "Need at least two IMU samples in the initial stationary window."
        )

    baseline_offset = float(np.mean(signals.imu_gyro_mag[stationary_mask]))
    imu_drift_fixed = np.maximum(signals.imu_gyro_mag - baseline_offset, 0.0)

    if float(np.max(imu_drift_fixed)) <= 0.0:
        raise RuntimeError("IMU signal is non-positive after baseline removal.")

    motion_threshold = 0.1 * float(np.max(signals.arm_rate_mag))
    motion_mask = signals.arm_rate_mag >= motion_threshold
    if np.count_nonzero(motion_mask) < 3:
        raise RuntimeError("Need at least three arm samples in the motion window.")

    # Pad the detected motion window slightly so the fitted lag can still use the
    # local rise/fall shoulders instead of only the strict high-amplitude core.
    motion_indices = np.flatnonzero(motion_mask)
    pad_samples = 3
    motion_start_idx = max(int(motion_indices[0]) - pad_samples, 0)
    motion_end_idx = min(
        int(motion_indices[-1]) + pad_samples + 1,
        signals.ts_arm.size,
    )
    fit_ts_arm = signals.ts_arm[motion_start_idx:motion_end_idx]
    fit_arm = signals.arm_rate_mag[motion_start_idx:motion_end_idx]

    dt_arm = float(np.median(np.diff(signals.ts_arm)))
    dt_imu = float(np.median(np.diff(signals.ts_imu)))
    dt_common = max(dt_arm, dt_imu)
    if dt_common <= 0.0:
        raise RuntimeError("Non-positive sample spacing encountered.")

    lag_candidates = np.arange(
        -MAX_ALIGNMENT_SHIFT_S,
        MAX_ALIGNMENT_SHIFT_S + dt_common,
        dt_common,
    )
    if lag_candidates.size == 0:
        raise RuntimeError("Could not construct any candidate lags.")

    min_overlap_samples = max(8, int(0.5 * fit_ts_arm.size))

    best_lag_s = None
    best_mse = None
    best_scale_factor = None

    for lag_s in lag_candidates:
        shifted_imu_times = signals.ts_imu - lag_s
        t_start = max(float(fit_ts_arm[0]), float(shifted_imu_times[0]))
        t_end = min(float(fit_ts_arm[-1]), float(shifted_imu_times[-1]))
        if t_end <= t_start:
            continue

        common_t = np.arange(t_start, t_end, dt_common)
        if common_t.size < min_overlap_samples:
            continue

        arm_common = np.interp(common_t, fit_ts_arm, fit_arm)
        imu_common = np.interp(common_t, shifted_imu_times, imu_drift_fixed)
        imu_energy = float(np.dot(imu_common, imu_common))
        if imu_energy <= 0.0:
            continue

        # Solve the best nonnegative scalar gain for this lag in closed form.
        scale_factor = float(np.dot(arm_common, imu_common) / imu_energy)
        scale_factor = max(scale_factor, 0.0)
        residual = arm_common - scale_factor * imu_common
        mse = float(np.mean(residual**2))

        if best_mse is None or mse < best_mse:
            best_mse = mse
            best_lag_s = float(lag_s)
            best_scale_factor = scale_factor

    if (
        best_lag_s is None
        or best_mse is None
        or best_scale_factor is None
    ):
        raise RuntimeError("Calibrated shift alignment could not find a valid lag.")

    imu_scaled = imu_drift_fixed * best_scale_factor

    return AlignmentResult(
        method="calibrated_shift",
        lag_s=best_lag_s,
        ts_imu_in_arm_time=signals.ts_imu - best_lag_s,
        robot_signal=signals.arm_rate_mag,
        imu_signal=imu_scaled,
        robot_label=r"Arm $|\dot{q}_0|$",
        imu_label=r"IMU corrected $\|\omega\|$",
        score_name="fit RMSE",
        score_value=float(np.sqrt(best_mse)),
        scale_factor=best_scale_factor,
        baseline_offset=baseline_offset,
    )


def _plot_signals(
    signals: ProcessedSignals,
    alignment_result: AlignmentResult,
) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), squeeze=False)
    axes_flat = axes[:, 0]

    axes_flat[0].plot(
        signals.ts_arm,
        signals.arm_rate_mag,
        color="tab:blue",
        linewidth=1.8,
        label=r"Arm $|\dot{q}_0|$",
    )
    axes_flat[0].plot(
        signals.ts_imu,
        signals.imu_gyro_mag,
        color="tab:orange",
        linewidth=1.6,
        linestyle="--",
        label=r"IMU $\|\omega\|$",
    )
    axes_flat[0].set_xlabel("Timestamp [s]")
    axes_flat[0].set_ylabel("Magnitude")
    axes_flat[0].set_title("Raw Overlay In Naive Time")
    axes_flat[0].grid(True, alpha=0.3)
    axes_flat[0].legend()

    axes_flat[1].plot(
        signals.ts_arm,
        alignment_result.robot_signal,
        color="tab:blue",
        linewidth=1.8,
        label=alignment_result.robot_label,
    )
    axes_flat[1].plot(
        alignment_result.ts_imu_in_arm_time,
        alignment_result.imu_signal,
        linewidth=1.6,
        linestyle="--",
        color="tab:orange",
        label=(
            f"{alignment_result.imu_label} "
            f"({alignment_result.method}, lag={alignment_result.lag_s:.3f} s)"
        ),
    )
    axes_flat[1].set_xlabel("Timestamp [s]")
    axes_flat[1].set_ylabel("Magnitude")
    axes_flat[1].set_title(
        f"Adjusted IMU Signal In Robot Time ({alignment_result.method}: "
        f"{alignment_result.score_name} = {alignment_result.score_value:.6f})"
    )
    axes_flat[1].grid(True, alpha=0.3)
    axes_flat[1].legend()

    plt.tight_layout()
    plt.show()


def main() -> None:
    input_path = _find_latest_input_path()
    print(f"Loading recording from {input_path}")

    with input_path.open("rb") as f:
        payload = pickle.load(f)

    signals = _extract_processed_signals(payload)
    q0 = payload["q0"]
    q_plus = payload.get("q_plus")
    q_minus = payload.get("q_minus")
    q1 = payload.get("q1")
    alignment_result = estimate_alignment_calibrated_shift(signals)

    print(f"q0[0] = {q0[0]:.6f} rad")
    if q_plus is not None:
        print(f"q_plus[0] = {q_plus[0]:.6f} rad")
    if q_minus is not None:
        print(f"q_minus[0] = {q_minus[0]:.6f} rad")
    if q_plus is None and q_minus is None and q1 is not None:
        print(f"q1[0] = {q1[0]:.6f} rad")
    print(f"Arm samples = {signals.ts_arm.size}")
    print(f"IMU samples = {signals.ts_imu.size}")
    print(f"[{alignment_result.method}] {alignment_result.score_name} = {alignment_result.score_value}")
    print(
        f"[{alignment_result.method}] estimated lag (IMU relative to arm) = "
        f"{alignment_result.lag_s} s"
    )
    if alignment_result.baseline_offset is not None:
        print(
            f"[{alignment_result.method}] IMU baseline offset = "
            f"{alignment_result.baseline_offset}"
        )
    if alignment_result.scale_factor is not None:
        print(
            f"[{alignment_result.method}] IMU scale factor = "
            f"{alignment_result.scale_factor}"
        )

    _plot_signals(signals, alignment_result)


if __name__ == "__main__":
    main()
