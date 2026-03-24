"""Calibrate the IMU time offset against the arm on one short motion.

Use this module in one of two ways:
- Import and call ``run_arm_imu_time_calibration(...)`` from Python.
- Run ``python -m robot.arm_imu_time_calibration`` from the terminal.

The calibration always starts from a fresh ``MuseIMUClient(time_offset_sec=0.0)``
and solves for the runtime ``time_offset_sec`` that should later be supplied
directly to the IMU client:

``imu = MuseIMUClient(time_offset_sec=result.time_offset_sec)``

Do not negate the fitted value. Use it exactly as returned.

Sign convention:
- positive ``time_offset_sec`` means the IMU timestamps should be moved later
- negative ``time_offset_sec`` means the IMU timestamps should be moved earlier
"""

from __future__ import annotations

import argparse
import datetime
import sys
import time
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from reforge_core.imu.client import MuseIMUClient
from reforge_core.imu.data import IMUStateTraj

from robot.arm_client import ArmClient, ArmStateTraj


ROBOT_IP = "192.168.1.208"
DEFAULT_POSITIVE_STEP_DEG = 5.0
DEFAULT_NEGATIVE_STEP_DEG = 5.0
DEFAULT_BIAS_CALIBRATION_SEC = 0.5
MAX_STEP_DEG = 10.0

IMU_RECORDING_HZ = 200.0
ARM_RECORDING_HZ = 250.0
POSITION_SPEED = 100.0


@dataclass(frozen=True, slots=True)
class ArmIMUTimeCalibrationResult:
    """Result returned by ``run_arm_imu_time_calibration(...)``."""

    positive_step_deg: float
    negative_step_deg: float
    bias_calibration_sec: float
    supplied_arg_names: frozenset[str]
    arm_client_injected: bool
    imu_bias: float
    time_offset_sec: float
    scale: float
    rmse: float
    q0: np.ndarray
    q_plus: np.ndarray
    q_minus: np.ndarray
    arm_traj: ArmStateTraj
    imu_traj: IMUStateTraj
    imu_bias_traj: IMUStateTraj


def run_motion_primitive(
    arm: ArmClient,
    q0: np.ndarray,
    *,
    positive_step_deg: float,
    negative_step_deg: float,
    position_speed: float = POSITION_SPEED,
) -> tuple[np.ndarray, np.ndarray]:
    """Run the calibration motion primitive.

    The trajectory is intentionally small and simple:
    ``q0 -> q_plus -> q_minus -> q0``.
    """
    # Build one positive and one negative base-joint step around the current pose.
    q_plus = q0.copy()
    q_minus = q0.copy()
    q_plus[0] = q0[0] + np.deg2rad(positive_step_deg)
    q_minus[0] = q0[0] - np.deg2rad(negative_step_deg)

    code_q_plus = arm.command_position(q_plus, speed=position_speed, wait=True)
    if code_q_plus != 0:
        raise RuntimeError(f"command_position(q_plus) returned code {code_q_plus}")

    code_q_minus = arm.command_position(q_minus, speed=position_speed, wait=True)
    if code_q_minus != 0:
        raise RuntimeError(f"command_position(q_minus) returned code {code_q_minus}")

    code_q0 = arm.command_position(q0, speed=position_speed, wait=True)
    if code_q0 != 0:
        raise RuntimeError(f"command_position(q0) returned code {code_q0}")

    return q_plus, q_minus


def estimate_imu_bias(
    imu: MuseIMUClient,
    duration_s: float,
) -> tuple[float, IMUStateTraj]:
    """Estimate scalar gyro-magnitude bias from a short stationary recording."""
    imu.start_recording()
    time.sleep(duration_s)
    imu.stop_recording()
    bias_traj = imu.get_recording()
    bias = estimate_imu_bias_from_traj(bias_traj)
    return bias, bias_traj


def estimate_imu_bias_from_traj(
    imu_traj: IMUStateTraj,
    stationary_duration_s: float | None = None,
) -> float:
    """Estimate scalar gyro-magnitude bias from one stationary IMU trajectory."""
    ts_imu, _, gyro = imu_traj.unpack()
    if ts_imu.size < 1:
        raise RuntimeError("Need at least one IMU sample to estimate bias.")

    if stationary_duration_s is None:
        stationary_mask = np.ones(ts_imu.shape, dtype=bool)
    else:
        stationary_mask = ts_imu <= float(ts_imu[0] + stationary_duration_s)

    if np.count_nonzero(stationary_mask) < 1:
        raise RuntimeError("Need at least one IMU sample in the stationary window.")
    return float(np.mean(np.linalg.norm(gyro[stationary_mask], axis=1)))


def solve_scale_and_time_offset(
    arm_traj: ArmStateTraj,
    imu_traj: IMUStateTraj,
    *,
    imu_bias: float,
) -> tuple[float, float, float]:
    """Solve scalar magnitude scale and time shift on the arm motion window."""
    # Unpack only the signals we actually use for calibration: arm base-joint
    # speed magnitude and IMU gyro magnitude, both against their own timestamps.
    ts_arm, _, qd_arm, _ = arm_traj.unpack()
    ts_imu, _, gyro = imu_traj.unpack()
    if ts_arm.size < 2:
        raise RuntimeError("Need at least two arm samples for time calibration.")
    if ts_imu.size < 2:
        raise RuntimeError("Need at least two IMU samples for time calibration.")

    arm_rate_mag = np.abs(qd_arm[:, 0])
    imu_gyro_mag = np.linalg.norm(gyro, axis=1)

    # Bias is estimated from the separate stationary IMU capture. After
    # subtracting it, clamp at zero so the magnitude signal stays nonnegative.
    imu_drift_fixed = np.maximum(imu_gyro_mag - imu_bias, 0.0)
    if float(np.max(imu_drift_fixed)) <= 0.0:
        raise RuntimeError("IMU signal is non-positive after bias removal.")

    # Use the arm signal to define where motion actually occurred. This keeps
    # the fit focused on the informative transient instead of long stationary
    # segments where almost everything is near zero.
    motion_threshold = 0.1 * float(np.max(arm_rate_mag))
    motion_mask = arm_rate_mag >= motion_threshold
    if np.count_nonzero(motion_mask) < 3:
        raise RuntimeError("Need at least three arm samples in the motion window.")

    motion_indices = np.flatnonzero(motion_mask)
    pad_samples = 3
    # Pad the arm-defined motion window slightly so the fit can use the local
    # rise/fall shoulders and not just the strict core above threshold.
    motion_start_idx = max(int(motion_indices[0]) - pad_samples, 0)
    motion_end_idx = min(
        int(motion_indices[-1]) + pad_samples + 1,
        ts_arm.size,
    )
    fit_ts_arm = ts_arm[motion_start_idx:motion_end_idx]
    fit_arm = arm_rate_mag[motion_start_idx:motion_end_idx]

    # Compare the two streams on a common time grid using the slower of the two
    # median sample intervals. That avoids inventing finer timing precision than
    # either source actually supports.
    dt_arm = float(np.median(np.diff(ts_arm)))
    dt_imu = float(np.median(np.diff(ts_imu)))
    dt_common = max(dt_arm, dt_imu)
    if dt_common <= 0.0:
        raise RuntimeError("Non-positive sample spacing encountered.")

    # Search the full feasible offset range implied by the two recorded spans.
    # Any larger shift would leave no overlap between the arm motion window and
    # the shifted IMU trace.
    min_time_offset_sec = float(fit_ts_arm[0] - ts_imu[-1])
    max_time_offset_sec = float(fit_ts_arm[-1] - ts_imu[0])
    time_offset_candidates = np.arange(
        min_time_offset_sec,
        max_time_offset_sec + dt_common,
        dt_common,
    )
    if time_offset_candidates.size == 0:
        raise RuntimeError("Could not construct any candidate time offsets.")

    min_overlap_samples = max(8, int(0.5 * fit_ts_arm.size))
    best_time_offset_sec = None
    best_scale = None
    best_rmse = None

    for time_offset_sec in time_offset_candidates:
        # Applying a positive time_offset_sec moves IMU timestamps later in time,
        # matching the deployed MuseIMUClient(time_offset_sec=...) convention.
        shifted_imu_times = ts_imu + time_offset_sec
        t_start = max(float(fit_ts_arm[0]), float(shifted_imu_times[0]))
        t_end = min(float(fit_ts_arm[-1]), float(shifted_imu_times[-1]))
        if t_end <= t_start:
            continue

        # Restrict scoring to the actual overlap for this candidate offset.
        common_t = np.arange(t_start, t_end, dt_common)
        if common_t.size < min_overlap_samples:
            continue

        # Resample both signals onto the same grid so the fit compares like with
        # like. The arm signal is the target; the IMU signal gets both shifted
        # and bias-corrected before the fit.
        arm_common = np.interp(common_t, fit_ts_arm, fit_arm)
        imu_common = np.interp(common_t, shifted_imu_times, imu_drift_fixed)
        imu_energy = float(np.dot(imu_common, imu_common))
        if imu_energy <= 0.0:
            continue

        # For a fixed time offset, the best scalar magnitude scale has a simple
        # closed-form least-squares solution. Clamp it nonnegative because both
        # signals are magnitudes.
        scale = float(np.dot(arm_common, imu_common) / imu_energy)
        scale = max(scale, 0.0)

        # Use RMSE on the overlap as the score for this candidate offset.
        rmse = float(np.sqrt(np.mean((arm_common - scale * imu_common) ** 2)))
        if best_rmse is None or rmse < best_rmse:
            best_time_offset_sec = float(time_offset_sec)
            best_scale = scale
            best_rmse = rmse

    if best_time_offset_sec is None or best_scale is None or best_rmse is None:
        raise RuntimeError("Could not solve for scale and time offset.")

    return best_time_offset_sec, best_scale, best_rmse


def plot_calibration(
    result: ArmIMUTimeCalibrationResult,
    *,
    block: bool = True,
):
    """Plot raw and aligned magnitude signals on absolute timestamps."""
    ts_arm, _, qd_arm, _ = result.arm_traj.unpack()
    ts_imu, _, gyro = result.imu_traj.unpack()
    arm_rate_mag = np.abs(qd_arm[:, 0])
    imu_gyro_mag = np.linalg.norm(gyro, axis=1)
    imu_corrected = np.maximum(imu_gyro_mag - result.imu_bias, 0.0) * result.scale

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), squeeze=False)
    axes_flat = axes[:, 0]

    axes_flat[0].plot(
        ts_arm,
        arm_rate_mag,
        color="tab:blue",
        linewidth=1.8,
        label=r"Arm $|\dot{q}_0|$",
    )
    axes_flat[0].plot(
        ts_imu,
        imu_gyro_mag,
        color="tab:orange",
        linestyle="--",
        linewidth=1.6,
        label=r"IMU $\|\omega\|$",
    )
    axes_flat[0].set_xlabel("Timestamp [s]")
    axes_flat[0].set_ylabel("Magnitude")
    axes_flat[0].set_title("Raw Overlay In Naive Time")
    axes_flat[0].grid(True, alpha=0.3)
    axes_flat[0].legend()

    axes_flat[1].plot(
        ts_arm,
        arm_rate_mag,
        color="tab:blue",
        linewidth=1.8,
        label=r"Arm $|\dot{q}_0|$",
    )
    axes_flat[1].plot(
        ts_imu + result.time_offset_sec,
        imu_corrected,
        color="tab:orange",
        linestyle="--",
        linewidth=1.6,
        label=(
            rf"IMU corrected $\|\omega\|$ "
            f"(time_offset_sec={result.time_offset_sec:.3f} s)"
        ),
    )
    axes_flat[1].set_xlabel("Timestamp [s]")
    axes_flat[1].set_ylabel("Magnitude")
    axes_flat[1].set_title(f"Aligned Overlay (RMSE = {result.rmse:.6f})")
    axes_flat[1].grid(True, alpha=0.3)
    axes_flat[1].legend()

    plt.tight_layout()
    plt.show(block=block)
    if not block:
        plt.pause(0.1)
    return fig


def format_calibration_summary_lines(result: ArmIMUTimeCalibrationResult) -> list[str]:
    """Format one small, deployment-oriented calibration summary."""
    return [
        f"Estimated time_offset_sec: {result.time_offset_sec:.6f}",
        f"Estimated imu_bias: {result.imu_bias:.6f}",
        f"Estimated scale: {result.scale:.6f}",
        f"Fit RMSE: {result.rmse:.6f}",
        "MuseIMUClient(time_offset_sec=0.0) assumed during calibration",
        (
            "Recommended MuseIMUClient"
            f"(time_offset_sec={result.time_offset_sec:.6f})"
        ),
        f"Recommended export IMU_TIME_OFFSET_SEC={result.time_offset_sec:.6f}",
    ]


def print_calibration_summary(result: ArmIMUTimeCalibrationResult) -> None:
    for line in format_calibration_summary_lines(result):
        print(line)


def run_arm_imu_time_calibration(
    *,
    positive_step_deg: float | None = None,
    negative_step_deg: float | None = None,
    bias_calibration_sec: float | None = None,
    arm_client: ArmClient | None = None,
    plot: bool = False,
    plot_block: bool = False,
) -> ArmIMUTimeCalibrationResult:
    """Run the full arm/IMU time calibration and return the fitted offset.

    This function always constructs a fresh ``MuseIMUClient(time_offset_sec=0.0)``
    and, at the beginning of the run, calls the IMU client's private
    ``_set_time_unsafe(...)`` helper so the device clock is aligned to host UTC
    before collecting calibration data.

    The returned ``result.time_offset_sec`` is the runtime value to pass directly
    into:

    ``MuseIMUClient(time_offset_sec=result.time_offset_sec)``

    Args:
        positive_step_deg: Positive base-joint step in degrees. Must be in
            ``(0, MAX_STEP_DEG]``. Defaults to ``DEFAULT_POSITIVE_STEP_DEG``.
        negative_step_deg: Negative base-joint step magnitude in degrees. Must
            be in ``(0, MAX_STEP_DEG]``. Defaults to
            ``DEFAULT_NEGATIVE_STEP_DEG``.
        bias_calibration_sec: Stationary IMU recording duration used to estimate
            gyro-magnitude bias before the motion starts. Must be ``> 0``.
            Defaults to ``DEFAULT_BIAS_CALIBRATION_SEC``.
        arm_client: Optional existing ``ArmClient`` to reuse. The IMU client is
            always constructed internally with zero initial offset.
        plot: When ``True``, also show the calibration plot.
        plot_block: Passed through to ``plot_calibration(...)`` when
            ``plot`` is ``True``.

    Returns:
        Returns ``ArmIMUTimeCalibrationResult`` with the fitted
        ``time_offset_sec``, scale, RMSE, estimated bias, recorded trajectories,
        and motion targets.

    Sign convention:
        Use the returned value directly. Do not negate it.
        If ``result.time_offset_sec`` is positive, pass that positive value to
        the runtime IMU client:

        ``MuseIMUClient(time_offset_sec=result.time_offset_sec)``
    """
    supplied_arg_names = frozenset(
        name
        for name, value in {
            "positive_step_deg": positive_step_deg,
            "negative_step_deg": negative_step_deg,
            "bias_calibration_sec": bias_calibration_sec,
        }.items()
        if value is not None
    )
    positive_step_deg = float(
        DEFAULT_POSITIVE_STEP_DEG
        if positive_step_deg is None
        else positive_step_deg
    )
    negative_step_deg = float(
        DEFAULT_NEGATIVE_STEP_DEG
        if negative_step_deg is None
        else negative_step_deg
    )
    bias_calibration_sec = float(
        DEFAULT_BIAS_CALIBRATION_SEC
        if bias_calibration_sec is None
        else bias_calibration_sec
    )
    if positive_step_deg <= 0.0:
        raise ValueError("positive_step_deg must be > 0.")
    if negative_step_deg <= 0.0:
        raise ValueError("negative_step_deg must be > 0.")
    if positive_step_deg > MAX_STEP_DEG:
        raise ValueError(f"positive_step_deg must be <= {MAX_STEP_DEG}.")
    if negative_step_deg > MAX_STEP_DEG:
        raise ValueError(f"negative_step_deg must be <= {MAX_STEP_DEG}.")
    if bias_calibration_sec <= 0.0:
        raise ValueError("bias_calibration_sec must be > 0.")

    arm_client_injected = arm_client is not None
    # Explicitly use zero time offset so we can recompute it from scratch
    imu_client = MuseIMUClient(time_offset_sec=0.0)
    if arm_client is None:
        arm_client = ArmClient(
            ROBOT_IP,
            recording_data_frequency_hz=ARM_RECORDING_HZ,
        )

    imu_client.connect()
    imu_client._set_time_unsafe(datetime.datetime.now(datetime.timezone.utc))

    imu_client.set_recording_data_frequency(IMU_RECORDING_HZ)
    arm_client.set_recording_data_frequency(ARM_RECORDING_HZ)

    imu_bias_traj: IMUStateTraj | None = None
    arm_traj: ArmStateTraj | None = None
    imu_traj: IMUStateTraj | None = None

    try:

        # While stationary, estimate the the IMU bias
        imu_bias, imu_bias_traj = estimate_imu_bias(
            imu_client,
            duration_s=bias_calibration_sec,
        )

        # Establish robot baseline at its current configuration
        q0 = arm_client.get_latest_state().q
        arm_client.set_position_mode()

        # Record data from IMU and arm while arm is excited with the motion primitive
        imu_client.start_recording()
        arm_client.start_recording()
        q_plus, q_minus = run_motion_primitive(
            arm_client,
            q0=q0,
            positive_step_deg=positive_step_deg,
            negative_step_deg=negative_step_deg,
        )
        imu_client.stop_recording()
        arm_client.stop_recording()

        arm_traj = arm_client.get_recording()
        imu_traj = imu_client.get_recording()

        # Use previously computed bias to solve to simultanously estimate time offset
        # and angular velocity magnitude scale on the motion primitive
        time_offset_sec, scale, rmse = solve_scale_and_time_offset(
            arm_traj,
            imu_traj,
            imu_bias=imu_bias,
        )

        result = ArmIMUTimeCalibrationResult(
            positive_step_deg=positive_step_deg,
            negative_step_deg=negative_step_deg,
            bias_calibration_sec=bias_calibration_sec,
            supplied_arg_names=supplied_arg_names,
            arm_client_injected=arm_client_injected,
            imu_bias=imu_bias,
            time_offset_sec=time_offset_sec,
            scale=scale,
            rmse=rmse,
            q0=q0.copy(),
            q_plus=q_plus,
            q_minus=q_minus,
            arm_traj=arm_traj,
            imu_traj=imu_traj,
            imu_bias_traj=imu_bias_traj,
        )
        if plot:
            plot_calibration(result, block=plot_block)
        return result
    finally:
        try:
            arm_client.stop_recording()
        except Exception:
            pass
        try:
            imu_client.stop_recording()
        except Exception:
            pass
        try:
            imu_client.disconnect()
        except Exception:
            pass


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Record one stationary IMU segment plus one short arm/IMU motion, "
            "then print the time_offset_sec to use directly in "
            "MuseIMUClient(time_offset_sec=...). Do not negate the printed value."
        )
    )
    parser.add_argument(
        "--positive-step-deg",
        type=float,
        default=None,
        help=(
            "Positive base-joint step magnitude in degrees. Must be > 0 and "
            f"<= {MAX_STEP_DEG}. Default: {DEFAULT_POSITIVE_STEP_DEG}."
        ),
    )
    parser.add_argument(
        "--negative-step-deg",
        type=float,
        default=None,
        help=(
            "Negative base-joint step magnitude in degrees. Must be > 0 and "
            f"<= {MAX_STEP_DEG}. Default: {DEFAULT_NEGATIVE_STEP_DEG}."
        ),
    )
    parser.add_argument(
        "--bias-calibration-sec",
        type=float,
        default=None,
        help=(
            "Stationary IMU recording duration in seconds used to estimate "
            f"bias before the motion. Default: {DEFAULT_BIAS_CALIBRATION_SEC}."
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint for one interactive arm/IMU calibration run.

    CLI arguments:
        --positive-step-deg:
            Positive base-joint step magnitude in degrees.
        --negative-step-deg:
            Negative base-joint step magnitude in degrees.
        --bias-calibration-sec:
            Stationary IMU recording duration used for bias estimation before
            the motion segment.

    Output:
        Prints the fitted ``time_offset_sec`` and shows a raw/aligned plot.
        The printed export command is the exact value to use directly in:

        ``MuseIMUClient(time_offset_sec=...)``

        Do not negate the printed value.
    """
    args = _build_arg_parser().parse_args(argv)

    result = run_arm_imu_time_calibration(
        positive_step_deg=args.positive_step_deg,
        negative_step_deg=args.negative_step_deg,
        bias_calibration_sec=args.bias_calibration_sec,
    )

    print_calibration_summary(result)

    fig = plot_calibration(result, block=False)

    if sys.stdin.isatty():
        print("\nCopy/paste this into your shell:\n")
        print(f"  export IMU_TIME_OFFSET_SEC={result.time_offset_sec:.9f}\n")
        input("Press Enter when you are done reviewing the plot...")

    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
