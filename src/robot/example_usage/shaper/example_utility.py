"""Utility functions for the Covalent Shaper example.

The functions in this file keep trajectory generation, simulation, and plotting
out of `shaper_example_usage.py` so the main example can focus on Shaper usage.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, Sequence, cast

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

from reforge_core.control.shaper import ResidualSwitchLimits, RobotState


SLOWER_REFERENCE_MOVE_DURATION_S = 0.6
EXAMPLE_1_LABEL = "Example 1: Shape a full known trajectory"
EXAMPLE_2_LABEL = "Example 2: Shape the final part of a known trajectory"
EXAMPLE_3_LABEL = "Example 3: Shape a known trajectory in fixed windows"
EXAMPLE_4_LABEL = "Example 4: Shape a sample-by-sample stream"


def make_speed_first_switch_limits(
    *,
    max_velocity_rad_s: float | Sequence[float],
    max_acceleration_rad_s2: float | Sequence[float],
    max_search_s: float,
    max_qp_attempts: int,
    window_target_margin_s: float = 0.0,
) -> ResidualSwitchLimits:
    """Build a bounded-search profile with compatibility objective weights.

    The physical limits and search budget must be qualified for the target
    robot, payload, controller, and representative trajectories.

    Args:
        max_velocity_rad_s: Symmetric joint velocity limits [rad/s].
        max_acceleration_rad_s2: Symmetric joint acceleration limits [rad/s^2].
        max_search_s: Maximum switch lookback duration [s].
        max_qp_attempts: Maximum QP solves in the candidate search.
        window_target_margin_s: Windowed attach-point safety margin [s].

    Returns:
        Constrained residual-switch settings favoring bounded search time.
    """

    return ResidualSwitchLimits(
        max_velocity=max_velocity_rad_s,
        max_acceleration=max_acceleration_rad_s2,
        max_search_s=max_search_s,
        max_qp_attempts=max_qp_attempts,
        window_target_margin_s=window_target_margin_s,
        tracking_weight=1.0,
        acceleration_weight=1.0,
        jerk_weight=0.01,
    )


def make_smoothness_first_switch_limits(
    *,
    max_velocity_rad_s: float | Sequence[float],
    max_acceleration_rad_s2: float | Sequence[float],
    tracking_weight: float,
    acceleration_weight: float,
    jerk_weight: float,
) -> ResidualSwitchLimits:
    """Build a profile from application-qualified smoothness weights.

    Select the dimensionless objective weights with a representative trajectory
    sweep instead of treating one robot's tuning as a reusable default.

    Args:
        max_velocity_rad_s: Symmetric joint velocity limits [rad/s].
        max_acceleration_rad_s2: Symmetric joint acceleration limits [rad/s^2].
        tracking_weight: Dimensionless raw-to-shaped tracking weight.
        acceleration_weight: Dimensionless acceleration regularization weight.
        jerk_weight: Dimensionless jerk regularization weight.

    Returns:
        Constrained residual-switch settings using the selected smoothness tune.
    """

    return ResidualSwitchLimits(
        max_velocity=max_velocity_rad_s,
        max_acceleration=max_acceleration_rad_s2,
        tracking_weight=tracking_weight,
        acceleration_weight=acceleration_weight,
        jerk_weight=jerk_weight,
    )


def make_jerk_limited_switch_limits(
    *,
    max_velocity_rad_s: float | Sequence[float],
    max_acceleration_rad_s2: float | Sequence[float],
    max_jerk_rad_s3: float | Sequence[float],
) -> ResidualSwitchLimits:
    """Build a profile with application-qualified joint-space hard limits.

    Args:
        max_velocity_rad_s: Symmetric joint velocity limits [rad/s].
        max_acceleration_rad_s2: Symmetric joint acceleration limits [rad/s^2].
        max_jerk_rad_s3: Symmetric joint jerk limits [rad/s^3].

    Returns:
        Constrained residual-switch settings with a hard jerk limit.
    """

    return ResidualSwitchLimits(
        max_velocity=max_velocity_rad_s,
        max_acceleration=max_acceleration_rad_s2,
        max_jerk=max_jerk_rad_s3,
    )


@dataclass(frozen=True)
class ModalPlant:
    """Represent one dominant flexible mode used in the example simulation.

    Args:
        natural_frequency_rad_s: Natural frequency of the flexible mode [rad/s].
        damping_ratio: Modal damping ratio [unitless].
    Returns:
        `None`.
    Raises:
        None.
    """

    natural_frequency_rad_s: float
    damping_ratio: float


@dataclass(frozen=True)
class SimulatedResponse:
    """Store the command and plant response for one simulated trajectory.

    Args:
        time_s: Sample times [s].
        command_rad: Input joint-position command [rad].
        response_rad: Simulated joint-position response [rad].
    Returns:
        `None`.
    Raises:
        None.
    """

    time_s: np.ndarray
    command_rad: np.ndarray
    response_rad: np.ndarray


class WindowedTrajectorySample(Protocol):
    """Describe the shaped window interface returned by the example buffer.

    Args:
        None.
    Returns:
        `None`.
    Raises:
        None.
    """

    time: np.ndarray
    positions: np.ndarray


def concatenate_windowed_outputs(
    windows: list[WindowedTrajectorySample],
) -> tuple[np.ndarray, np.ndarray]:
    """Concatenate shaped windows into trajectory arrays.

    Args:
        windows: Shaped windows returned from `WindowedTrajectoryBuffer.pop_window()`.
    Returns:
        `tuple[np.ndarray, np.ndarray]`: Time [s] and positions [rad].
    Raises:
        ValueError: If no windows were emitted.
    """

    if not windows:
        raise ValueError("At least one shaped window is required.")
    return (
        np.concatenate([window.time for window in windows]),
        np.vstack([window.positions for window in windows]),
    )


def generate_point_to_point_trajectory(
    start_position_rad: np.ndarray,
    goal_position_rad: np.ndarray,
    sample_time_s: float,
    move_duration_s: float,
    dwell_duration_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a smooth point-to-point joint trajectory.

    Args:
        start_position_rad: Initial joint positions [rad], shape `[num_joints]`.
        goal_position_rad: Final joint positions [rad], shape `[num_joints]`.
        sample_time_s: Controller sample period [s].
        move_duration_s: Duration of the point-to-point move [s].
        dwell_duration_s: Final-target dwell duration [s].
    Returns:
        `tuple[np.ndarray, np.ndarray]`: Time vector `[N]` [s] and joint
        trajectory `[N, num_joints]` [rad].
    Raises:
        ValueError: If the vectors have different shapes or durations are invalid.
    """

    start_position_rad = np.asarray(start_position_rad, dtype=float)
    goal_position_rad = np.asarray(goal_position_rad, dtype=float)
    if start_position_rad.shape != goal_position_rad.shape:
        raise ValueError("start_position_rad and goal_position_rad must match")
    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive")
    if move_duration_s <= 0.0:
        raise ValueError("move_duration_s must be positive")
    if dwell_duration_s < 0.0:
        raise ValueError("dwell_duration_s must be nonnegative")

    move_samples = int(round(move_duration_s / sample_time_s)) + 1
    dwell_samples = int(round(dwell_duration_s / sample_time_s))
    alpha = np.linspace(0.0, 1.0, move_samples)
    blend = 10.0 * alpha**3 - 15.0 * alpha**4 + 6.0 * alpha**5

    move_segment = start_position_rad + np.outer(
        blend,
        goal_position_rad - start_position_rad,
    )
    if dwell_samples > 0:
        dwell_segment = np.repeat(
            goal_position_rad.reshape(1, -1),
            dwell_samples,
            axis=0,
        )
        trajectory_rad = np.vstack((move_segment, dwell_segment))
    else:
        trajectory_rad = move_segment

    time_s = np.arange(trajectory_rad.shape[0], dtype=float) * sample_time_s
    return time_s, trajectory_rad


def generate_point_to_point_sample(
    start_position_rad: np.ndarray,
    goal_position_rad: np.ndarray,
    current_time_s: float,
    move_duration_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Generate one online point-to-point command sample.

    Args:
        start_position_rad: Initial joint positions [rad], shape `[num_joints]`.
        goal_position_rad: Final joint positions [rad], shape `[num_joints]`.
        current_time_s: Current control-loop time from the move start [s].
        move_duration_s: Duration of the point-to-point move [s].
    Returns:
        `tuple[np.ndarray, np.ndarray, np.ndarray]`: Position [rad], velocity
        [rad/s], and acceleration [rad/s^2] command vectors for the current sample.
    Raises:
        ValueError: If the vectors have different shapes or `move_duration_s`
            is not positive.
    """

    start_position_rad = np.asarray(start_position_rad, dtype=float)
    goal_position_rad = np.asarray(goal_position_rad, dtype=float)
    if start_position_rad.shape != goal_position_rad.shape:
        raise ValueError("start_position_rad and goal_position_rad must match")
    if move_duration_s <= 0.0:
        raise ValueError("move_duration_s must be positive")

    if current_time_s <= 0.0:
        phase = 0.0
    elif current_time_s >= move_duration_s:
        phase = 1.0
    else:
        phase = current_time_s / move_duration_s

    blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    blend_dot = 30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
    blend_ddot = 60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3
    if phase == 0.0 or phase == 1.0:
        blend_dot = 0.0
        blend_ddot = 0.0

    move_delta_rad = goal_position_rad - start_position_rad
    command_rad = start_position_rad + blend * move_delta_rad
    velocity_rad_s = (blend_dot / move_duration_s) * move_delta_rad
    acceleration_rad_s2 = (blend_ddot / move_duration_s**2) * move_delta_rad
    return command_rad, velocity_rad_s, acceleration_rad_s2


def generate_smooth_trapezoidal_point_to_point_trajectory(
    start_position_rad: np.ndarray,
    goal_position_rad: np.ndarray,
    sample_time_s: float,
    move_duration_s: float,
    acceleration_duration_s: float,
    final_deceleration_duration_s: float,
    dwell_duration_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a smooth acceleration-cruise-deceleration joint trajectory.

    Args:
        start_position_rad: Initial joint positions [rad], shape `[num_joints]`.
        goal_position_rad: Final joint positions [rad], shape `[num_joints]`.
        sample_time_s: Controller sample period [s].
        move_duration_s: Duration of the point-to-point move [s].
        acceleration_duration_s: Smooth initial acceleration duration [s].
        final_deceleration_duration_s: Smooth final deceleration duration [s].
        dwell_duration_s: Final-target dwell duration [s].

    Returns:
        `tuple[np.ndarray, np.ndarray]`: Time vector `[N]` [s] and joint
        trajectory `[N, num_joints]` [rad].

    Raises:
        ValueError: If vectors have different shapes or durations are invalid.
    """

    start_position_rad = np.asarray(start_position_rad, dtype=float)
    goal_position_rad = np.asarray(goal_position_rad, dtype=float)
    if start_position_rad.shape != goal_position_rad.shape:
        raise ValueError("start_position_rad and goal_position_rad must match")
    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive")
    if move_duration_s <= 0.0:
        raise ValueError("move_duration_s must be positive")
    if acceleration_duration_s <= 0.0:
        raise ValueError("acceleration_duration_s must be positive")
    if final_deceleration_duration_s <= 0.0:
        raise ValueError("final_deceleration_duration_s must be positive")
    if acceleration_duration_s + final_deceleration_duration_s >= move_duration_s:
        raise ValueError(
            "acceleration and final-deceleration durations must fit inside the move"
        )
    if dwell_duration_s < 0.0:
        raise ValueError("dwell_duration_s must be nonnegative")

    move_samples = int(round(move_duration_s / sample_time_s)) + 1
    dwell_samples = int(round(dwell_duration_s / sample_time_s))
    move_time_s = np.arange(move_samples, dtype=float) * sample_time_s
    final_deceleration_start_time_s = move_duration_s - final_deceleration_duration_s
    cruise_duration_s = final_deceleration_start_time_s - acceleration_duration_s
    normalized_cruise_speed = 1.0 / (
        cruise_duration_s
        + 0.5 * (acceleration_duration_s + final_deceleration_duration_s)
    )

    normalized_position = np.empty_like(move_time_s)
    acceleration_distance = 0.5 * normalized_cruise_speed * acceleration_duration_s
    cruise_start_distance = acceleration_distance
    final_deceleration_start_distance = normalized_cruise_speed * (
        0.5 * acceleration_duration_s + cruise_duration_s
    )

    # Each move sample follows acceleration, cruise, or final-deceleration timing.
    for sample_index, sample_time_s_i in enumerate(move_time_s):
        if sample_time_s_i <= acceleration_duration_s:
            phase = sample_time_s_i / acceleration_duration_s
            normalized_position[sample_index] = (
                normalized_cruise_speed
                * acceleration_duration_s
                * _smoothstep_integral(phase)
            )
        elif sample_time_s_i <= final_deceleration_start_time_s:
            normalized_position[sample_index] = cruise_start_distance + (
                normalized_cruise_speed * (sample_time_s_i - acceleration_duration_s)
            )
        else:
            phase = min(
                (sample_time_s_i - final_deceleration_start_time_s)
                / final_deceleration_duration_s,
                1.0,
            )
            normalized_position[sample_index] = final_deceleration_start_distance + (
                normalized_cruise_speed
                * final_deceleration_duration_s
                * (phase - _smoothstep_integral(phase))
            )

    normalized_position[-1] = 1.0
    move_segment = start_position_rad + np.outer(
        normalized_position,
        goal_position_rad - start_position_rad,
    )
    if dwell_samples > 0:
        dwell_segment = np.repeat(
            goal_position_rad.reshape(1, -1),
            dwell_samples,
            axis=0,
        )
        trajectory_rad = np.vstack((move_segment, dwell_segment))
    else:
        trajectory_rad = move_segment

    time_s = np.arange(trajectory_rad.shape[0], dtype=float) * sample_time_s
    return time_s, trajectory_rad


def _smoothstep_integral(phase: float) -> float:
    """Integrate the quintic smootherstep velocity profile.

    Args:
        phase: Normalized phase in `[0.0, 1.0]`.

    Returns:
        `float`: Integral of `10*x^3 - 15*x^4 + 6*x^5` from zero to `phase`.

    Raises:
        None.
    """

    return 2.5 * phase**4 - 3.0 * phase**5 + phase**6


def estimate_derivatives(
    trajectory_rad: np.ndarray,
    sample_time_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate joint velocity and acceleration from position samples.

    Args:
        trajectory_rad: Joint-position trajectory `[N, num_joints]` [rad].
        sample_time_s: Controller sample period [s].
    Returns:
        `tuple[np.ndarray, np.ndarray]`: Joint velocities `[N, num_joints]`
        [rad/s] and accelerations `[N, num_joints]` [rad/s^2].
    Raises:
        ValueError: If `trajectory_rad` is not a two-dimensional array or if
            `sample_time_s` is not positive.
    """

    trajectory_rad = np.asarray(trajectory_rad, dtype=float)
    if trajectory_rad.ndim != 2:
        raise ValueError("trajectory_rad must be two-dimensional")
    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive")

    edge_order = 2 if trajectory_rad.shape[0] > 2 else 1
    # Nosa: NumPy's current gradient stubs do not fully model this validated
    # 2D `ndarray` usage, so keep the runtime call and narrow the static type.
    velocity_rad_s = cast(
        np.ndarray,
        np.gradient(  # type: ignore[call-overload]
            trajectory_rad,
            sample_time_s,
            axis=0,
            edge_order=edge_order,
        ),
    )
    acceleration_rad_s2 = cast(
        np.ndarray,
        np.gradient(  # type: ignore[call-overload]
            velocity_rad_s,
            sample_time_s,
            axis=0,
            edge_order=edge_order,
        ),
    )
    return velocity_rad_s, acceleration_rad_s2


def infer_dominant_modal_plant(
    shaper: Any,
    representative_command_rad: np.ndarray,
    axis_index: int,
) -> ModalPlant:
    """Infer the least-damped simulation mode from the Shaper model.

    Args:
        shaper: Initialized `ShaperInterface`.
        representative_command_rad: Joint command used to evaluate the map model
            [rad], shape `[num_joints]`.
        axis_index: Shaped axis index used for the simulated plant.
    Returns:
        `ModalPlant`: Least-damped mode used by the example plant.
    Raises:
        ValueError: If `axis_index` is outside the configured shaped axes.
    """

    representative_command_rad = np.asarray(representative_command_rad, dtype=float)
    state = RobotState(
        joint_angles=representative_command_rad,
        tcp_position=shaper.compute_forward_kinematics(representative_command_rad),
    )
    fitted_axes = shaper.infer_fitted_modes(state)
    if axis_index < 0 or axis_index >= len(fitted_axes):
        raise ValueError("axis_index must be inside the configured shaped axes")

    dominant_mode = min(
        fitted_axes[axis_index].modes,
        key=lambda mode: mode.damping_ratio,
    )
    return ModalPlant(
        natural_frequency_rad_s=dominant_mode.natural_frequency_rad_s,
        damping_ratio=dominant_mode.damping_ratio,
    )


def simulate_modal_position_response(
    time_s: np.ndarray,
    command_rad: np.ndarray,
    plant: ModalPlant,
) -> SimulatedResponse:
    """Simulate a unity-gain second-order joint-position plant.

    Args:
        time_s: Sample times [s].
        command_rad: Input joint-position command [rad].
        plant: Modal plant parameters used by the simulation.
    Returns:
        `SimulatedResponse`: Time, command, and simulated response arrays.
    Raises:
        ValueError: If input arrays have different lengths or invalid dimensions.
    """

    time_s = np.asarray(time_s, dtype=float)
    command_rad = np.asarray(command_rad, dtype=float)
    if time_s.ndim != 1 or command_rad.ndim != 1:
        raise ValueError("time_s and command_rad must be one-dimensional")
    if time_s.shape[0] != command_rad.shape[0]:
        raise ValueError("time_s and command_rad must have the same length")

    wn = plant.natural_frequency_rad_s
    zeta = plant.damping_ratio
    system = signal.TransferFunction(
        [wn**2],
        [1.0, 2.0 * zeta * wn, wn**2],
    )
    _, response_rad, _ = signal.lsim(system, U=command_rad, T=time_s)
    return SimulatedResponse(
        time_s=time_s,
        command_rad=command_rad,
        response_rad=np.asarray(response_rad, dtype=float),
    )


def residual_vibration_rad(
    response: SimulatedResponse,
    final_value_rad: float,
    start_time_s: float,
) -> float:
    """Compute residual vibration after the commanded move finishes.

    Args:
        response: Simulated response.
        final_value_rad: Expected final joint position [rad].
        start_time_s: Time after which residual vibration is measured [s].
    Returns:
        `float`: Maximum absolute response error after `start_time_s` [rad].
    Raises:
        None.
    """

    mask = response.time_s >= start_time_s
    if not np.any(mask):
        return 0.0
    error_rad = response.response_rad[mask] - final_value_rad
    return float(np.max(np.abs(error_rad)))


def _sample_time_from_time_vector(time_s: np.ndarray) -> float:
    """Estimate the sample period from a monotonic time vector.

    Args:
        time_s: Sample times `[N]` [s].
    Returns:
        `float`: Median sample period [s].
    Raises:
        ValueError: If `time_s` is not one-dimensional, has fewer than two
            samples, or is not strictly increasing.
    """

    time_s = np.asarray(time_s, dtype=float)
    if time_s.ndim != 1:
        raise ValueError("time_s must be one-dimensional")
    if time_s.shape[0] < 2:
        raise ValueError("time_s must contain at least two samples")

    sample_intervals_s = np.diff(time_s)
    if np.any(sample_intervals_s <= 0.0):
        raise ValueError("time_s must be strictly increasing")
    return float(np.median(sample_intervals_s))


def _axis_position_velocity_acceleration(
    time_s: np.ndarray,
    positions_rad: np.ndarray,
    axis_index: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute position, velocity, and acceleration for one joint axis.

    Args:
        time_s: Sample times `[N]` [s].
        positions_rad: Joint-position samples `[N, num_joints]` [rad].
        axis_index: Joint axis used for the returned profiles.
    Returns:
        `tuple[np.ndarray, np.ndarray, np.ndarray]`: Position [rad], velocity
        [rad/s], and acceleration [rad/s^2] for `axis_index`.
    Raises:
        ValueError: If the position array shape is invalid or `axis_index` is
            outside the available joint axes.
    """

    positions_rad = np.asarray(positions_rad, dtype=float)
    if positions_rad.ndim != 2:
        raise ValueError("positions_rad must be two-dimensional")
    if axis_index < 0 or axis_index >= positions_rad.shape[1]:
        raise ValueError("axis_index must be inside the available joint axes")

    velocity_rad_s, acceleration_rad_s2 = estimate_derivatives(
        positions_rad,
        _sample_time_from_time_vector(time_s),
    )
    return (
        positions_rad[:, axis_index],
        velocity_rad_s[:, axis_index],
        acceleration_rad_s2[:, axis_index],
    )


def _streamed_samples_to_arrays(
    time_s: Sequence[float],
    positions_rad: Sequence[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Convert streamed Shaper samples into aligned NumPy arrays.

    Args:
        time_s: Streamed sample times [s].
        positions_rad: Streamed joint-position samples [rad].
    Returns:
        `tuple[np.ndarray, np.ndarray]`: Time vector `[N]` [s] and joint
        positions `[N, num_joints]` [rad].
    Raises:
        ValueError: If no samples were provided or time and position sample
            counts differ.
    """

    time_array_s = np.asarray(time_s, dtype=float)
    if len(positions_rad) == 0:
        raise ValueError("positions_rad must contain at least one sample")

    position_array_rad = np.vstack(positions_rad)
    if time_array_s.shape[0] != position_array_rad.shape[0]:
        raise ValueError("time_s and positions_rad must have the same sample count")
    return time_array_s, position_array_rad


def _final_deceleration_start_time(
    time_s: np.ndarray,
    positions_rad: np.ndarray,
    shaped_axis: int,
) -> float:
    """Estimate when a point-to-point command starts final deceleration.

    Args:
        time_s: Command sample times `[N]` [s].
        positions_rad: Joint-position samples `[N, num_joints]` [rad].
        shaped_axis: Joint axis used to find the peak command velocity.
    Returns:
        `float`: Time when velocity reaches its maximum before final deceleration [s].
    Raises:
        ValueError: If the trajectory has invalid dimensions.
    """

    velocity_rad_s, _ = estimate_derivatives(
        positions_rad,
        _sample_time_from_time_vector(time_s),
    )
    return float(time_s[int(np.argmax(velocity_rad_s[:, shaped_axis]))])


def _residual_shaping_start_time(
    desired_time_s: np.ndarray,
    desired_positions_rad: np.ndarray,
    residual_offline_time_s: np.ndarray,
    residual_offline_positions_rad: np.ndarray,
    shaped_axis: int,
    fallback_time_s: float,
) -> float:
    """Find when residual-tail shaping first changes the desired command.

    Args:
        desired_time_s: Desired command sample times `[N]` [s].
        desired_positions_rad: Desired joint positions `[N, num_joints]` [rad].
        residual_offline_time_s: Residual-tail offline Shaper sample times `[K]` [s].
        residual_offline_positions_rad: Residual-tail offline-shaped positions
            `[K, num_joints]` [rad].
        shaped_axis: Joint axis used for comparing raw and shaped positions.
        fallback_time_s: Returned time [s] if no residual-tail change is found.
    Returns:
        `float`: First sample time where residual shaping changes the command [s],
        or `fallback_time_s`.
    Raises:
        ValueError: If array dimensions are invalid.
    """

    raw_position_at_residual_time = np.interp(
        residual_offline_time_s,
        desired_time_s,
        desired_positions_rad[:, shaped_axis],
    )
    residual_difference_mask = (
        np.abs(
            residual_offline_positions_rad[:, shaped_axis]
            - raw_position_at_residual_time
        )
        > 1.0e-9
    )
    if not np.any(residual_difference_mask):
        return fallback_time_s
    return float(residual_offline_time_s[int(np.argmax(residual_difference_mask))])


def plot_command_profiles(
    desired_time_s: np.ndarray,
    desired_positions_rad: np.ndarray,
    always_on_time_s: np.ndarray,
    always_on_positions_rad: np.ndarray,
    residual_offline_time_s: np.ndarray,
    residual_offline_positions_rad: np.ndarray,
    streamed_time_s: np.ndarray,
    streamed_positions_rad: np.ndarray,
    windowed_time_s: np.ndarray,
    windowed_positions_rad: np.ndarray,
    shaped_axis: int,
    residual_shaping_start_time_s: float,
    output_path: Path | None = None,
) -> None:
    """Plot command position, velocity, and acceleration profiles.

    Args:
        desired_time_s: Desired command sample times `[N]` [s].
        desired_positions_rad: Desired joint positions `[N, num_joints]` [rad].
        always_on_time_s: Always-on Shaper sample times `[M]` [s].
        always_on_positions_rad: Always-on shaped positions `[M, num_joints]` [rad].
        residual_offline_time_s: Residual-tail offline Shaper sample times `[K]` [s].
        residual_offline_positions_rad: Residual-tail offline-shaped positions
            `[K, num_joints]` [rad].
        streamed_time_s: Sample-by-sample Shaper stream times `[L]` [s].
        streamed_positions_rad: Sample-by-sample shaped positions `[L, num_joints]` [rad].
        windowed_time_s: Fixed-window Shaper sample times `[W]` [s].
        windowed_positions_rad: Fixed-window shaped positions `[W, num_joints]` [rad].
        shaped_axis: Joint axis used for the single-axis profile plot.
        residual_shaping_start_time_s: Time when residual-tail shaping first changes
            the command [s].
        output_path: Optional file path for saving the plot.
    Returns:
        `None`.
    Raises:
        ValueError: If profile arrays have invalid shapes.
        OSError: If `output_path` cannot be written.
    """

    desired_profiles = _axis_position_velocity_acceleration(
        time_s=desired_time_s,
        positions_rad=desired_positions_rad,
        axis_index=shaped_axis,
    )
    always_on_profiles = _axis_position_velocity_acceleration(
        time_s=always_on_time_s,
        positions_rad=always_on_positions_rad,
        axis_index=shaped_axis,
    )
    residual_offline_profiles = _axis_position_velocity_acceleration(
        time_s=residual_offline_time_s,
        positions_rad=residual_offline_positions_rad,
        axis_index=shaped_axis,
    )
    streamed_profiles = _axis_position_velocity_acceleration(
        time_s=streamed_time_s,
        positions_rad=streamed_positions_rad,
        axis_index=shaped_axis,
    )
    windowed_profiles = _axis_position_velocity_acceleration(
        time_s=windowed_time_s,
        positions_rad=windowed_positions_rad,
        axis_index=shaped_axis,
    )

    profile_series = (
        (
            "Desired command",
            desired_time_s,
            desired_profiles,
            "black",
            "-",
            1.8,
            1.0,
            4,
        ),
        (
            EXAMPLE_1_LABEL,
            always_on_time_s,
            always_on_profiles,
            "tab:blue",
            "-",
            1.8,
            1.0,
            3,
        ),
        (
            EXAMPLE_2_LABEL,
            residual_offline_time_s,
            residual_offline_profiles,
            "tab:orange",
            "-.",
            2.0,
            1.0,
            5,
        ),
        (
            EXAMPLE_3_LABEL,
            windowed_time_s,
            windowed_profiles,
            "tab:cyan",
            ":",
            2.3,
            1.0,
            7,
        ),
        (
            EXAMPLE_4_LABEL,
            streamed_time_s,
            streamed_profiles,
            "tab:purple",
            "--",
            2.2,
            1.0,
            6,
        ),
    )
    y_labels = ("Position [rad]", "Velocity [rad/s]", "Acceleration [rad/s^2]")
    figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10.0, 8.0))

    for profile_index, y_label in enumerate(y_labels):
        axis = axes[profile_index]
        for (
            label,
            time_s,
            profiles,
            color,
            linestyle,
            linewidth,
            alpha,
            zorder,
        ) in profile_series:
            axis.plot(
                time_s,
                profiles[profile_index],
                label=label,
                color=color,
                linestyle=linestyle,
                linewidth=linewidth,
                alpha=alpha,
                zorder=zorder,
            )
        axis.axvline(
            residual_shaping_start_time_s,
            color="tab:purple",
            linestyle=":",
            label="Residual Shaper starts" if profile_index == 0 else None,
        )
        axis.set_ylabel(y_label)
        axis.grid(True, alpha=0.3)

    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time [s]")
    figure.tight_layout()

    if output_path is not None:
        figure.savefig(output_path, dpi=160)
    plt.show()


def plot_shaper_example_results(
    shaper: Any,
    desired_time_s: np.ndarray,
    desired_trajectory_rad: np.ndarray,
    always_on_time_s: np.ndarray,
    always_on_positions_rad: np.ndarray,
    residual_offline_time_s: np.ndarray,
    residual_offline_positions_rad: np.ndarray,
    streamed_times_s: Sequence[float],
    streamed_positions_rad: Sequence[np.ndarray],
    windowed_time_s: np.ndarray,
    windowed_positions_rad: np.ndarray,
    goal_position_rad: np.ndarray,
    shaped_axis: int,
    move_duration_s: float,
    dwell_duration_s: float,
    residual_window_duration_s: float,
) -> None:
    """Plot command profiles and simulated responses for the Shaper example.

    Args:
        shaper: Initialized `ShaperInterface` used to infer the simulation mode.
        desired_time_s: Desired command sample times `[N]` [s].
        desired_trajectory_rad: Desired joint positions `[N, num_joints]` [rad].
        always_on_time_s: Always-on Shaper sample times `[M]` [s].
        always_on_positions_rad: Always-on shaped positions `[M, num_joints]` [rad].
        residual_offline_time_s: Residual-tail offline Shaper sample times `[K]` [s].
        residual_offline_positions_rad: Residual-tail offline-shaped positions
            `[K, num_joints]` [rad].
        streamed_times_s: Sample-by-sample Shaper stream times [s].
        streamed_positions_rad: Sample-by-sample shaped positions [rad].
        windowed_time_s: Fixed-window Shaper sample times `[W]` [s].
        windowed_positions_rad: Fixed-window shaped positions `[W, num_joints]` [rad].
        goal_position_rad: Final target joint position `[num_joints]` [rad].
        shaped_axis: Joint axis used for single-axis plots.
        move_duration_s: Point-to-point move duration [s].
        dwell_duration_s: Time spent holding the final target after the move [s].
        residual_window_duration_s: Duration subtracted from final dwell to choose
            the residual-vibration measurement start [s].
    Returns:
        `None`.
    Raises:
        ValueError: If plotted arrays have invalid shapes.
        OSError: If plot output cannot be written.
    """

    streamed_time_array_s, streamed_position_array_rad = _streamed_samples_to_arrays(
        time_s=streamed_times_s,
        positions_rad=streamed_positions_rad,
    )
    deceleration_start_time_s = _final_deceleration_start_time(
        time_s=desired_time_s,
        positions_rad=desired_trajectory_rad,
        shaped_axis=shaped_axis,
    )
    residual_shaping_start_time_s = _residual_shaping_start_time(
        desired_time_s=desired_time_s,
        desired_positions_rad=desired_trajectory_rad,
        residual_offline_time_s=residual_offline_time_s,
        residual_offline_positions_rad=residual_offline_positions_rad,
        shaped_axis=shaped_axis,
        fallback_time_s=deceleration_start_time_s,
    )

    plot_command_profiles(
        desired_time_s=desired_time_s,
        desired_positions_rad=desired_trajectory_rad,
        always_on_time_s=always_on_time_s,
        always_on_positions_rad=always_on_positions_rad,
        residual_offline_time_s=residual_offline_time_s,
        residual_offline_positions_rad=residual_offline_positions_rad,
        streamed_time_s=streamed_time_array_s,
        streamed_positions_rad=streamed_position_array_rad,
        windowed_time_s=windowed_time_s,
        windowed_positions_rad=windowed_positions_rad,
        shaped_axis=shaped_axis,
        residual_shaping_start_time_s=residual_shaping_start_time_s,
    )
    simulate_and_plot_shaper_example(
        shaper=shaper,
        desired_time_s=desired_time_s,
        desired_trajectory_rad=desired_trajectory_rad,
        always_on_time_s=always_on_time_s,
        always_on_positions_rad=always_on_positions_rad,
        residual_offline_time_s=residual_offline_time_s,
        residual_offline_positions_rad=residual_offline_positions_rad,
        streamed_time_s=streamed_time_array_s,
        streamed_positions_rad=streamed_position_array_rad,
        windowed_time_s=windowed_time_s,
        windowed_positions_rad=windowed_positions_rad,
        goal_position_rad=goal_position_rad,
        shaped_axis=shaped_axis,
        move_duration_s=move_duration_s,
        deceleration_start_time_s=deceleration_start_time_s,
        residual_shaping_start_time_s=residual_shaping_start_time_s,
        dwell_duration_s=dwell_duration_s,
        residual_window_duration_s=residual_window_duration_s,
    )


def simulate_and_plot_shaper_example(
    shaper: Any,
    desired_time_s: np.ndarray,
    desired_trajectory_rad: np.ndarray,
    always_on_time_s: np.ndarray,
    always_on_positions_rad: np.ndarray,
    residual_offline_time_s: np.ndarray,
    residual_offline_positions_rad: np.ndarray,
    streamed_time_s: np.ndarray,
    streamed_positions_rad: np.ndarray,
    windowed_time_s: np.ndarray,
    windowed_positions_rad: np.ndarray,
    goal_position_rad: np.ndarray,
    shaped_axis: int,
    move_duration_s: float,
    deceleration_start_time_s: float,
    residual_shaping_start_time_s: float,
    dwell_duration_s: float,
    residual_window_duration_s: float,
) -> None:
    """Simulate and plot the Shaper example response.

    Args:
        shaper: Initialized `ShaperInterface` used to infer the simulation mode.
        desired_time_s: Desired command sample times `[N]` [s].
        desired_trajectory_rad: Unshaped joint positions `[N, num_joints]` [rad].
        always_on_time_s: Always-on Shaper sample times `[M]` [s].
        always_on_positions_rad: Always-on shaped positions `[M, num_joints]` [rad].
        residual_offline_time_s: Residual-tail offline Shaper sample times `[K]` [s].
        residual_offline_positions_rad: Residual-tail offline-shaped positions
            `[K, num_joints]` [rad].
        streamed_time_s: Sample-by-sample Shaper stream times `[L]` [s].
        streamed_positions_rad: Sample-by-sample shaped positions `[L, num_joints]` [rad].
        windowed_time_s: Fixed-window Shaper sample times `[W]` [s].
        windowed_positions_rad: Fixed-window shaped positions `[W, num_joints]` [rad].
        goal_position_rad: Final target joint position `[num_joints]` [rad].
        shaped_axis: Joint axis used for the single-axis plot.
        move_duration_s: Point-to-point move duration [s].
        deceleration_start_time_s: Time when the unshaped command starts decelerating
            from peak velocity [s].
        residual_shaping_start_time_s: Time when the residual-tail Shaper first
            deviates from the raw command [s].
        dwell_duration_s: Time spent holding the final target after the move [s].
        residual_window_duration_s: Duration subtracted from the final dwell to choose
            the residual-vibration measurement start [s].
    Returns:
        `None`.
    Raises:
        ValueError: If arrays passed to simulation have inconsistent shapes.
        OSError: If plot output cannot be written.
    """

    plant = infer_dominant_modal_plant(
        shaper=shaper,
        representative_command_rad=desired_trajectory_rad[0],
        axis_index=shaped_axis,
    )
    sample_time_s = float(desired_time_s[1] - desired_time_s[0])
    slower_time_s, slower_trajectory_rad = generate_point_to_point_trajectory(
        start_position_rad=desired_trajectory_rad[0],
        goal_position_rad=goal_position_rad,
        sample_time_s=sample_time_s,
        move_duration_s=SLOWER_REFERENCE_MOVE_DURATION_S,
        dwell_duration_s=dwell_duration_s,
    )
    desired_response = simulate_modal_position_response(
        time_s=desired_time_s,
        command_rad=desired_trajectory_rad[:, shaped_axis],
        plant=plant,
    )
    slower_response = simulate_modal_position_response(
        time_s=slower_time_s,
        command_rad=slower_trajectory_rad[:, shaped_axis],
        plant=plant,
    )
    always_on_response = simulate_modal_position_response(
        time_s=always_on_time_s,
        command_rad=always_on_positions_rad[:, shaped_axis],
        plant=plant,
    )
    residual_offline_response = simulate_modal_position_response(
        time_s=residual_offline_time_s,
        command_rad=residual_offline_positions_rad[:, shaped_axis],
        plant=plant,
    )
    streamed_response = simulate_modal_position_response(
        time_s=streamed_time_s,
        command_rad=streamed_positions_rad[:, shaped_axis],
        plant=plant,
    )
    windowed_response = simulate_modal_position_response(
        time_s=windowed_time_s,
        command_rad=windowed_positions_rad[:, shaped_axis],
        plant=plant,
    )

    final_position_rad = float(goal_position_rad[shaped_axis])
    residual_start_time_s = (
        move_duration_s + dwell_duration_s - residual_window_duration_s
    )
    unshaped_residual_rad = residual_vibration_rad(
        desired_response,
        final_value_rad=final_position_rad,
        start_time_s=residual_start_time_s,
    )
    slower_residual_start_time_s = (
        SLOWER_REFERENCE_MOVE_DURATION_S + dwell_duration_s - residual_window_duration_s
    )
    slower_residual_rad = residual_vibration_rad(
        slower_response,
        final_value_rad=final_position_rad,
        start_time_s=slower_residual_start_time_s,
    )
    always_on_residual_rad = residual_vibration_rad(
        always_on_response,
        final_value_rad=final_position_rad,
        start_time_s=residual_start_time_s,
    )
    residual_offline_residual_rad = residual_vibration_rad(
        residual_offline_response,
        final_value_rad=final_position_rad,
        start_time_s=residual_start_time_s,
    )
    streamed_residual_rad = residual_vibration_rad(
        streamed_response,
        final_value_rad=final_position_rad,
        start_time_s=residual_start_time_s,
    )
    windowed_residual_rad = residual_vibration_rad(
        windowed_response,
        final_value_rad=final_position_rad,
        start_time_s=residual_start_time_s,
    )

    print("Covalent Shaper example complete.")
    print(
        "Dominant simulation mode: "
        f"wn={plant.natural_frequency_rad_s:.3f} rad/s, "
        f"zeta={plant.damping_ratio:.4f}"
    )
    print(f"Residual metric starts at t={residual_start_time_s:.3f} s")
    print(f"Unshaped residual vibration: {unshaped_residual_rad:.6f} rad")
    print(
        f"Slower unshaped residual vibration: {slower_residual_rad:.6f} rad "
        f"after a {SLOWER_REFERENCE_MOVE_DURATION_S:.1f} s move"
    )
    print(f"Command deceleration starts at t={deceleration_start_time_s:.3f} s")
    print(
        "Residual Shaper starts changing the command at "
        f"t={residual_shaping_start_time_s:.3f} s"
    )
    print(f"Always-on Shaper residual vibration: {always_on_residual_rad:.6f} rad")
    print(
        "Residual-tail offline Shaper residual vibration: "
        f"{residual_offline_residual_rad:.6f} rad"
    )
    print(
        "Sample-by-sample stream Shaper residual vibration: "
        f"{streamed_residual_rad:.6f} rad"
    )
    print(
        "Fixed-window buffer Shaper residual vibration: "
        f"{windowed_residual_rad:.6f} rad"
    )

    plot_responses(
        desired_response=desired_response,
        slower_response=slower_response,
        always_on_response=always_on_response,
        residual_offline_response=residual_offline_response,
        streamed_response=streamed_response,
        windowed_response=windowed_response,
        move_duration_s=move_duration_s,
        residual_shaping_start_time_s=residual_shaping_start_time_s,
        slower_move_duration_s=SLOWER_REFERENCE_MOVE_DURATION_S,
    )


def plot_responses(
    desired_response: SimulatedResponse,
    slower_response: SimulatedResponse,
    always_on_response: SimulatedResponse,
    residual_offline_response: SimulatedResponse,
    streamed_response: SimulatedResponse,
    windowed_response: SimulatedResponse,
    move_duration_s: float,
    residual_shaping_start_time_s: float,
    slower_move_duration_s: float,
    output_path: Path | None = None,
) -> None:
    """Plot desired command and simulated plant responses.

    Args:
        desired_response: Plant response to the unshaped desired command.
        slower_response: Plant response to a slower unshaped desired command.
        always_on_response: Plant response when Shaper is enabled from the start.
        residual_offline_response: Plant response to the offline residual-tail Shaper command.
        streamed_response: Plant response to the sample-by-sample Shaper stream.
        windowed_response: Plant response to the fixed-window Shaper buffer.
        move_duration_s: End time of the point-to-point move [s].
        residual_shaping_start_time_s: Time when the residual-tail Shaper first
            deviates from the raw command [s].
        slower_move_duration_s: End time of the slower point-to-point move [s].
        output_path: Optional file path for saving the plot.
    Returns:
        `None`.
    Raises:
        OSError: If `output_path` cannot be written.
    """

    figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10.0, 7.0))
    axes[0].plot(
        desired_response.time_s,
        desired_response.command_rad,
        label=f"Baseline command: {move_duration_s:.1f} s move",
        color="black",
    )
    axes[0].plot(
        slower_response.time_s,
        slower_response.command_rad,
        label=f"Comparison command: {slower_move_duration_s:.1f} s move",
        color="tab:green",
        linestyle="-.",
    )
    axes[0].set_ylabel("Command [rad]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(
        desired_response.time_s,
        desired_response.response_rad,
        label=f"Baseline response: unshaped {move_duration_s:.1f} s move",
        color="tab:gray",
    )
    axes[1].plot(
        slower_response.time_s,
        slower_response.response_rad,
        label=f"Comparison response: unshaped {slower_move_duration_s:.1f} s move",
        color="tab:green",
        linestyle="-.",
    )
    axes[1].plot(
        always_on_response.time_s,
        always_on_response.response_rad,
        label=EXAMPLE_1_LABEL,
        color="tab:blue",
        linestyle="-",
        linewidth=1.8,
        zorder=3,
    )
    axes[1].plot(
        residual_offline_response.time_s,
        residual_offline_response.response_rad,
        label=EXAMPLE_2_LABEL,
        color="tab:orange",
    )
    axes[1].plot(
        windowed_response.time_s,
        windowed_response.response_rad,
        label=EXAMPLE_3_LABEL,
        color="tab:cyan",
        linestyle=":",
        linewidth=2.3,
        zorder=7,
    )
    axes[1].plot(
        streamed_response.time_s,
        streamed_response.response_rad,
        label=EXAMPLE_4_LABEL,
        color="tab:purple",
        linestyle="--",
        linewidth=2.2,
        zorder=6,
    )
    axes[1].axvline(
        residual_shaping_start_time_s,
        color="tab:purple",
        linestyle=":",
        label="Residual Shaper starts",
    )
    axes[1].axvline(
        move_duration_s,
        color="tab:red",
        linestyle=":",
        label="End of point-to-point move",
    )
    axes[1].axvline(
        slower_move_duration_s,
        color="tab:green",
        linestyle=":",
        label="End of slower point-to-point move",
    )
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("Response [rad]")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    figure.tight_layout()

    if output_path is not None:
        figure.savefig(output_path, dpi=160)
    plt.show()
