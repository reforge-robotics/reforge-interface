"""Utility functions for the Covalent Joint Tracker example."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import json
from pathlib import Path
from typing import Any, Literal, Sequence

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

EXAMPLE_MODEL_DIRECTORY = Path(__file__).resolve().parent / "example_models"
JOINT_CONTROLLER_ARTIFACT_FILENAME = "joint_controller_models.json"
EXAMPLE_AXIS_COUNT = 6
QUINTIC_MAX_VELOCITY_FACTOR = 1.875
QUINTIC_MAX_ACCELERATION_FACTOR = 10.0 * np.sqrt(3.0) / 3.0
MIN_TBI_SEGMENT_DURATION_S = 0.55
REAL_TIME_RATIO_LIMIT = 1.0


@dataclass(frozen=True)
class SimulatedTrajectorySet:
    """Store command and plant-response arrays for one trajectory set.

    Args:
        time_s: Sample times [s].
        positions_rad: Commanded joint positions [rad], shape `(N, num_joints)`.
        response_rad: Simulated joint response [rad], shape `(N, num_joints)`.
    Raises:
        None.
    """

    time_s: np.ndarray
    positions_rad: np.ndarray
    response_rad: np.ndarray


@dataclass(frozen=True)
class WindowStreamRecord:
    """Store metadata for one emitted Joint Tracker Controller window.

    Args:
        start_index: First global sample emitted by the window [count].
        stop_index: One-past-last global sample emitted by the window [count].
        sample_count: Number of emitted samples [count].
        compute_time_s: Controller compute time reported by the payload [s].
        pop_wait_time_s: Wall time spent waiting for this payload [s].
        n_apply: Controller apply stride for the producer session [samples].
        solve_indices: Per-solve monotonically increasing IDs reported by the
            persistent controller session [count].
        solve_start_indices: Per-solve global start sample indices [samples].
        solve_stop_indices: Per-solve global stop sample indices [samples].
        terminal_padding_samples: Number of terminal lookahead samples padded by
            the producer for this emitted window [samples].
        hold_samples_added: Number of held-reference samples committed because
            the planner underrun fallback was active [samples].
    Raises:
        None.
    """

    start_index: int
    stop_index: int
    sample_count: int
    compute_time_s: float
    pop_wait_time_s: float
    n_apply: int
    solve_indices: tuple[int, ...]
    solve_start_indices: tuple[int, ...]
    solve_stop_indices: tuple[int, ...]
    terminal_padding_samples: int
    hold_samples_added: int


@dataclass(frozen=True)
class WindowContinuitySummary:
    """Store worst boundary jumps across emitted command windows.

    Args:
        boundary_count: Number of adjacent window boundaries evaluated [count].
        max_position_jump_rad: Largest position step at a boundary [rad].
        max_velocity_jump_rad_s: Largest slope jump at a boundary [rad/s].
        max_boundary_acceleration_rad_s2: Largest single-sample acceleration
            implied by a boundary slope jump [rad/s^2].
    Raises:
        None.
    """

    boundary_count: int
    max_position_jump_rad: float
    max_velocity_jump_rad_s: float
    max_boundary_acceleration_rad_s2: float


@dataclass(frozen=True)
class WindowTimingSummary:
    """Store timing measurements for one windowed Joint Tracker Controller run.

    Args:
        window_count: Number of emitted windows [count].
        prefill_time_s: Wall time spent in initial synchronous prefill [s].
        total_execution_time_s: Total emitted command duration [s].
        total_pop_wait_time_s: Total wall time spent waiting for payloads [s].
        total_compute_time_s: Sum of controller compute time reported by payloads [s].
        max_window_compute_time_s: Largest reported payload compute time [s].
        max_window_execution_time_s: Largest emitted window duration [s].
        total_compute_ratio: Ratio of total compute time to emitted command duration.
        max_window_compute_ratio: Largest window compute/execution ratio.
        solve_index_count: Number of solve indices reported by the stream [count].
        solve_indices_are_continuous: Whether solve indices advance by one without
            repeats or gaps.
        cold_start_count: Number of solve-index discontinuities after stream start [count].
    Raises:
        None.
    """

    window_count: int
    prefill_time_s: float
    total_execution_time_s: float
    total_pop_wait_time_s: float
    total_compute_time_s: float
    max_window_compute_time_s: float
    max_window_execution_time_s: float
    total_compute_ratio: float
    max_window_compute_ratio: float
    solve_index_count: int
    solve_indices_are_continuous: bool
    cold_start_count: int


@dataclass(frozen=True)
class AppendablePlannerSimulationResult:
    """Store outputs from the appendable real-time planner simulation.

    Args:
        desired_time_s: Committed desired-reference times, including hold samples [s].
        desired_positions_rad: Committed desired-reference positions [rad].
        optimized_time_s: Emitted optimized command times [s].
        optimized_positions_rad: Emitted optimized command positions [rad].
        window_records: Metadata for emitted optimized windows.
        appended_source_samples: Number of original planner samples appended [samples].
        terminal_padding_samples: Total terminal lookahead padding used [samples].
        hold_samples_added: Total held-reference samples committed because of
            planner underruns [samples].
    Raises:
        None.
    """

    desired_time_s: np.ndarray
    desired_positions_rad: np.ndarray
    optimized_time_s: np.ndarray
    optimized_positions_rad: np.ndarray
    window_records: tuple[WindowStreamRecord, ...]
    appended_source_samples: int
    terminal_padding_samples: int
    hold_samples_added: int


@dataclass
class AppendablePlannerSimulationState:
    """Store mutable state for a simulated online planner.

    Args:
        source_positions_rad: Full reference used by the planner simulator [rad].
        append_stop_index: One-past-last source sample already appended [samples].
        append_chunk_samples: Samples appended per planner update [samples].
        low_watermark_samples: Minimum lookahead remaining before another append [samples].
        delay_thresholds: Source-sample thresholds where simulated planner delays start [samples].
        delay_windows: Output-window counts to wait at each delay threshold [windows].
        next_delay_index: Next delay threshold to evaluate [count].
        remaining_delay_windows: Remaining output windows to consume before appending resumes [windows].
        has_finished_appending: Whether `finish()` should already have been called.
    Returns:
        `None`.
    Raises:
        None.
    """

    source_positions_rad: np.ndarray
    append_stop_index: int
    append_chunk_samples: int
    low_watermark_samples: int
    delay_thresholds: tuple[int, ...]
    delay_windows: tuple[int, ...]
    next_delay_index: int = 0
    remaining_delay_windows: int = 0
    has_finished_appending: bool = False


@dataclass(frozen=True)
class AppendablePlannerDecision:
    """Describe the next planner action after one optimized window is consumed.

    Args:
        command_to_append: Next desired samples [rad], or `None` if no append is due.
        should_finish: Whether the planner has appended all source samples.
    Returns:
        `None`.
    Raises:
        None.
    """

    command_to_append: np.ndarray | None
    should_finish: bool


@lru_cache(maxsize=1)
def load_example_dynamics() -> tuple[tuple[list[float], list[float]], ...]:
    """Load example joint dynamics from the persisted controller model bundle.

    Args:
        None.
    Returns:
        `tuple[tuple[list[float], list[float]], ...]`: Numerator and denominator
        coefficients ordered by axis index.
    Raises:
        FileNotFoundError: If the example model artifact is missing.
        KeyError: If the artifact is missing required model fields.
        ValueError: If no axes are present in the model artifact.
        json.JSONDecodeError: If the artifact is not valid JSON.
    """

    artifact_path = EXAMPLE_MODEL_DIRECTORY / JOINT_CONTROLLER_ARTIFACT_FILENAME
    with artifact_path.open("r", encoding="utf-8") as artifact_file:
        artifact = json.load(artifact_file)

    axes = sorted(artifact["axes"], key=lambda axis_payload: int(axis_payload["axis"]))
    if not axes:
        raise ValueError("Example joint controller artifact must contain axes.")

    return tuple(
        (
            [float(value) for value in axis_payload["numerator"]],
            [float(value) for value in axis_payload["denominator"]],
        )
        for axis_payload in axes
    )


def generate_tbi_trajectory(
    sample_time_s: float,
    num_joints: int,
    joint_position_limit_rad: float,
    max_joint_speed_rad_s: float,
    max_joint_acceleration_rad_s2: float,
    dwell_duration_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Generate a feasible multi-segment joint trajectory for tracking.

    Args:
        sample_time_s: Controller sample period [s].
        num_joints: Number of robot joints in the returned trajectory.
        joint_position_limit_rad: Symmetric joint position limit [rad].
        max_joint_speed_rad_s: Per-joint speed limit [rad/s].
        max_joint_acceleration_rad_s2: Per-joint acceleration limit [rad/s^2].
        dwell_duration_s: Final hold duration after the last motion segment [s].
    Returns:
        `tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]`: Time [s],
        joint positions [rad], velocities [rad/s], accelerations [rad/s^2],
        and the end time of the commanded motion before the final dwell [s].
    Raises:
        ValueError: If timing, limit, or joint-count inputs are invalid.
    """

    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive.")
    if num_joints < EXAMPLE_AXIS_COUNT:
        raise ValueError("num_joints must cover all modeled axes.")
    if joint_position_limit_rad <= 0.0:
        raise ValueError("joint_position_limit_rad must be positive.")
    if max_joint_speed_rad_s <= 0.0:
        raise ValueError("max_joint_speed_rad_s must be positive.")
    if max_joint_acceleration_rad_s2 <= 0.0:
        raise ValueError("max_joint_acceleration_rad_s2 must be positive.")
    if dwell_duration_s < 0.0:
        raise ValueError("dwell_duration_s must be nonnegative.")

    modeled_waypoints_rad = np.array(
        [
            [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
            [0.45, -0.25, 0.15, -0.18, 0.12, -0.10],
            [-0.20, 0.32, -0.12, 0.20, -0.16, 0.14],
            [0.55, 0.10, 0.18, -0.25, 0.20, -0.18],
            [0.10, -0.35, 0.00, 0.12, -0.10, 0.08],
        ],
        dtype=float,
    )
    if np.max(np.abs(modeled_waypoints_rad)) > joint_position_limit_rad:
        raise ValueError("TBI waypoints exceed the configured joint position limit.")

    waypoint_count = modeled_waypoints_rad.shape[0]
    waypoints_rad = np.zeros((waypoint_count, num_joints), dtype=float)
    waypoints_rad[:, : modeled_waypoints_rad.shape[1]] = modeled_waypoints_rad

    time_chunks_s: list[np.ndarray] = []
    position_chunks_rad: list[np.ndarray] = []
    velocity_chunks_rad_s: list[np.ndarray] = []
    acceleration_chunks_rad_s2: list[np.ndarray] = []
    elapsed_time_s = 0.0

    # Each segment uses a quintic blend so position, velocity, and acceleration
    # start and end smoothly at every waypoint.
    for segment_index in range(waypoints_rad.shape[0] - 1):
        start_position_rad = waypoints_rad[segment_index]
        end_position_rad = waypoints_rad[segment_index + 1]
        delta_rad = end_position_rad - start_position_rad
        max_abs_delta_rad = float(np.max(np.abs(delta_rad)))
        if max_abs_delta_rad == 0.0:
            continue

        speed_limited_duration_s = (
            QUINTIC_MAX_VELOCITY_FACTOR * max_abs_delta_rad / max_joint_speed_rad_s
        )
        acceleration_limited_duration_s = np.sqrt(
            QUINTIC_MAX_ACCELERATION_FACTOR
            * max_abs_delta_rad
            / max_joint_acceleration_rad_s2
        )
        required_duration_s = max(
            MIN_TBI_SEGMENT_DURATION_S,
            speed_limited_duration_s,
            float(acceleration_limited_duration_s),
        )
        segment_sample_count = int(np.ceil(required_duration_s / sample_time_s))
        segment_duration_s = segment_sample_count * sample_time_s
        local_time_s = np.arange(segment_sample_count + 1, dtype=float) * sample_time_s
        phase = local_time_s / segment_duration_s

        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        blend_dot = 30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
        blend_ddot = 60.0 * phase - 180.0 * phase**2 + 120.0 * phase**3

        positions_rad = start_position_rad + blend[:, None] * delta_rad
        velocities_rad_s = (blend_dot[:, None] / segment_duration_s) * delta_rad
        accelerations_rad_s2 = (blend_ddot[:, None] / segment_duration_s**2) * delta_rad
        segment_time_s = elapsed_time_s + local_time_s

        if time_chunks_s:
            segment_time_s = segment_time_s[1:]
            positions_rad = positions_rad[1:]
            velocities_rad_s = velocities_rad_s[1:]
            accelerations_rad_s2 = accelerations_rad_s2[1:]

        time_chunks_s.append(segment_time_s)
        position_chunks_rad.append(positions_rad)
        velocity_chunks_rad_s.append(velocities_rad_s)
        acceleration_chunks_rad_s2.append(accelerations_rad_s2)
        elapsed_time_s += segment_duration_s

    if not time_chunks_s:
        raise ValueError("TBI trajectory must contain at least one moving segment.")

    motion_end_time_s = elapsed_time_s
    dwell_sample_count = int(round(dwell_duration_s / sample_time_s))
    if dwell_sample_count > 0:
        dwell_time_s = (
            motion_end_time_s
            + np.arange(1, dwell_sample_count + 1, dtype=float) * sample_time_s
        )
        final_position_rad = position_chunks_rad[-1][-1]
        time_chunks_s.append(dwell_time_s)
        position_chunks_rad.append(
            np.repeat(final_position_rad[None, :], dwell_sample_count, axis=0)
        )
        velocity_chunks_rad_s.append(np.zeros((dwell_sample_count, num_joints)))
        acceleration_chunks_rad_s2.append(np.zeros((dwell_sample_count, num_joints)))

    return (
        np.concatenate(time_chunks_s),
        np.vstack(position_chunks_rad),
        np.vstack(velocity_chunks_rad_s),
        np.vstack(acceleration_chunks_rad_s2),
        motion_end_time_s,
    )


def extract_trajectory_window(
    time_s: np.ndarray,
    positions_rad: np.ndarray,
    velocities_rad_s: np.ndarray,
    accelerations_rad_s2: np.ndarray,
    start_index: int,
    horizon_sample_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Extract a finite future trajectory window for online controller usage.

    Args:
        time_s: Full trajectory sample times [s], shape `(N,)`.
        positions_rad: Full joint positions [rad], shape `(N, num_joints)`.
        velocities_rad_s: Full joint velocities [rad/s], shape `(N, num_joints)`.
        accelerations_rad_s2: Full joint accelerations [rad/s^2], shape
            `(N, num_joints)`.
        start_index: First sample index in the returned horizon [count].
        horizon_sample_count: Number of samples returned in the horizon [count].
    Returns:
        `tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]`: Horizon-local
        time [s], positions [rad], velocities [rad/s], and accelerations
        [rad/s^2].
    Raises:
        ValueError: If arrays are inconsistent or the requested horizon is invalid.
    """

    time_s = np.asarray(time_s, dtype=float)
    positions_rad = np.asarray(positions_rad, dtype=float)
    velocities_rad_s = np.asarray(velocities_rad_s, dtype=float)
    accelerations_rad_s2 = np.asarray(accelerations_rad_s2, dtype=float)

    if time_s.ndim != 1 or time_s.shape[0] < 2:
        raise ValueError("time_s must be one-dimensional with at least two samples.")
    if positions_rad.ndim != 2:
        raise ValueError("positions_rad must be two-dimensional.")
    if (
        velocities_rad_s.shape != positions_rad.shape
        or accelerations_rad_s2.shape != positions_rad.shape
    ):
        raise ValueError(
            "Trajectory position, velocity, and acceleration shapes must match."
        )
    if positions_rad.shape[0] != time_s.shape[0]:
        raise ValueError("Trajectory arrays must have the same sample count.")
    if start_index < 0 or start_index >= time_s.shape[0]:
        raise ValueError("start_index must refer to an existing trajectory sample.")
    if horizon_sample_count < 2:
        raise ValueError("horizon_sample_count must be at least two.")

    sample_time_s = float(time_s[1] - time_s[0])
    if sample_time_s <= 0.0:
        raise ValueError("time_s must be strictly increasing.")

    stop_index = start_index + horizon_sample_count
    clipped_stop_index = min(stop_index, time_s.shape[0])
    window_positions_rad = positions_rad[start_index:clipped_stop_index].copy()
    window_velocities_rad_s = velocities_rad_s[start_index:clipped_stop_index].copy()
    window_accelerations_rad_s2 = accelerations_rad_s2[
        start_index:clipped_stop_index
    ].copy()

    pad_count = horizon_sample_count - window_positions_rad.shape[0]
    if pad_count > 0:
        final_position_rad = positions_rad[-1]
        window_positions_rad = np.vstack(
            [
                window_positions_rad,
                np.repeat(final_position_rad[None, :], pad_count, axis=0),
            ]
        )
        window_velocities_rad_s = np.vstack(
            [window_velocities_rad_s, np.zeros((pad_count, positions_rad.shape[1]))]
        )
        window_accelerations_rad_s2 = np.vstack(
            [
                window_accelerations_rad_s2,
                np.zeros((pad_count, positions_rad.shape[1])),
            ]
        )

    local_time_s = np.arange(horizon_sample_count, dtype=float) * sample_time_s
    return (
        local_time_s,
        window_positions_rad,
        window_velocities_rad_s,
        window_accelerations_rad_s2,
    )


def estimate_derivatives(
    trajectory_rad: np.ndarray,
    sample_time_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate joint velocity and acceleration from position samples.

    Args:
        trajectory_rad: Joint positions [rad], shape `(N, num_joints)`.
        sample_time_s: Sample period [s].
    Returns:
        `tuple[np.ndarray, np.ndarray]`: Velocity [rad/s] and acceleration
        [rad/s^2], each shaped `(N, num_joints)`.
    Raises:
        ValueError: If the trajectory is not two-dimensional or sample time is
            not positive.
    """

    trajectory_rad = np.asarray(trajectory_rad, dtype=float)
    if trajectory_rad.ndim != 2:
        raise ValueError("trajectory_rad must be two-dimensional.")
    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive.")
    if trajectory_rad.shape[0] <= 1:
        return np.zeros_like(trajectory_rad), np.zeros_like(trajectory_rad)

    edge_order: Literal[1, 2] = 2 if trajectory_rad.shape[0] > 2 else 1
    velocity_rad_s = np.gradient(
        trajectory_rad,
        sample_time_s,
        axis=0,
        edge_order=edge_order,
    )
    acceleration_rad_s2 = np.gradient(
        velocity_rad_s,
        sample_time_s,
        axis=0,
        edge_order=edge_order,
    )
    return velocity_rad_s, acceleration_rad_s2


def append_window_payload_from_payload_time(
    window_payload: Any,
    streamed_times_s: list[float],
    streamed_positions_rad: list[np.ndarray],
    window_records: list[WindowStreamRecord],
    pop_wait_time_s: float,
) -> None:
    """Append one emitted payload using the payload's own time vector.

    Args:
        window_payload: Payload returned by a Joint Tracker stream.
        streamed_times_s: Mutable list receiving emitted sample times [s].
        streamed_positions_rad: Mutable list receiving emitted joint positions [rad].
        window_records: Mutable list receiving emitted-window metadata.
        pop_wait_time_s: Wall time spent waiting for this payload [s].
    Returns:
        `None`.
    Raises:
        ValueError: If the payload has invalid positions or time data.
        AttributeError: If required payload fields are missing.
    """
    window_positions_rad = np.atleast_2d(
        np.asarray(window_payload.positions, dtype=float)
    )
    if window_positions_rad.shape[0] == 0:
        raise ValueError("Window payload must contain at least one sample.")
    if window_payload.time is None:
        raise ValueError("Appendable payloads must include a time vector.")
    window_time_s = np.asarray(window_payload.time, dtype=float).reshape(-1)
    if window_time_s.shape[0] != window_positions_rad.shape[0]:
        raise ValueError("Payload time and position sample counts must match.")

    for sample_time_s, sample_position_rad in zip(
        window_time_s,
        window_positions_rad,
        strict=True,
    ):
        streamed_times_s.append(float(sample_time_s))
        streamed_positions_rad.append(sample_position_rad.copy())

    metadata = getattr(window_payload, "metadata", {})
    n_apply_metadata = metadata.get("n_apply")
    n_apply = (
        int(n_apply_metadata)
        if n_apply_metadata is not None
        else int(window_positions_rad.shape[0])
    )
    window_records.append(
        WindowStreamRecord(
            start_index=int(window_payload.start_index),
            stop_index=int(window_payload.stop_index),
            sample_count=int(window_positions_rad.shape[0]),
            compute_time_s=float(window_payload.compute_time_s),
            pop_wait_time_s=float(pop_wait_time_s),
            n_apply=n_apply,
            solve_indices=tuple(
                int(index) for index in metadata.get("solve_indices", ())
            ),
            solve_start_indices=tuple(
                int(index) for index in metadata.get("solve_start_indices", ())
            ),
            solve_stop_indices=tuple(
                int(index) for index in metadata.get("solve_stop_indices", ())
            ),
            terminal_padding_samples=int(metadata.get("terminal_padding_samples", 0)),
            hold_samples_added=int(metadata.get("hold_samples_added", 0)),
        )
    )


def create_appendable_planner_simulation_state(
    source_positions_rad: np.ndarray,
    initial_chunk_samples: int,
    append_chunk_samples: int,
    low_watermark_samples: int,
    delay_after_source_samples: Sequence[int],
    delay_windows: Sequence[int],
) -> tuple[AppendablePlannerSimulationState, np.ndarray]:
    """Create simulated-planner state and return the first reference chunk.

    Args:
        source_positions_rad: Planner's original desired trajectory [rad], shape `(N, J)`.
        initial_chunk_samples: Samples appended before the stream starts [samples].
        append_chunk_samples: Samples appended per planner update [samples].
        low_watermark_samples: Remaining committed samples that trigger another append [samples].
        delay_after_source_samples: Source-sample thresholds where planner delays are inserted [samples].
        delay_windows: Number of output windows to consume before appending resumes at each delay.
    Returns:
        `tuple[AppendablePlannerSimulationState, np.ndarray]`: Mutable planner
        state and initial desired samples [rad].
    Raises:
        ValueError: If trajectory or delay settings are invalid.
    """

    source_positions_rad = np.asarray(source_positions_rad, dtype=float)
    if source_positions_rad.ndim != 2 or source_positions_rad.shape[0] == 0:
        raise ValueError("source_positions_rad must have shape `(N, J)`.")
    if initial_chunk_samples <= 0 or append_chunk_samples <= 0:
        raise ValueError("chunk sample counts must be positive.")
    if low_watermark_samples < 0:
        raise ValueError("low_watermark_samples must be non-negative.")
    if len(delay_after_source_samples) != len(delay_windows):
        raise ValueError("delay thresholds and delay window counts must match.")

    source_sample_count = int(source_positions_rad.shape[0])
    append_stop_index = min(int(initial_chunk_samples), source_sample_count)
    state = AppendablePlannerSimulationState(
        source_positions_rad=source_positions_rad.copy(),
        append_stop_index=append_stop_index,
        append_chunk_samples=int(append_chunk_samples),
        low_watermark_samples=int(low_watermark_samples),
        delay_thresholds=tuple(int(value) for value in delay_after_source_samples),
        delay_windows=tuple(int(value) for value in delay_windows),
    )
    return state, source_positions_rad[:append_stop_index].copy()


def next_appendable_planner_decision(
    planner_state: AppendablePlannerSimulationState,
    available_reference_samples: int,
    emitted_stop_index: int,
) -> AppendablePlannerDecision:
    """Return the planner append or finish action after one output window.

    Args:
        planner_state: Mutable simulated-planner state.
        available_reference_samples: Samples currently committed to the input stream [samples].
        emitted_stop_index: One-past-last optimized sample consumed by the robot [samples].
    Returns:
        `AppendablePlannerDecision`: Desired samples to append and finish signal.
    Raises:
        ValueError: If sample counts are negative.
    """

    if available_reference_samples < 0 or emitted_stop_index < 0:
        raise ValueError("sample counts must be non-negative.")
    if planner_state.has_finished_appending:
        return AppendablePlannerDecision(command_to_append=None, should_finish=False)

    command_to_append = None
    if planner_state.remaining_delay_windows > 0:
        planner_state.remaining_delay_windows -= 1
    elif (
        planner_state.next_delay_index < len(planner_state.delay_thresholds)
        and planner_state.append_stop_index
        >= planner_state.delay_thresholds[planner_state.next_delay_index]
    ):
        planner_state.remaining_delay_windows = planner_state.delay_windows[
            planner_state.next_delay_index
        ]
        planner_state.next_delay_index += 1
    elif (
        available_reference_samples - emitted_stop_index
        <= planner_state.low_watermark_samples
    ):
        next_chunk_stop_index = (
            planner_state.append_stop_index + planner_state.append_chunk_samples
        )
        if (
            planner_state.next_delay_index < len(planner_state.delay_thresholds)
            and planner_state.append_stop_index
            < planner_state.delay_thresholds[planner_state.next_delay_index]
        ):
            next_chunk_stop_index = min(
                next_chunk_stop_index,
                planner_state.delay_thresholds[planner_state.next_delay_index],
            )

        source_sample_count = int(planner_state.source_positions_rad.shape[0])
        next_stop_index = min(next_chunk_stop_index, source_sample_count)
        if next_stop_index > planner_state.append_stop_index:
            command_to_append = planner_state.source_positions_rad[
                planner_state.append_stop_index : next_stop_index
            ].copy()
            planner_state.append_stop_index = next_stop_index
        else:
            command_to_append = None
    else:
        command_to_append = None

    source_sample_count = int(planner_state.source_positions_rad.shape[0])
    should_finish = (
        planner_state.append_stop_index >= source_sample_count
        and planner_state.remaining_delay_windows == 0
    )
    if should_finish:
        planner_state.has_finished_appending = True

    return AppendablePlannerDecision(
        command_to_append=command_to_append,
        should_finish=should_finish,
    )


def build_appendable_planner_result(
    appendable_stream: Any,
    streamed_times_s: Sequence[float],
    streamed_positions_rad: Sequence[np.ndarray],
    window_records: Sequence[WindowStreamRecord],
    sample_time_s: float,
    appended_source_samples: int,
) -> AppendablePlannerSimulationResult:
    """Build a plotting and diagnostics result from a visible controller loop.

    Args:
        appendable_stream: Stream returned by `JointTrackerInterface.process_trajectory_stream_mode`.
        streamed_times_s: Optimized output sample times emitted by the loop [s].
        streamed_positions_rad: Optimized output joint positions emitted by the loop [rad].
        window_records: Metadata for emitted optimized windows.
        sample_time_s: Controller sample period [s].
        appended_source_samples: Number of original planner samples appended [samples].
    Returns:
        `AppendablePlannerSimulationResult`: Committed desired trajectory and optimized output.
    Raises:
        ValueError: If stream arrays are empty or sample time is invalid.
        AttributeError: If the appendable stream does not expose reference positions.
    """

    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive.")
    desired_positions_rad = appendable_stream.reference_positions()
    desired_time_s = np.arange(desired_positions_rad.shape[0], dtype=float) * float(
        sample_time_s
    )
    optimized_time_s, optimized_positions_rad = _streamed_samples_to_arrays(
        time_s=streamed_times_s,
        positions_rad=streamed_positions_rad,
    )
    return AppendablePlannerSimulationResult(
        desired_time_s=desired_time_s,
        desired_positions_rad=desired_positions_rad,
        optimized_time_s=optimized_time_s,
        optimized_positions_rad=optimized_positions_rad,
        window_records=tuple(window_records),
        appended_source_samples=int(appended_source_samples),
        terminal_padding_samples=sum(
            window_record.terminal_padding_samples for window_record in window_records
        ),
        hold_samples_added=sum(
            window_record.hold_samples_added for window_record in window_records
        ),
    )


def summarize_window_stream(
    streamed_positions_rad: Sequence[np.ndarray],
    window_records: Sequence[WindowStreamRecord],
    sample_time_s: float,
    modeled_axes: Sequence[int],
    prefill_time_s: float,
) -> tuple[WindowContinuitySummary, WindowTimingSummary]:
    """Summarize continuity and timing for emitted JTC windows.

    Args:
        streamed_positions_rad: Emitted window samples [rad].
        window_records: Metadata for emitted JTC windows.
        sample_time_s: Controller sample period [s].
        modeled_axes: Joint axes included in the continuity metrics.
        prefill_time_s: Wall time spent in initial buffer prefill [s].
    Returns:
        `tuple[WindowContinuitySummary, WindowTimingSummary]`: Boundary and
        timing diagnostics for the stream.
    Raises:
        ValueError: If inputs are empty or sample time is invalid.
    """

    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive.")
    if not window_records:
        raise ValueError("window_records must contain at least one record.")
    if len(streamed_positions_rad) == 0:
        raise ValueError("streamed_positions_rad must contain at least one sample.")

    position_array_rad = np.vstack(streamed_positions_rad)
    modeled_axis_tuple = tuple(int(axis_index) for axis_index in modeled_axes)
    if not modeled_axis_tuple:
        raise ValueError("modeled_axes must contain at least one axis.")

    max_position_jump_rad = 0.0
    max_velocity_jump_rad_s = 0.0
    max_boundary_acceleration_rad_s2 = 0.0
    cumulative_index = 0

    for previous_record, current_record in zip(
        window_records[:-1],
        window_records[1:],
        strict=False,
    ):
        previous_start = cumulative_index
        previous_stop = previous_start + previous_record.sample_count
        current_start = previous_stop
        cumulative_index = previous_stop

        if previous_record.stop_index != current_record.start_index:
            position_jump_rad = np.inf
            velocity_jump_rad_s = np.inf
        else:
            previous_last_rad = position_array_rad[previous_stop - 1]
            current_first_rad = position_array_rad[current_start]
            position_jump_by_axis_rad = np.abs(
                current_first_rad[list(modeled_axis_tuple)]
                - previous_last_rad[list(modeled_axis_tuple)]
            )
            position_jump_rad = float(np.max(position_jump_by_axis_rad))

            if previous_record.sample_count >= 2 and current_record.sample_count >= 2:
                previous_velocity_rad_s = (
                    position_array_rad[previous_stop - 1]
                    - position_array_rad[previous_stop - 2]
                ) / sample_time_s
                current_velocity_rad_s = (
                    position_array_rad[current_start + 1]
                    - position_array_rad[current_start]
                ) / sample_time_s
                velocity_jump_by_axis_rad_s = np.abs(
                    current_velocity_rad_s[list(modeled_axis_tuple)]
                    - previous_velocity_rad_s[list(modeled_axis_tuple)]
                )
                velocity_jump_rad_s = float(np.max(velocity_jump_by_axis_rad_s))
            else:
                velocity_jump_rad_s = 0.0

        max_position_jump_rad = max(max_position_jump_rad, position_jump_rad)
        max_velocity_jump_rad_s = max(max_velocity_jump_rad_s, velocity_jump_rad_s)
        max_boundary_acceleration_rad_s2 = max(
            max_boundary_acceleration_rad_s2,
            velocity_jump_rad_s / sample_time_s,
        )

    total_compute_time_s = float(
        sum(window_record.compute_time_s for window_record in window_records)
    )
    total_execution_time_s = float(
        sum(window_record.sample_count for window_record in window_records)
        * sample_time_s
    )
    max_window_compute_time_s = float(
        max(window_record.compute_time_s for window_record in window_records)
    )
    max_window_execution_time_s = float(
        max(window_record.sample_count for window_record in window_records)
        * sample_time_s
    )
    total_pop_wait_time_s = float(
        sum(window_record.pop_wait_time_s for window_record in window_records)
    )
    solve_indices = tuple(
        solve_index
        for window_record in window_records
        for solve_index in window_record.solve_indices
    )
    solve_index_jumps = tuple(
        next_index - previous_index
        for previous_index, next_index in zip(
            solve_indices[:-1],
            solve_indices[1:],
            strict=False,
        )
    )
    cold_start_count = sum(1 for index_jump in solve_index_jumps if index_jump != 1)
    solve_indices_are_continuous = bool(
        not solve_indices or all(index_jump == 1 for index_jump in solve_index_jumps)
    )
    total_compute_ratio = (
        total_compute_time_s / total_execution_time_s
        if total_execution_time_s > 0.0
        else np.inf
    )
    max_window_compute_ratio = (
        max_window_compute_time_s / max_window_execution_time_s
        if max_window_execution_time_s > 0.0
        else np.inf
    )

    return (
        WindowContinuitySummary(
            boundary_count=max(0, len(window_records) - 1),
            max_position_jump_rad=float(max_position_jump_rad),
            max_velocity_jump_rad_s=float(max_velocity_jump_rad_s),
            max_boundary_acceleration_rad_s2=float(max_boundary_acceleration_rad_s2),
        ),
        WindowTimingSummary(
            window_count=len(window_records),
            prefill_time_s=float(prefill_time_s),
            total_execution_time_s=total_execution_time_s,
            total_pop_wait_time_s=total_pop_wait_time_s,
            total_compute_time_s=total_compute_time_s,
            max_window_compute_time_s=max_window_compute_time_s,
            max_window_execution_time_s=max_window_execution_time_s,
            total_compute_ratio=float(total_compute_ratio),
            max_window_compute_ratio=float(max_window_compute_ratio),
            solve_index_count=len(solve_indices),
            solve_indices_are_continuous=solve_indices_are_continuous,
            cold_start_count=int(cold_start_count),
        ),
    )


def print_window_stream_diagnostics(
    continuity_summary: WindowContinuitySummary,
    timing_summary: WindowTimingSummary,
) -> None:
    """Print a terminal table with emitted JTC window-stream diagnostics.

    Args:
        continuity_summary: Boundary continuity metrics for emitted windows.
        timing_summary: Compute and wait-time metrics for emitted windows.
    Returns:
        `None`.
    Raises:
        None.
    """

    print("\nAppendable-stream JTC stream diagnostics")
    print(
        _format_terminal_table(
            headers=("Metric", "Value", "What it means"),
            rows=(
                (
                    "Windows emitted",
                    str(timing_summary.window_count),
                    "Optimized windows consumed by the demo robot loop.",
                ),
                (
                    "Trajectory duration",
                    f"{timing_summary.total_execution_time_s:.3f} s",
                    "Execution time represented by the emitted command.",
                ),
                (
                    "Total compute time",
                    f"{timing_summary.total_compute_time_s * 1.0e3:.2f} ms",
                    "Total JTC optimization time across emitted windows.",
                ),
                (
                    "Max window compute time",
                    f"{timing_summary.max_window_compute_time_s * 1.0e3:.2f} ms",
                    "Slowest single optimized window.",
                ),
                (
                    "Boundary count",
                    str(continuity_summary.boundary_count),
                    "Adjacent window boundaries checked for continuity.",
                ),
                (
                    "Max position jump",
                    f"{continuity_summary.max_position_jump_rad:.6f} rad",
                    "Largest position discontinuity between windows.",
                ),
                (
                    "Max velocity jump",
                    f"{continuity_summary.max_velocity_jump_rad_s:.6f} rad/s",
                    "Largest slope change between adjacent windows.",
                ),
                (
                    "Max boundary acceleration",
                    f"{continuity_summary.max_boundary_acceleration_rad_s2:.6f} rad/s^2",
                    "Single-sample acceleration implied by boundary slope change.",
                ),
            ),
        )
    )


def _format_terminal_table(
    headers: Sequence[str], rows: Sequence[Sequence[str]]
) -> str:
    """Build a fixed-width ASCII table for terminal output.

    Args:
        headers: Column names to print in the first row.
        rows: Table rows with one string value per column.
    Returns:
        `str`: Terminal table using ASCII borders.
    Raises:
        ValueError: If a row has a different column count from `headers`.
    """

    if not headers:
        raise ValueError("headers must contain at least one column.")
    column_count = len(headers)
    normalized_rows = []
    for row in rows:
        if len(row) != column_count:
            raise ValueError("all rows must have the same column count as headers.")
        normalized_rows.append(tuple(str(value) for value in row))

    string_headers = tuple(str(header) for header in headers)
    column_widths = [
        max(
            len(string_headers[column_index]),
            *(len(row[column_index]) for row in normalized_rows),
        )
        for column_index in range(column_count)
    ]
    separator = "+".join("-" * (width + 2) for width in column_widths)
    separator = f"+{separator}+"
    header_line = "|".join(
        f" {header:<{column_widths[column_index]}} "
        for column_index, header in enumerate(string_headers)
    )
    table_lines = [separator, f"|{header_line}|", separator]
    for row in normalized_rows:
        row_line = "|".join(
            f" {value:<{column_widths[column_index]}} "
            for column_index, value in enumerate(row)
        )
        table_lines.append(f"|{row_line}|")
    table_lines.append(separator)
    return "\n".join(table_lines)


def print_real_time_timing_analysis(
    label: str,
    appendable_result: AppendablePlannerSimulationResult,
    modeled_axes: Sequence[int],
    sample_time_s: float,
) -> WindowTimingSummary:
    """Print real-time compute and cold-start diagnostics for one JTC stream.

    Args:
        label: Human-readable stream label, such as `Example 2A`.
        appendable_result: Emitted stream result to analyze.
        modeled_axes: Joint axes included in continuity checks.
        sample_time_s: Controller sample period [s].
    Returns:
        `WindowTimingSummary`: Timing summary used for the printed verdict.
    Raises:
        ValueError: If the stream result is empty or the timing inputs are invalid.
    """

    _, timing_summary = summarize_window_stream(
        streamed_positions_rad=list(appendable_result.optimized_positions_rad),
        window_records=appendable_result.window_records,
        sample_time_s=sample_time_s,
        modeled_axes=modeled_axes,
        prefill_time_s=0.0,
    )
    total_status = (
        "PASS" if timing_summary.total_compute_ratio < REAL_TIME_RATIO_LIMIT else "FAIL"
    )
    window_status = (
        "PASS"
        if timing_summary.max_window_compute_ratio < REAL_TIME_RATIO_LIMIT
        else "FAIL"
    )
    cold_start_status = (
        "PASS" if timing_summary.solve_indices_are_continuous else "FAIL"
    )
    print(f"\n{label} timing analysis")
    print(
        _format_terminal_table(
            headers=(
                "Check",
                "Measured",
                "Requirement",
                "Result",
                "What it means",
            ),
            rows=(
                (
                    "Total compute vs trajectory",
                    (
                        f"{timing_summary.total_compute_time_s:.3f} s / "
                        f"{timing_summary.total_execution_time_s:.3f} s "
                        f"({timing_summary.total_compute_ratio:.4f})"
                    ),
                    "ratio < 1.0",
                    total_status,
                    "JTC computes faster than robot execution time.",
                ),
                (
                    "Worst window compute",
                    (
                        f"{timing_summary.max_window_compute_time_s:.3f} s / "
                        f"{timing_summary.max_window_execution_time_s:.3f} s "
                        f"({timing_summary.max_window_compute_ratio:.4f})"
                    ),
                    "ratio < 1.0",
                    window_status,
                    "Each emitted window is ready before it must execute.",
                ),
                (
                    "Consumer wait time",
                    f"{timing_summary.total_pop_wait_time_s:.3f} s",
                    "informational",
                    "INFO",
                    "Time the demo loop spent waiting for optimized windows.",
                ),
                (
                    "Cold starts after stream start",
                    (
                        f"{timing_summary.cold_start_count} discontinuities "
                        f"across {timing_summary.solve_index_count} solves"
                    ),
                    "0 discontinuities",
                    cold_start_status,
                    "Solve indices stayed continuous; the optimizer did not restart.",
                ),
            ),
        )
    )
    return timing_summary


def max_command_difference_rad(
    first_positions_rad: np.ndarray,
    second_positions_rad: np.ndarray,
) -> float:
    """Compute the largest absolute position difference between two commands.

    Args:
        first_positions_rad: First joint-position command [rad], shape `(N, J)`.
        second_positions_rad: Second joint-position command [rad], shape `(M, J)`.
    Returns:
        `float`: Maximum absolute difference over the common sample range [rad].
    Raises:
        ValueError: If arrays are not two-dimensional or have incompatible joints.
    """

    first_positions_rad = np.asarray(first_positions_rad, dtype=float)
    second_positions_rad = np.asarray(second_positions_rad, dtype=float)
    if first_positions_rad.ndim != 2 or second_positions_rad.ndim != 2:
        raise ValueError("position arrays must be two-dimensional.")
    if first_positions_rad.shape[1] != second_positions_rad.shape[1]:
        raise ValueError("position arrays must have the same joint count.")

    common_samples = min(first_positions_rad.shape[0], second_positions_rad.shape[0])
    if common_samples == 0:
        raise ValueError("position arrays must share at least one sample.")
    return float(
        np.max(
            np.abs(
                first_positions_rad[:common_samples]
                - second_positions_rad[:common_samples]
            )
        )
    )


def print_appendable_stream_comparison(
    label_prefix: str,
    reference_label: str,
    max_difference_rad: float,
    appendable_result: AppendablePlannerSimulationResult,
) -> None:
    """Print appendable-stream comparison and fallback diagnostics.

    Args:
        label_prefix: Output label before the comparison description.
        reference_label: Human-readable benchmark label.
        max_difference_rad: Maximum command difference against the benchmark [rad].
        appendable_result: Appendable stream result containing fallback counters.
    Returns:
        `None`.
    Raises:
        None.
    """

    print(f"\n{label_prefix} comparison")
    print(
        _format_terminal_table(
            headers=("Metric", "Value", "What it means"),
            rows=(
                (
                    f"Max command difference vs {reference_label}",
                    f"{max_difference_rad:.6f} rad",
                    "Lower means the streamed result matches the benchmark.",
                ),
                (
                    "Held-reference samples",
                    str(appendable_result.hold_samples_added),
                    "Samples inserted when the simulated planner paused.",
                ),
                (
                    "Terminal padding samples",
                    str(appendable_result.terminal_padding_samples),
                    "Lookahead padding used to drain the final windows.",
                ),
            ),
        )
    )


def report_and_plot_joint_tracker_stream(
    desired_time_s: np.ndarray,
    desired_positions_rad: np.ndarray,
    offline_time_s: np.ndarray,
    offline_positions_rad: np.ndarray,
    streamed_times_s: Sequence[float],
    streamed_positions_rad: Sequence[np.ndarray],
    window_records: Sequence[WindowStreamRecord],
    modeled_axes: Sequence[int],
    sample_time_s: float,
    motion_end_time_s: float,
    prefill_time_s: float,
    show_stream_diagnostics: bool = False,
) -> None:
    """Print stream diagnostics and render the Joint Tracker example plots.

    Args:
        desired_time_s: Desired command sample times [s].
        desired_positions_rad: Desired joint positions [rad].
        offline_time_s: Offline optimized command sample times [s].
        offline_positions_rad: Offline optimized joint positions [rad].
        streamed_times_s: Windowed or appendable optimized sample times [s].
        streamed_positions_rad: Windowed or appendable optimized joint positions [rad].
        window_records: Metadata for emitted optimized windows.
        modeled_axes: Joint axes included in simulation and diagnostics.
        sample_time_s: Controller sample period [s].
        motion_end_time_s: End time of the commanded motion before final dwell [s].
        prefill_time_s: Wall time spent in initial synchronous prefill [s].
        show_stream_diagnostics: Whether to print detailed stream timing and
            continuity diagnostics.
    Returns:
        `None`.
    Side Effects:
        Prints diagnostics and renders Matplotlib figures.
    Raises:
        ValueError: If diagnostics or plot inputs are invalid.
        OSError: If Matplotlib cannot render the figures.
    """

    continuity_summary, timing_summary = summarize_window_stream(
        streamed_positions_rad=streamed_positions_rad,
        window_records=window_records,
        sample_time_s=sample_time_s,
        modeled_axes=modeled_axes,
        prefill_time_s=prefill_time_s,
    )
    if show_stream_diagnostics:
        print_window_stream_diagnostics(
            continuity_summary=continuity_summary,
            timing_summary=timing_summary,
        )
    plot_joint_tracker_example_results(
        desired_time_s=desired_time_s,
        desired_positions_rad=desired_positions_rad,
        offline_time_s=offline_time_s,
        offline_positions_rad=offline_positions_rad,
        streamed_times_s=streamed_times_s,
        streamed_positions_rad=streamed_positions_rad,
        modeled_axes=modeled_axes,
        sample_time_s=sample_time_s,
        motion_end_time_s=motion_end_time_s,
    )


def plot_joint_tracker_example_results(
    desired_time_s: np.ndarray,
    desired_positions_rad: np.ndarray,
    offline_time_s: np.ndarray,
    offline_positions_rad: np.ndarray,
    streamed_times_s: Sequence[float],
    streamed_positions_rad: Sequence[np.ndarray],
    modeled_axes: Sequence[int],
    sample_time_s: float,
    motion_end_time_s: float,
) -> None:
    """Plot command profiles and simulated plant responses for the example.

    Args:
        desired_time_s: Desired command sample times [s], shape `(N,)`.
        desired_positions_rad: Desired joint command [rad], shape `(N, num_joints)`.
        offline_time_s: Offline Joint Tracker command times [s], shape `(M,)`.
        offline_positions_rad: Offline optimized command [rad], shape
            `(M, num_joints)`.
        streamed_times_s: Appendable-stream command times [s].
        streamed_positions_rad: Appendable-stream optimized positions [rad].
        modeled_axes: Joint axes backed by the example transfer functions.
        sample_time_s: Controller sample period [s].
        motion_end_time_s: End time of the commanded motion before final dwell [s].
    Returns:
        `None`.
    Side Effects:
        Prints RMS tracking-error summaries and renders Matplotlib figures.
    Raises:
        ValueError: If plotted arrays have invalid dimensions.
        OSError: If Matplotlib cannot render the figures.
    """

    streamed_time_array_s, streamed_position_array_rad = _streamed_samples_to_arrays(
        time_s=streamed_times_s,
        positions_rad=streamed_positions_rad,
    )
    desired_response = simulate_joint_response(
        time_s=desired_time_s,
        command_positions_rad=desired_positions_rad,
        modeled_axes=modeled_axes,
    )
    offline_response = simulate_joint_response(
        time_s=offline_time_s,
        command_positions_rad=offline_positions_rad,
        modeled_axes=modeled_axes,
    )
    streamed_response = simulate_joint_response(
        time_s=streamed_time_array_s,
        command_positions_rad=streamed_position_array_rad,
        modeled_axes=modeled_axes,
    )

    print("Covalent Joint Tracker example complete.")
    _print_tracking_summary(
        label="Unoptimized desired command",
        desired_positions_rad=desired_positions_rad,
        response_rad=desired_response.response_rad,
        modeled_axes=modeled_axes,
    )
    _print_tracking_summary(
        label="Offline Joint Tracker command",
        desired_positions_rad=_interp_positions(
            source_time_s=desired_time_s,
            source_positions_rad=desired_positions_rad,
            target_time_s=offline_time_s,
        ),
        response_rad=offline_response.response_rad,
        modeled_axes=modeled_axes,
    )
    _print_tracking_summary(
        label="Appendable-stream Joint Tracker command",
        desired_positions_rad=_interp_positions(
            source_time_s=desired_time_s,
            source_positions_rad=desired_positions_rad,
            target_time_s=streamed_time_array_s,
        ),
        response_rad=streamed_response.response_rad,
        modeled_axes=modeled_axes,
    )

    _plot_command_profiles(
        desired_time_s=desired_time_s,
        desired_positions_rad=desired_positions_rad,
        offline_time_s=offline_time_s,
        offline_positions_rad=offline_positions_rad,
        streamed_time_s=streamed_time_array_s,
        streamed_positions_rad=streamed_position_array_rad,
        modeled_axes=modeled_axes,
        sample_time_s=sample_time_s,
    )
    _plot_response_comparison(
        desired_response=desired_response,
        offline_response=offline_response,
        streamed_response=streamed_response,
        modeled_axes=modeled_axes,
        motion_end_time_s=motion_end_time_s,
    )


def simulate_joint_response(
    time_s: np.ndarray,
    command_positions_rad: np.ndarray,
    modeled_axes: Sequence[int],
) -> SimulatedTrajectorySet:
    """Simulate the plant response for the command using the example dynamics.

    Args:
        time_s: Command sample times [s], shape `(N,)`.
        command_positions_rad: Commanded joint positions [rad], shape
            `(N, num_joints)`.
        modeled_axes: Joint axes simulated with transfer functions.
    Returns:
        `SimulatedTrajectorySet`: Original command and simulated response [rad].
    Raises:
        ValueError: If arrays have invalid shapes or modeled axes are invalid.
    """

    time_s = np.asarray(time_s, dtype=float)
    command_positions_rad = np.asarray(command_positions_rad, dtype=float)
    if time_s.ndim != 1:
        raise ValueError("time_s must be one-dimensional.")
    if command_positions_rad.ndim != 2:
        raise ValueError("command_positions_rad must be two-dimensional.")
    if time_s.shape[0] != command_positions_rad.shape[0]:
        raise ValueError("time_s and command_positions_rad must have matching rows.")

    example_dynamics = load_example_dynamics()
    response_rad = command_positions_rad.copy()
    for model_index, axis_index in enumerate(modeled_axes):
        if axis_index < 0 or axis_index >= command_positions_rad.shape[1]:
            raise ValueError("modeled_axes must refer to available command columns.")
        if model_index >= len(example_dynamics):
            raise ValueError("modeled_axes exceeds the example model axis count.")
        numerator, denominator = example_dynamics[model_index]
        axis_system = signal.TransferFunction(numerator, denominator)
        initial_command_rad = float(command_positions_rad[0, axis_index])
        input_zeroed_rad = command_positions_rad[:, axis_index] - initial_command_rad
        _, axis_response_zeroed_rad, _ = signal.lsim(
            axis_system,
            U=input_zeroed_rad,
            T=time_s,
        )
        response_rad[:, axis_index] = (
            np.asarray(axis_response_zeroed_rad, dtype=float).reshape(-1)
            + initial_command_rad
        )

    return SimulatedTrajectorySet(
        time_s=time_s,
        positions_rad=command_positions_rad,
        response_rad=response_rad,
    )


def _streamed_samples_to_arrays(
    time_s: Sequence[float],
    positions_rad: Sequence[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Convert streamed samples into aligned NumPy arrays.

    Args:
        time_s: Streamed sample times [s].
        positions_rad: Streamed joint positions [rad].
    Returns:
        `tuple[np.ndarray, np.ndarray]`: Time vector [s] and positions [rad].
    Raises:
        ValueError: If no samples are present or sample counts differ.
    """

    time_array_s = np.asarray(time_s, dtype=float)
    if len(positions_rad) == 0:
        raise ValueError("positions_rad must contain at least one sample.")
    position_array_rad = np.vstack(positions_rad)
    if time_array_s.shape[0] != position_array_rad.shape[0]:
        raise ValueError("time_s and positions_rad must have the same sample count.")
    return time_array_s, position_array_rad


def _interp_positions(
    source_time_s: np.ndarray,
    source_positions_rad: np.ndarray,
    target_time_s: np.ndarray,
) -> np.ndarray:
    """Interpolate joint positions onto a target time vector.

    Args:
        source_time_s: Source sample times [s], shape `(N,)`.
        source_positions_rad: Source positions [rad], shape `(N, num_joints)`.
        target_time_s: Target sample times [s], shape `(M,)`.
    Returns:
        `np.ndarray`: Interpolated positions [rad], shape `(M, num_joints)`.
    Raises:
        ValueError: If the source positions are not two-dimensional.
    """

    source_positions_rad = np.asarray(source_positions_rad, dtype=float)
    if source_positions_rad.ndim != 2:
        raise ValueError("source_positions_rad must be two-dimensional.")
    return np.column_stack(
        [
            np.interp(target_time_s, source_time_s, source_positions_rad[:, axis_index])
            for axis_index in range(source_positions_rad.shape[1])
        ]
    )


def _tracking_rms_error(
    desired_positions_rad: np.ndarray,
    response_rad: np.ndarray,
    axis_index: int,
) -> float:
    """Compute RMS tracking error over all available samples.

    Args:
        desired_positions_rad: Desired joint positions [rad].
        response_rad: Simulated joint response [rad].
        axis_index: Joint axis used for the metric [count].
    Returns:
        `float`: RMS tracking error [rad].
    Raises:
        None.
    """

    error_rad = response_rad[:, axis_index] - desired_positions_rad[:, axis_index]
    return float(np.sqrt(np.mean(error_rad**2)))


def _print_tracking_summary(
    label: str,
    desired_positions_rad: np.ndarray,
    response_rad: np.ndarray,
    modeled_axes: Sequence[int],
) -> None:
    """Print one RMS tracking summary line per modeled joint axis.

    Args:
        label: Human-readable command label.
        desired_positions_rad: Desired joint positions [rad].
        response_rad: Simulated joint response [rad].
        modeled_axes: Joint axes included in the printed summary.
    Returns:
        `None`.
    Raises:
        None.
    """

    axis_summaries = []
    for axis_index in modeled_axes:
        rms_rad = _tracking_rms_error(
            desired_positions_rad=desired_positions_rad,
            response_rad=response_rad,
            axis_index=axis_index,
        )
        axis_summaries.append(f"J{axis_index + 1}={rms_rad:.6f} rad")
    print(f"{label} full-trajectory RMS tracking error: " + ", ".join(axis_summaries))


def _plot_command_profiles(
    desired_time_s: np.ndarray,
    desired_positions_rad: np.ndarray,
    offline_time_s: np.ndarray,
    offline_positions_rad: np.ndarray,
    streamed_time_s: np.ndarray,
    streamed_positions_rad: np.ndarray,
    modeled_axes: Sequence[int],
    sample_time_s: float,
) -> None:
    """Plot command position, velocity, and acceleration for modeled axes.

    Args:
        desired_time_s: Desired command sample times [s].
        desired_positions_rad: Desired joint command [rad].
        offline_time_s: Offline optimized command times [s].
        offline_positions_rad: Offline optimized command [rad].
        streamed_time_s: Appendable-stream optimized command times [s].
        streamed_positions_rad: Appendable-stream optimized command [rad].
        modeled_axes: Modeled joint axes included in the plot.
        sample_time_s: Controller sample period [s].
    Returns:
        `None`.
    Raises:
        OSError: If Matplotlib cannot render the figure.
    """

    desired_velocity_rad_s, desired_acceleration_rad_s2 = estimate_derivatives(
        desired_positions_rad,
        sample_time_s,
    )
    offline_velocity_rad_s, offline_acceleration_rad_s2 = estimate_derivatives(
        offline_positions_rad,
        sample_time_s,
    )
    streamed_velocity_rad_s, streamed_acceleration_rad_s2 = estimate_derivatives(
        streamed_positions_rad,
        sample_time_s,
    )
    series = (
        (
            "Desired command",
            desired_time_s,
            desired_positions_rad,
            desired_velocity_rad_s,
            desired_acceleration_rad_s2,
            "black",
            "-",
        ),
        (
            "Example 1 command",
            offline_time_s,
            offline_positions_rad,
            offline_velocity_rad_s,
            offline_acceleration_rad_s2,
            "tab:blue",
            "-",
        ),
        (
            "Example 2 command",
            streamed_time_s,
            streamed_positions_rad,
            streamed_velocity_rad_s,
            streamed_acceleration_rad_s2,
            "tab:purple",
            "--",
        ),
    )
    profile_rows = (
        ("Position [rad]", 0),
        ("Velocity [rad/s]", 1),
        ("Acceleration [rad/s^2]", 2),
    )
    figure, axes = plt.subplots(
        len(profile_rows),
        len(modeled_axes),
        sharex=True,
        figsize=(20.0, 8.0),
    )

    for column_index, axis_index in enumerate(modeled_axes):
        for row_index, (row_label, profile_index) in enumerate(profile_rows):
            axis = axes[row_index, column_index]
            for (
                label,
                time_s,
                position_rad,
                velocity_rad_s,
                acceleration_rad_s2,
                color,
                linestyle,
            ) in series:
                profiles = (position_rad, velocity_rad_s, acceleration_rad_s2)
                axis.plot(
                    time_s,
                    profiles[profile_index][:, axis_index],
                    label=label,
                    color=color,
                    linestyle=linestyle,
                    linewidth=1.8,
                )
            axis.grid(True, alpha=0.3)
            if column_index == 0:
                axis.set_ylabel(row_label)
            if row_index == 0:
                axis.set_title(f"Joint {axis_index + 1}")
            if row_index == len(profile_rows) - 1:
                axis.set_xlabel("Time [s]")

    axes[0, -1].legend(loc="best")
    figure.tight_layout()
    plt.show()


def _plot_response_comparison(
    desired_response: SimulatedTrajectorySet,
    offline_response: SimulatedTrajectorySet,
    streamed_response: SimulatedTrajectorySet,
    modeled_axes: Sequence[int],
    motion_end_time_s: float,
) -> None:
    """Plot simulated plant response for desired and optimized commands.

    Args:
        desired_response: Plant response to the unoptimized desired command.
        offline_response: Plant response to the offline optimized command.
        streamed_response: Plant response to the appendable-stream optimized command.
        modeled_axes: Modeled joint axes included in the plot.
        motion_end_time_s: End time of the commanded motion before final dwell [s].
    Returns:
        `None`.
    Raises:
        OSError: If Matplotlib cannot render the figure.
    """

    figure, axes = plt.subplots(
        len(modeled_axes),
        1,
        sharex=True,
        figsize=(10.0, 12.0),
    )
    axes_array = np.atleast_1d(axes)

    for row_index, axis_index in enumerate(modeled_axes):
        axis = axes_array[row_index]
        axis.plot(
            desired_response.time_s,
            desired_response.positions_rad[:, axis_index],
            label="Desired trajectory",
            color="black",
            linestyle=":",
            linewidth=2.0,
        )
        axis.plot(
            desired_response.time_s,
            desired_response.response_rad[:, axis_index],
            label="Robot Joint response to desired command",
            color="tab:gray",
            linewidth=1.8,
        )
        axis.plot(
            offline_response.time_s,
            offline_response.response_rad[:, axis_index],
            label="Robot Joint response to JTC example 1 command",
            color="tab:blue",
            linewidth=1.8,
        )
        axis.plot(
            streamed_response.time_s,
            streamed_response.response_rad[:, axis_index],
            label="Robot Joint response to JTC example 2 command",
            color="tab:purple",
            linestyle="--",
            linewidth=1.8,
        )
        axis.axvline(
            motion_end_time_s,
            color="tab:red",
            linestyle=":",
            label="End of commanded motion" if row_index == 0 else None,
        )
        axis.set_ylabel(f"Joint {axis_index + 1} [rad]")
        axis.grid(True, alpha=0.3)

    axes_array[0].legend(loc="best")
    axes_array[-1].set_xlabel("Time [s]")
    figure.tight_layout()
    plt.show()
