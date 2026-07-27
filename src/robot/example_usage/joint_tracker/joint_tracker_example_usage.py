"""Demonstrate Covalent Joint Tracker in offline and streaming modes."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np

from reforge_core.control.joint_tracker import JointTrackerInterface
from robot.example_usage.joint_tracker.example_utility import (
    WindowStreamRecord,
    append_window_payload_from_payload_time,
    build_appendable_planner_result,
    create_appendable_planner_simulation_state,
    estimate_derivatives,
    generate_tbi_trajectory,
    max_command_difference_rad,
    next_appendable_planner_decision,
    print_appendable_stream_comparison,
    print_real_time_timing_analysis,
    report_and_plot_joint_tracker_stream,
)

THIS_DIR = Path(__file__).resolve().parent

MODEL_DIRECTORY = THIS_DIR / "example_models"

SAMPLE_TIME_S = 0.01
NUM_AXES = 6
NUM_JOINTS = 6
MODELED_AXES = tuple(range(NUM_AXES))
DWELL_DURATION_S = 0.75
COBOT_JOINT_POSITION_LIMIT_RAD = np.deg2rad(350.0)
COBOT_MAX_JOINT_SPEED_RAD_S = np.deg2rad(180.0)
COBOT_MAX_JOINT_ACCELERATION_RAD_S2 = np.deg2rad(720.0)


def parse_cli_args() -> argparse.Namespace:
    """Parse optional reporting flags for the example script.

    Args:
        None.
    Returns:
        `argparse.Namespace`: Parsed flags controlling optional terminal output.
    Raises:
        SystemExit: If CLI parsing fails.
    """

    parser = argparse.ArgumentParser(
        description="Run the Covalent Joint Tracker example usage script."
    )
    parser.add_argument(
        "--show-timing-analysis",
        action="store_true",
        help=(
            "Print real-time compute tables for the stream examples. "
            "Disabled by default to keep normal example output concise."
        ),
    )
    parser.add_argument(
        "--show-online-planner-analysis",
        action="store_true",
        help=(
            "Print the delayed online-planner benchmark comparison for Example 2B. "
            "Disabled by default because it is a diagnostic check, not required "
            "controller usage."
        ),
    )
    return parser.parse_args()


def main(
    *,
    show_timing_analysis: bool = False,
    show_online_planner_analysis: bool = False,
    show_plots: bool = True,
) -> None:
    """Run complete-trajectory and appendable JTC examples.

    Args:
        show_timing_analysis: Whether to print timing diagnostics.
        show_online_planner_analysis: Whether to print delayed-planner diagnostics.
        show_plots: Whether to render the example plots.
    Returns:
        `None`.
    Raises:
        OSError: If the example model bundle or plots cannot be created.
        ValueError: If trajectory or controller inputs are invalid.
        RuntimeError: If Joint Tracker initialization or trajectory optimization fails.
    """
    # SIMULATION ONLY: this helper creates a feasible desired trajectory so the
    # example can run without a robot SDK. In your application, replace this
    # block with the trajectory from your planner, teach pendant, or robot SDK.
    (
        time_s,
        desired_trajectory_rad,
        desired_velocity_rad_s,
        desired_acceleration_rad_s2,
        motion_end_time_s,
    ) = generate_tbi_trajectory(
        sample_time_s=SAMPLE_TIME_S,
        num_joints=NUM_JOINTS,
        joint_position_limit_rad=COBOT_JOINT_POSITION_LIMIT_RAD,
        max_joint_speed_rad_s=COBOT_MAX_JOINT_SPEED_RAD_S,
        max_joint_acceleration_rad_s2=COBOT_MAX_JOINT_ACCELERATION_RAD_S2,
        dwell_duration_s=DWELL_DURATION_S,
    )

    # =========================================================================
    # Start Example 1: Optimize a complete trajectory with Joint Tracker.
    #
    # CONTROLLER CODE TO COPY: use this pattern when the complete desired
    # trajectory is available before execution starts.
    # =========================================================================
    initializing_JTC = JointTrackerInterface(
        sample_time=SAMPLE_TIME_S,
        num_joints=NUM_JOINTS,
        model_directory=str(MODEL_DIRECTORY),
        num_axes=NUM_AXES,
        axis_indices=list(MODELED_AXES),
    )
    JTC_offline_mode = initializing_JTC.process_trajectory(
        command=desired_trajectory_rad,
        command_dot=desired_velocity_rad_s,
        command_ddot=desired_acceleration_rad_s2,
        time_vector=list(time_s),
    )
    # REPLACE WITH YOUR ROBOT SDK: send `JTC_offline_mode.positions`,
    # `JTC_offline_mode.velocities`, and `JTC_offline_mode.accelerations`
    # to the robot if your SDK accepts full trajectory uploads.
    # =========================================================================
    # End Example 1.
    # =========================================================================

    # =========================================================================
    # Start Example 2A: Optimize a known trajectory while the robot is moving.
    #
    # CONTROLLER CODE TO COPY: use this pattern when the complete desired
    # trajectory is known before execution, but you want JTC to optimize and emit
    # windows while your robot loop consumes them.
    # =========================================================================
    initializing_JTC = JointTrackerInterface(
        sample_time=SAMPLE_TIME_S,
        num_joints=NUM_JOINTS,
        model_directory=str(MODEL_DIRECTORY),
        num_axes=NUM_AXES,
        axis_indices=list(MODELED_AXES),
    )
    JTC_stream_mode = initializing_JTC.process_trajectory_stream_mode()
    # CONTROLLER CODE TO COPY: append the complete desired trajectory before
    # execution starts. JTC will optimize and emit windows while the robot loop
    # consumes them.
    JTC_stream_mode.append_reference(desired_trajectory_rad)
    # `finish()` tells JTC the planner has no more desired samples to append.
    JTC_stream_mode.finish()
    # `start()` tells JTC to begin processing the stream to generate optimized command.
    JTC_stream_mode.start()

    # SIMULATION/REPORTING ONLY: these lists store emitted windows for plots.
    lookahead_stream_times_s: list[float] = []
    lookahead_stream_positions_rad: list[np.ndarray] = []
    lookahead_stream_records: list[WindowStreamRecord] = []
    # CONTROLLER CODE TO COPY: each loop iteration consumes one optimized command
    # window from the appendable stream.
    while not JTC_stream_mode.is_complete():
        pop_start_wall_s = time.perf_counter()
        optimized_command_window = JTC_stream_mode.pop()
        pop_wait_time_s = time.perf_counter() - pop_start_wall_s
        if optimized_command_window is None:
            if JTC_stream_mode.is_complete():
                break
            raise RuntimeError(
                "Appendable Joint Tracker stream did not produce a window."
            )

        # REPLACE WITH YOUR ROBOT SDK: send `optimized_command_window.positions`
        # to the robot here. Keep the helper below only when running this demo.
        append_window_payload_from_payload_time(
            window_payload=optimized_command_window,
            streamed_times_s=lookahead_stream_times_s,
            streamed_positions_rad=lookahead_stream_positions_rad,
            window_records=lookahead_stream_records,
            pop_wait_time_s=pop_wait_time_s,
        )

    JTC_stream_mode.close()
    # SIMULATION/REPORTING ONLY: package emitted samples for plots and metrics.
    lookahead_stream_result = build_appendable_planner_result(
        appendable_stream=JTC_stream_mode,
        streamed_times_s=lookahead_stream_times_s,
        streamed_positions_rad=lookahead_stream_positions_rad,
        window_records=lookahead_stream_records,
        sample_time_s=SAMPLE_TIME_S,
        appended_source_samples=desired_trajectory_rad.shape[0],
    )
    # =========================================================================
    # End Example 2A.
    # =========================================================================

    # SIMULATION/REPORTING ONLY: verify that Example 2A matches Example 1 when
    # the same desired trajectory is available to both modes.
    max_example_2a_difference_rad = max_command_difference_rad(
        first_positions_rad=JTC_offline_mode.positions,
        second_positions_rad=lookahead_stream_result.optimized_positions_rad,
    )
    print_appendable_stream_comparison(
        label_prefix="Appendable-stream JTC",
        reference_label="Example 1 offline",
        max_difference_rad=max_example_2a_difference_rad,
        appendable_result=lookahead_stream_result,
    )
    if show_timing_analysis:
        print_real_time_timing_analysis(
            label="Example 2A",
            appendable_result=lookahead_stream_result,
            modeled_axes=MODELED_AXES,
            sample_time_s=SAMPLE_TIME_S,
        )
    # SIMULATION/REPORTING ONLY: plot the appendable stream result.
    if show_plots:
        report_and_plot_joint_tracker_stream(
            desired_time_s=time_s,
            desired_positions_rad=desired_trajectory_rad,
            offline_time_s=JTC_offline_mode.time,
            offline_positions_rad=JTC_offline_mode.positions,
            streamed_times_s=list(lookahead_stream_result.optimized_time_s),
            streamed_positions_rad=list(
                lookahead_stream_result.optimized_positions_rad
            ),
            window_records=lookahead_stream_result.window_records,
            modeled_axes=MODELED_AXES,
            sample_time_s=SAMPLE_TIME_S,
            motion_end_time_s=motion_end_time_s,
            prefill_time_s=0.0,
            show_stream_diagnostics=show_timing_analysis,
        )

    # =========================================================================
    # Start Example 2B: Optimize a trajectory generated by an online planner.
    #
    #
    # CONTROLLER CODE TO COPY: use this pattern when your online planner generates
    # trajectory samples while the robot is moving, including pauses between
    # point-to-point moves. JTC keeps holding the last desired sample until more
    # samples arrive.
    # =========================================================================
    initializing_JTC = JointTrackerInterface(
        sample_time=SAMPLE_TIME_S,
        num_joints=NUM_JOINTS,
        model_directory=str(MODEL_DIRECTORY),
        num_axes=NUM_AXES,
        axis_indices=list(MODELED_AXES),
    )
    JTC_online_mode = initializing_JTC.process_trajectory_stream_mode()
    # SIMULATION ONLY: this helper mimics an online planner that sometimes
    # delays new trajectory samples. Replace it with your actual planner state.
    simulated_planner_state, desired_trajectory_beginning_rad = (
        create_appendable_planner_simulation_state(
            source_positions_rad=desired_trajectory_rad,
            initial_chunk_samples=56,
            append_chunk_samples=60,
            low_watermark_samples=10,
            delay_after_source_samples=(56, 111, 170),
            delay_windows=(3, 3, 4),
        )
    )
    # CONTROLLER CODE TO COPY: append the first desired samples before starting
    # the stream. In your application this is the first planner output chunk.
    JTC_online_mode.append_reference(desired_trajectory_beginning_rad)
    JTC_online_mode.start()

    # SIMULATION/REPORTING ONLY: these lists store emitted windows for plots.
    online_stream_times_s: list[float] = []
    online_stream_positions_rad: list[np.ndarray] = []
    online_stream_records: list[WindowStreamRecord] = []
    # CONTROLLER CODE TO COPY: each loop iteration consumes one optimized command
    # window and appends more desired samples when the planner has produced them.
    while not JTC_online_mode.is_complete():
        pop_start_wall_s = time.perf_counter()
        optimized_command_window = JTC_online_mode.pop()
        pop_wait_time_s = time.perf_counter() - pop_start_wall_s
        if optimized_command_window is None:
            if JTC_online_mode.is_complete():
                break
            raise RuntimeError(
                "Appendable Joint Tracker stream did not produce a window."
            )

        # REPLACE WITH YOUR ROBOT SDK: send `optimized_command_window.positions`
        # to the robot here. Keep the helper below only when running this demo.
        append_window_payload_from_payload_time(
            window_payload=optimized_command_window,
            streamed_times_s=online_stream_times_s,
            streamed_positions_rad=online_stream_positions_rad,
            window_records=online_stream_records,
            pop_wait_time_s=pop_wait_time_s,
        )

        # SIMULATION ONLY: this computes whether the fake planner has produced
        # more desired samples. Replace it with your own planner output check.
        planner_decision = next_appendable_planner_decision(
            planner_state=simulated_planner_state,
            available_reference_samples=JTC_online_mode.available_reference_samples(),
            emitted_stop_index=int(optimized_command_window.stop_index),
        )
        if planner_decision.command_to_append is not None:
            # CONTROLLER CODE TO COPY: append newly planned desired samples.
            JTC_online_mode.append_reference(planner_decision.command_to_append)
        if planner_decision.should_finish:
            # CONTROLLER CODE TO COPY: call finish when no more desired samples
            # will be appended.
            JTC_online_mode.finish()

    JTC_online_mode.close()

    # =========================================================================
    # End Example 2B.
    # =========================================================================

    # SIMULATION/REPORTING ONLY: package emitted samples for plots and metrics.
    online_stream_result = build_appendable_planner_result(
        appendable_stream=JTC_online_mode,
        streamed_times_s=online_stream_times_s,
        streamed_positions_rad=online_stream_positions_rad,
        window_records=online_stream_records,
        sample_time_s=SAMPLE_TIME_S,
        appended_source_samples=desired_trajectory_rad.shape[0],
    )

    # SIMULATION/REPORTING ONLY: build a benchmark for the delayed reference.
    delayed_velocity_rad_s, delayed_acceleration_rad_s2 = estimate_derivatives(
        trajectory_rad=online_stream_result.desired_positions_rad,
        sample_time_s=SAMPLE_TIME_S,
    )
    initializing_JTC = JointTrackerInterface(
        sample_time=SAMPLE_TIME_S,
        num_joints=NUM_JOINTS,
        model_directory=str(MODEL_DIRECTORY),
        num_axes=NUM_AXES,
        axis_indices=list(MODELED_AXES),
    )
    JTC_delayed_reference_offline_mode = initializing_JTC.process_trajectory(
        command=online_stream_result.desired_positions_rad,
        command_dot=delayed_velocity_rad_s,
        command_ddot=delayed_acceleration_rad_s2,
        time_vector=list(online_stream_result.desired_time_s),
    )

    # SIMULATION/REPORTING ONLY: compare and plot the delayed-stream result.
    max_example_2b_difference_rad = max_command_difference_rad(
        first_positions_rad=JTC_delayed_reference_offline_mode.positions,
        second_positions_rad=online_stream_result.optimized_positions_rad,
    )
    if show_online_planner_analysis:
        print_appendable_stream_comparison(
            label_prefix="Delayed appendable-stream JTC",
            reference_label="delayed-reference offline benchmark",
            max_difference_rad=max_example_2b_difference_rad,
            appendable_result=online_stream_result,
        )
    if show_timing_analysis:
        print_real_time_timing_analysis(
            label="Example 2B",
            appendable_result=online_stream_result,
            modeled_axes=MODELED_AXES,
            sample_time_s=SAMPLE_TIME_S,
        )
    # report_and_plot_joint_tracker_stream(
    #     desired_time_s=online_stream_result.desired_time_s,
    #     desired_positions_rad=online_stream_result.desired_positions_rad,
    #     offline_time_s=JTC_delayed_reference_offline_mode.time,
    #     offline_positions_rad=JTC_delayed_reference_offline_mode.positions,
    #     streamed_times_s=online_stream_result.optimized_time_s,
    #     streamed_positions_rad=online_stream_result.optimized_positions_rad,
    #     window_records=online_stream_result.window_records,
    #     modeled_axes=MODELED_AXES,
    #     sample_time_s=SAMPLE_TIME_S,
    #     motion_end_time_s=float(online_stream_result.desired_time_s[-1]),
    #     prefill_time_s=0.0,
    # )


if __name__ == "__main__":
    cli_args = parse_cli_args()
    main(
        show_timing_analysis=cli_args.show_timing_analysis,
        show_online_planner_analysis=cli_args.show_online_planner_analysis,
    )
