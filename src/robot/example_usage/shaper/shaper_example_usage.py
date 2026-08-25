"""Demonstrate Covalent Shaper in offline and streaming control modes."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from robot.example_usage.shaper.example_utility import (
    concatenate_windowed_outputs,
    estimate_derivatives,
    generate_point_to_point_sample,
    generate_point_to_point_trajectory,
    make_speed_first_switch_limits,
    plot_shaper_example_results,
)
from reforge_core.control.shaper import (
    ResidualShapingStrategy,
    ShaperBackendKind,
    ShaperInterface,
)

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[3]

SAMPLE_TIME_S = 0.004
NUM_AXES = 1
NUM_JOINTS = 6
SHAPED_AXIS = 0
MOVE_DURATION_S = 0.2
DWELL_DURATION_S = 0.8
RESIDUAL_WINDOW_DURATION_S = 0.12
WINDOW_DURATION_S = 0.20
SHAPER_ENABLED_WEIGHT = 1.0

MODEL_DIRECTORY = THIS_DIR
URDF_FILEPATH = REPO_ROOT / "src/robot/urdf/test_robot.urdf"

SWITCH_MAX_VELOCITY_RAD_S = 3.0
SWITCH_MAX_ACCELERATION_RAD_S2 = 45.0
SWITCH_MAX_SEARCH_S = 0.75
SWITCH_WINDOW_TARGET_MARGIN_S = 0.0
SWITCH_MAX_QP_ATTEMPTS = 10

# This example ships a Torch model for deterministic Python-backend metrics.
EXAMPLE_BACKEND_KIND = ShaperBackendKind.PYTHON


def main() -> None:
    """Run offline and streaming Covalent Shaper examples.

    Args:
        None.
    Returns:
        `None`.
    Raises:
        RuntimeError: If Shaper initialization or simulation fails.
    """

    start_position_rad = np.zeros(NUM_JOINTS, dtype=float)
    goal_position_rad = np.zeros(NUM_JOINTS, dtype=float)
    goal_position_rad[SHAPED_AXIS] = 0.35

    time_s, desired_trajectory_rad = generate_point_to_point_trajectory(
        start_position_rad=start_position_rad,
        goal_position_rad=goal_position_rad,
        sample_time_s=SAMPLE_TIME_S,
        move_duration_s=MOVE_DURATION_S,
        dwell_duration_s=DWELL_DURATION_S,
    )
    desired_velocity_rad_s, desired_acceleration_rad_s2 = estimate_derivatives(
        desired_trajectory_rad,
        SAMPLE_TIME_S,
    )

    # =========================================================================
    # Start Example 1: Shape a complete trajectory with Shaper enabled from the
    # start.
    # =========================================================================
    always_on_shaper = ShaperInterface(
        sample_time=SAMPLE_TIME_S,
        model_directory=str(MODEL_DIRECTORY),
        urdf_filepath=str(URDF_FILEPATH),
        num_axes=NUM_AXES,
        num_joints=NUM_JOINTS,
        backend_kind=EXAMPLE_BACKEND_KIND,
    )
    always_on_shaped = always_on_shaper.process_trajectory(
        command=desired_trajectory_rad,
        command_dot=desired_velocity_rad_s,
        command_ddot=desired_acceleration_rad_s2,
        time_vector=list(time_s),
        vibration_shaping_weight=SHAPER_ENABLED_WEIGHT,
        residual_shaping_strategy=None,
        finalize_tail=False,
    )
    # =========================================================================
    # End Example 1.
    # =========================================================================

    # =========================================================================
    # Start Example 2: Shape only the residual tail of a planned trajectory.
    # =========================================================================
    speed_first_switch_limits = make_speed_first_switch_limits(
        max_velocity_rad_s=SWITCH_MAX_VELOCITY_RAD_S,
        max_acceleration_rad_s2=SWITCH_MAX_ACCELERATION_RAD_S2,
        max_search_s=SWITCH_MAX_SEARCH_S,
        max_qp_attempts=SWITCH_MAX_QP_ATTEMPTS,
        window_target_margin_s=SWITCH_WINDOW_TARGET_MARGIN_S,
    )
    # To prioritize smoothness, call `make_smoothness_first_switch_limits(...)`
    # with weights selected by a representative application-level sweep. To
    # enforce a hard jerk bound, call `make_jerk_limited_switch_limits(...)`
    # with a max_jerk_rad_s3 value qualified for the target robot and payload.
    # Keep one selected profile here so batch and windowed results are directly
    # comparable without running several expensive variants in this example.
    residual_offline_shaper = ShaperInterface(
        sample_time=SAMPLE_TIME_S,
        model_directory=str(MODEL_DIRECTORY),
        urdf_filepath=str(URDF_FILEPATH),
        num_axes=NUM_AXES,
        num_joints=NUM_JOINTS,
        backend_kind=EXAMPLE_BACKEND_KIND,
    )
    residual_offline_shaped = residual_offline_shaper.process_trajectory(
        command=desired_trajectory_rad,
        command_dot=desired_velocity_rad_s,
        command_ddot=desired_acceleration_rad_s2,
        time_vector=list(time_s),
        vibration_shaping_weight=SHAPER_ENABLED_WEIGHT,
        residual_shaping_strategy=ResidualShapingStrategy.ALIGNED_TAIL,
        residual_switch_limits=speed_first_switch_limits,
        finalize_tail=False,
    )
    # =========================================================================
    # End Example 2.
    # =========================================================================

    # =========================================================================
    # Start Example 3: Shape a complete trajectory in fixed-size windows.
    # =========================================================================
    windowed_shaper = ShaperInterface(
        sample_time=SAMPLE_TIME_S,
        model_directory=str(MODEL_DIRECTORY),
        urdf_filepath=str(URDF_FILEPATH),
        num_axes=NUM_AXES,
        num_joints=NUM_JOINTS,
        backend_kind=EXAMPLE_BACKEND_KIND,
    )
    windowed_buffer = windowed_shaper.create_windowed_buffer(
        command=desired_trajectory_rad,
        command_dot=desired_velocity_rad_s,
        command_ddot=desired_acceleration_rad_s2,
        time_vector=list(time_s),
        vibration_shaping_weight=SHAPER_ENABLED_WEIGHT,
        residual_shaping_strategy=None,
        window_s=WINDOW_DURATION_S,
        auto_qualify_window=False,
        finalize_tail=False,
    )

    # Windowed Shaper usage:
    #
    # `create_windowed_buffer(...)` returns a FIFO-style buffer. In an offline
    # script, drain it to reconstruct the full shaped trajectory. In an online
    # robot loop, call `fill_available(...)` from the producer side and stream
    # each `pop_window()` output to the robot SDK in timestamp order.
    windowed_buffer.fill_available()
    shaped_windows = []
    while windowed_buffer.has_next():
        shaped_window = windowed_buffer.pop_window()
        if shaped_window is not None:
            shaped_windows.append(shaped_window)
            continue
        windowed_buffer.fill_available(max_windows=1)

    windowed_time_s, windowed_positions_rad = concatenate_windowed_outputs(
        shaped_windows
    )
    # =========================================================================
    # End Example 3.
    # =========================================================================

    # =========================================================================
    # Start Example 4: Shape one command sample at a time in an online loop.
    # =========================================================================
    streaming_shaper = ShaperInterface(
        sample_time=SAMPLE_TIME_S,
        model_directory=str(MODEL_DIRECTORY),
        urdf_filepath=str(URDF_FILEPATH),
        num_axes=NUM_AXES,
        num_joints=NUM_JOINTS,
        backend_kind=EXAMPLE_BACKEND_KIND,
        shared_impulse_policy="per_axis",
        shared_impulse_shapes_all_joints=False,
    )

    # These lists are only used later to plot the example output. A real robot
    # loop would usually send each `shaped_sample` directly to the robot SDK and
    # would not need to store the full stream.
    streamed_times_s: list[float] = []
    streamed_positions_rad: list[np.ndarray] = []

    # This for-loop represents a real-time robot command loop running every
    # `SAMPLE_TIME_S`. In production, the loop duration is normally controlled by
    # the robot SDK, a realtime scheduler, or a ROS timer.
    stream_sample_count = (
        int(round((MOVE_DURATION_S + DWELL_DURATION_S) / SAMPLE_TIME_S)) + 1
    )
    for sample_index in range(stream_sample_count):
        sample_time_s = sample_index * SAMPLE_TIME_S

        # Example-only online trajectory generation:
        #
        # `generate_point_to_point_sample(...)` is here only to simulate an
        # online planner that produces the next command at the current control
        # cycle. Replace this function call with your own real-time planner,
        # joystick command, force controller, task-space controller, or robot
        # SDK trajectory generator.
        #
        # The important detail is that only the current command sample is known
        # at this point. The Shaper is not receiving the full trajectory.
        command_rad, velocity_rad_s, acceleration_rad_s2 = (
            generate_point_to_point_sample(
                start_position_rad=start_position_rad,
                goal_position_rad=goal_position_rad,
                current_time_s=sample_time_s,
                move_duration_s=MOVE_DURATION_S,
            )
        )

        # Online Shaper usage:
        #
        # `process_sample(...)` is the call a user should copy into a real-time
        # control loop. It receives the command for the current sample and
        # returns shaped position, velocity, and acceleration commands for that
        # same control cycle.
        shaped_sample = streaming_shaper.process_sample(
            command=command_rad,
            command_dot=velocity_rad_s,
            command_ddot=acceleration_rad_s2,
            vibration_shaping_weight=SHAPER_ENABLED_WEIGHT,
        )

        # Robot command output:
        #
        # In a real application, send `shaped_sample.positions`,
        # `shaped_sample.velocities`, and `shaped_sample.accelerations` to the
        # robot SDK here. The append calls below are only for plotting this demo.
        streamed_times_s.append(sample_time_s)
        streamed_positions_rad.append(shaped_sample.positions)
    # =========================================================================
    # End Example 4.
    # =========================================================================

    plot_shaper_example_results(
        shaper=always_on_shaper,
        desired_time_s=time_s,
        desired_trajectory_rad=desired_trajectory_rad,
        always_on_time_s=always_on_shaped.time,
        always_on_positions_rad=always_on_shaped.positions,
        residual_offline_time_s=residual_offline_shaped.time,
        residual_offline_positions_rad=residual_offline_shaped.positions,
        streamed_times_s=streamed_times_s,
        streamed_positions_rad=streamed_positions_rad,
        windowed_time_s=windowed_time_s,
        windowed_positions_rad=windowed_positions_rad,
        goal_position_rad=goal_position_rad,
        shaped_axis=SHAPED_AXIS,
        move_duration_s=MOVE_DURATION_S,
        dwell_duration_s=DWELL_DURATION_S,
        residual_window_duration_s=RESIDUAL_WINDOW_DURATION_S,
    )


if __name__ == "__main__":
    main()
