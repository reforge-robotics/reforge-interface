"""Compare C++ artifacts with the Standard Bots Python native backend."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

from robot.example_usage.shaper.python.example_utility import (
    concatenate_windowed_outputs,
    estimate_derivatives,
    generate_point_to_point_sample,
    generate_point_to_point_trajectory,
    infer_dominant_modal_plant,
    make_speed_first_switch_limits,
    residual_vibration_rad,
    simulate_modal_position_response,
)
from reforge_core.control.shaper import (
    ResidualShapingStrategy,
    ShaperBackendKind,
    ShaperInterface,
)


SAMPLE_TIME_S = 0.004
NUM_AXES = 3
NUM_JOINTS = 6
SHAPED_AXIS = 0
MOVE_DURATION_S = 0.2
DWELL_DURATION_S = 0.8
SLOWER_MOVE_DURATION_S = 0.6
RESIDUAL_WINDOW_DURATION_S = 0.12
RESIDUAL_TRANSITION_MARGIN_S = 0.04
WINDOW_DURATION_S = 0.20


def _parse_args() -> argparse.Namespace:
    """Parse paths supplied by CTest.

    Returns:
        Parsed executable, asset, and frozen-metric paths.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--assets-dir", type=Path, required=True)
    parser.add_argument("--expected-metrics", type=Path, required=True)
    return parser.parse_args()


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of one deterministic asset.

    Args:
        path: File to hash.

    Returns:
        Lowercase hexadecimal SHA-256 digest.
    """

    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_native_extension_override() -> None:
    """Load a source-matched native extension when explicitly provided.

    `REFORGE_SHAPER_NATIVE_EXTENSION` is used only by source-tree qualification;
    installed-package parity resolves the normal packaged extension.
    """

    extension_path = os.environ.get("REFORGE_SHAPER_NATIVE_EXTENSION")
    if extension_path is None:
        return
    module_name = "reforge_core.control._native_shaper"
    specification = importlib.util.spec_from_file_location(module_name, extension_path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"could not load native extension: {extension_path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    specification.loader.exec_module(module)


def _make_shaper(
    assets_dir: Path,
    *,
    stream: bool = False,
) -> ShaperInterface:
    """Construct one Python native backend from the shared Standard Bots assets.

    Args:
        assets_dir: Shared example asset directory.
        stream: Whether to select the per-axis streaming policy.

    Returns:
        Fresh native Shaper session.
    """

    return ShaperInterface(
        sample_time=SAMPLE_TIME_S,
        model_directory=str(assets_dir / "model"),
        urdf_filepath=str(assets_dir / "modelone.urdf"),
        num_axes=NUM_AXES,
        num_joints=NUM_JOINTS,
        backend_kind=ShaperBackendKind.NATIVE,
        shared_impulse_policy="per_axis" if stream else "combine_all_modes",
        shared_impulse_shapes_all_joints=not stream,
    )


def _run_python_reference(assets_dir: Path) -> dict[str, Any]:
    """Run all four Python SDK paths against the Standard Bots native bundle.

    Args:
        assets_dir: Shared example asset directory.

    Returns:
        Reference trajectories and hardware-free residual metrics.
    """

    start_position_rad = np.zeros(NUM_JOINTS, dtype=float)
    goal_position_rad = np.zeros(NUM_JOINTS, dtype=float)
    goal_position_rad[SHAPED_AXIS] = 0.35
    time_s, desired_rad = generate_point_to_point_trajectory(
        start_position_rad,
        goal_position_rad,
        SAMPLE_TIME_S,
        MOVE_DURATION_S,
        DWELL_DURATION_S,
    )
    velocity_rad_s, acceleration_rad_s2 = estimate_derivatives(
        desired_rad, SAMPLE_TIME_S
    )

    always_on_shaper = _make_shaper(assets_dir)
    always_on = always_on_shaper.process_trajectory(
        command=desired_rad,
        command_dot=velocity_rad_s,
        command_ddot=acceleration_rad_s2,
        time_vector=list(time_s),
        vibration_shaping_weight=1.0,
        residual_shaping_strategy=None,
        finalize_tail=False,
    )

    limits = make_speed_first_switch_limits(
        max_velocity_rad_s=[3.0] * NUM_JOINTS,
        max_acceleration_rad_s2=[45.0] * NUM_JOINTS,
        max_search_s=0.75,
        max_qp_attempts=10,
    )
    residual_shaper = _make_shaper(assets_dir)
    residual_tail = residual_shaper.process_trajectory(
        command=desired_rad,
        command_dot=velocity_rad_s,
        command_ddot=acceleration_rad_s2,
        time_vector=list(time_s),
        vibration_shaping_weight=1.0,
        residual_shaping_strategy=ResidualShapingStrategy.ALIGNED_TAIL,
        residual_switch_limits=limits,
        residual_transition_margin_s=RESIDUAL_TRANSITION_MARGIN_S,
        finalize_tail=False,
    )

    windowed_shaper = _make_shaper(assets_dir)
    windowed_buffer = windowed_shaper.create_windowed_buffer(
        command=desired_rad,
        command_dot=velocity_rad_s,
        command_ddot=acceleration_rad_s2,
        time_vector=list(time_s),
        vibration_shaping_weight=1.0,
        residual_shaping_strategy=None,
        window_s=WINDOW_DURATION_S,
        auto_qualify_window=False,
        finalize_tail=False,
    )
    windowed_buffer.fill_available()
    windows = []
    while windowed_buffer.has_next():
        window = windowed_buffer.pop_window()
        if window is None:
            windowed_buffer.fill_available(max_windows=1)
        else:
            windows.append(window)
    windowed_time_s, windowed_positions_rad = concatenate_windowed_outputs(windows)
    windowed_velocity_rad_s = np.vstack([window.velocities for window in windows])
    windowed_acceleration_rad_s2 = np.vstack(
        [window.accelerations for window in windows]
    )

    streaming_shaper = _make_shaper(assets_dir, stream=True)
    streamed_positions = []
    streamed_velocities = []
    streamed_accelerations = []
    for sample_time_s in time_s:
        command_rad, command_velocity_rad_s, command_acceleration_rad_s2 = (
            generate_point_to_point_sample(
                start_position_rad,
                goal_position_rad,
                float(sample_time_s),
                MOVE_DURATION_S,
            )
        )
        sample = streaming_shaper.process_sample(
            command=command_rad,
            command_dot=command_velocity_rad_s,
            command_ddot=command_acceleration_rad_s2,
            vibration_shaping_weight=1.0,
        )
        streamed_positions.append(sample.positions)
        streamed_velocities.append(sample.velocities)
        streamed_accelerations.append(sample.accelerations)
    streamed_positions_rad = np.vstack(streamed_positions)

    plant = infer_dominant_modal_plant(
        always_on_shaper,
        representative_command_rad=desired_rad[0],
        axis_index=SHAPED_AXIS,
    )
    slower_time_s, slower_positions_rad = generate_point_to_point_trajectory(
        start_position_rad,
        goal_position_rad,
        SAMPLE_TIME_S,
        SLOWER_MOVE_DURATION_S,
        DWELL_DURATION_S,
    )
    residual_start_time_s = (
        MOVE_DURATION_S + DWELL_DURATION_S - RESIDUAL_WINDOW_DURATION_S
    )
    slower_residual_start_time_s = (
        SLOWER_MOVE_DURATION_S + DWELL_DURATION_S - RESIDUAL_WINDOW_DURATION_S
    )

    def residual(
        response_time_s: np.ndarray,
        response_positions_rad: np.ndarray,
        start_time_s: float,
    ) -> float:
        """Simulate and measure one shaped or unshaped command.

        Args:
            response_time_s: Command times [s].
            response_positions_rad: Joint commands [rad].
            start_time_s: Inclusive residual start time [s].

        Returns:
            Maximum residual vibration [rad].
        """

        response = simulate_modal_position_response(
            response_time_s,
            response_positions_rad[:, SHAPED_AXIS],
            plant,
        )
        return residual_vibration_rad(
            response, goal_position_rad[SHAPED_AXIS], start_time_s
        )

    desired_at_residual_time = np.interp(
        residual_tail.time, time_s, desired_rad[:, SHAPED_AXIS]
    )
    difference = np.abs(
        residual_tail.positions[:, SHAPED_AXIS] - desired_at_residual_time
    )
    changed = np.flatnonzero(difference > 1.0e-9)
    deceleration_index = int(np.argmax(velocity_rad_s[:, SHAPED_AXIS]))
    change_start_time_s = (
        float(residual_tail.time[changed[0]])
        if changed.size
        else float(time_s[deceleration_index])
    )
    metrics = {
        "dominant_natural_frequency_rad_s": plant.natural_frequency_rad_s,
        "dominant_damping_ratio": plant.damping_ratio,
        "residual_start_time_s": residual_start_time_s,
        "unshaped_residual_rad": residual(time_s, desired_rad, residual_start_time_s),
        "slower_unshaped_residual_rad": residual(
            slower_time_s, slower_positions_rad, slower_residual_start_time_s
        ),
        "deceleration_start_time_s": float(time_s[deceleration_index]),
        "residual_change_start_time_s": change_start_time_s,
        "always_on_residual_rad": residual(
            always_on.time, always_on.positions, residual_start_time_s
        ),
        "residual_tail_residual_rad": residual(
            residual_tail.time, residual_tail.positions, residual_start_time_s
        ),
        "windowed_residual_rad": residual(
            windowed_time_s, windowed_positions_rad, residual_start_time_s
        ),
        "streamed_residual_rad": residual(
            time_s, streamed_positions_rad, residual_start_time_s
        ),
    }
    return {
        "desired": (time_s, desired_rad, velocity_rad_s, acceleration_rad_s2),
        "example_1_always_on": (
            always_on.time,
            always_on.positions,
            always_on.velocities,
            always_on.accelerations,
        ),
        "example_2_residual_tail": (
            residual_tail.time,
            residual_tail.positions,
            residual_tail.velocities,
            residual_tail.accelerations,
        ),
        "example_3_fixed_windows": (
            windowed_time_s,
            windowed_positions_rad,
            windowed_velocity_rad_s,
            windowed_acceleration_rad_s2,
        ),
        "example_4_sample_stream": (
            time_s,
            streamed_positions_rad,
            np.vstack(streamed_velocities),
            np.vstack(streamed_accelerations),
        ),
        "metrics": metrics,
    }


def _read_cpp_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Read one deterministic C++ trajectory artifact.

    Args:
        path: CSV artifact path.

    Returns:
        Time, position, velocity, and acceleration arrays.
    """

    values = np.loadtxt(path, delimiter=",", skiprows=1)
    return (
        values[:, 0],
        values[:, 1:7],
        values[:, 7:13],
        values[:, 13:19],
    )


def _compare_trajectory(
    label: str,
    cpp: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    python: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    expected: dict[str, Any],
) -> None:
    """Assert exact shape and frozen per-field numeric tolerances.

    Args:
        label: Example identity used in assertion messages.
        cpp: C++ time and kinematics arrays.
        python: Python time and kinematics arrays.
        expected: Frozen tolerance manifest.
    """

    tolerances = (
        expected["time_tolerance_s"],
        expected["position_tolerance_rad"],
        expected["velocity_tolerance_rad_s"],
        expected["acceleration_tolerance_rad_s2"],
    )
    for field, cpp_values, python_values, tolerance in zip(
        ("time", "position", "velocity", "acceleration"),
        cpp,
        python,
        tolerances,
        strict=True,
    ):
        if cpp_values.shape != python_values.shape:
            raise AssertionError(
                f"{label} {field} shape differs: "
                f"{cpp_values.shape} != {python_values.shape}"
            )
        if not np.isfinite(cpp_values).all():
            raise AssertionError(f"{label} {field} contains a non-finite C++ value")
        np.testing.assert_allclose(
            cpp_values,
            python_values,
            atol=tolerance,
            rtol=0.0 if field == "time" else 1.0e-8,
            err_msg=f"{label} {field} differs",
        )


def main() -> None:
    """Run the C++ executable and compare every output with Python native."""

    args = _parse_args()
    _load_native_extension_override()
    expected = json.loads(args.expected_metrics.read_text())
    if (
        _sha256(args.assets_dir / "model" / "shaper_models.native.json")
        != expected["artifact_sha256"]
    ):
        raise AssertionError(
            "native model artifact is not the frozen Standard Bots bundle"
        )
    if _sha256(args.assets_dir / "modelone.urdf") != expected["urdf_sha256"]:
        raise AssertionError(
            "URDF is not the matching frozen Standard Bots modelone.urdf"
        )

    python_reference = _run_python_reference(args.assets_dir)
    with tempfile.TemporaryDirectory(prefix="shaper-cpp-parity-") as temporary:
        output_dir = Path(temporary)
        completed = subprocess.run(
            [
                str(args.binary),
                "--assets-dir",
                str(args.assets_dir),
                "--output-dir",
                str(output_dir),
                "--headless",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            raise AssertionError(
                "C++ example failed during parity test:\n"
                + completed.stdout
                + completed.stderr
            )
        for label in (
            "desired",
            "example_1_always_on",
            "example_2_residual_tail",
            "example_3_fixed_windows",
            "example_4_sample_stream",
        ):
            _compare_trajectory(
                label,
                _read_cpp_csv(output_dir / f"{label}.csv"),
                python_reference[label],
                expected,
            )
        cpp_metrics = json.loads((output_dir / "metrics.json").read_text())

    for label, frozen_value in expected["metrics"].items():
        np.testing.assert_allclose(
            python_reference["metrics"][label],
            frozen_value,
            atol=expected["metric_tolerance"],
            rtol=0.0,
            err_msg=f"Python Standard Bots metric drifted: {label}",
        )
        np.testing.assert_allclose(
            cpp_metrics[label],
            python_reference["metrics"][label],
            atol=expected["metric_tolerance"],
            rtol=0.0,
            err_msg=f"C++ Standard Bots metric differs: {label}",
        )
    np.testing.assert_allclose(
        cpp_metrics["dominant_natural_frequency_rad_s"],
        expected["dominant_natural_frequency_rad_s"],
        atol=expected["metric_tolerance"],
        rtol=0.0,
    )
    np.testing.assert_allclose(
        cpp_metrics["dominant_damping_ratio"],
        expected["dominant_damping_ratio"],
        atol=expected["metric_tolerance"],
        rtol=0.0,
    )
    print("Standard Bots Python/C++ native parity: PASS")


if __name__ == "__main__":
    main()
