"""Utility functions for the Covalent KineCal control example."""

from __future__ import annotations

from dataclasses import dataclass
from os import PathLike
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from reforge_core.control.kinecal.compensation import CompensationSelectionInfo
from reforge_core.util.robot_dynamics import Dynamics
from reforge_core.util.timing.joint_limits import JointStateBox, JointStateLimits
from reforge_core.util.timing.joint_trajectory import JointTrajectory
from reforge_core.util.trajopt.traj_qp import TrajOptReport, plan_trajectory_qp


@dataclass(frozen=True)
class TerminalPoseValidation:
    """Store terminal pose agreement between nominal and compensated models.

    Args:
        common_link_name: Most distal link shared by the baseline and calibrated
            URDFs.
        baseline_position_m: Nominal baseline common-link position [m].
        calibrated_position_m: Compensated calibrated common-link position [m].
        position_delta_m: Euclidean position error between common-link poses [m].
        orientation_delta_rad: Relative orientation error between common-link
            poses [rad].
    """

    common_link_name: str
    baseline_position_m: np.ndarray
    calibrated_position_m: np.ndarray
    position_delta_m: float
    orientation_delta_rad: float


def create_joint_limits_from_urdf(
    baseline_urdf_path: str | PathLike[str],
    max_joint_speed_rad_s: float,
    max_joint_acceleration_rad_s2: float,
) -> JointStateLimits:
    """Create planning limits from a baseline URDF and fixed derivative limits.

    Args:
        baseline_urdf_path: Baseline robot URDF path.
        max_joint_speed_rad_s: Symmetric joint velocity limit [rad/s].
        max_joint_acceleration_rad_s2: Symmetric joint acceleration limit [rad/s^2].
    Returns:
        `JointStateLimits` with URDF position limits and symmetric velocity and
        acceleration limits.
    Raises:
        FileNotFoundError: If `baseline_urdf_path` does not exist.
        ValueError: If derivative limits are invalid.
    """

    if max_joint_speed_rad_s <= 0.0:
        raise ValueError("max_joint_speed_rad_s must be positive.")
    if max_joint_acceleration_rad_s2 <= 0.0:
        raise ValueError("max_joint_acceleration_rad_s2 must be positive.")

    baseline_dynamics = Dynamics(baseline_urdf_path)
    position_lower_rad, position_upper_rad = baseline_dynamics.joint_limits
    joint_count = int(position_lower_rad.shape[0])
    return JointStateLimits(
        q=JointStateBox(lower=position_lower_rad, upper=position_upper_rad),
        qd=JointStateBox(
            lower=np.full(joint_count, -max_joint_speed_rad_s, dtype=float),
            upper=np.full(joint_count, max_joint_speed_rad_s, dtype=float),
        ),
        qdd=JointStateBox(
            lower=np.full(joint_count, -max_joint_acceleration_rad_s2, dtype=float),
            upper=np.full(joint_count, max_joint_acceleration_rad_s2, dtype=float),
        ),
    )


def plan_nominal_joint_trajectory(
    start_position_rad: np.ndarray,
    goal_position_rad: np.ndarray,
    limits: JointStateLimits,
    sample_count: int,
    sample_time_s: float,
) -> tuple[JointTrajectory, TrajOptReport]:
    """Plan a smooth nominal joint trajectory in the baseline URDF joint space.

    Args:
        start_position_rad: Initial joint positions [rad], shape `(num_joints,)`.
        goal_position_rad: Final joint positions [rad], shape `(num_joints,)`.
        limits: Joint position, velocity, and acceleration limits.
        sample_count: Number of planned trajectory samples [count].
        sample_time_s: Sample period [s].
    Returns:
        `tuple[JointTrajectory, TrajOptReport]`: planned nominal trajectory and
        optimizer report.
    Raises:
        ValueError: If inputs are dimensionally inconsistent or timing is invalid.
        RuntimeError: If the trajectory optimizer fails.
    """

    start_position_rad = np.asarray(start_position_rad, dtype=float)
    goal_position_rad = np.asarray(goal_position_rad, dtype=float)
    if start_position_rad.shape != goal_position_rad.shape:
        raise ValueError("start_position_rad and goal_position_rad must match.")
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2.")
    if sample_time_s <= 0.0:
        raise ValueError("sample_time_s must be positive.")

    zero_velocity_rad_s = np.zeros_like(start_position_rad)
    zero_acceleration_rad_s2 = np.zeros_like(start_position_rad)
    return plan_trajectory_qp(
        q0=start_position_rad,
        qd0=zero_velocity_rad_s,
        qdd0=zero_acceleration_rad_s2,
        q1=goal_position_rad,
        qd1=zero_velocity_rad_s,
        qdd1=zero_acceleration_rad_s2,
        limits=limits,
        N=sample_count,
        dt=sample_time_s,
        w_acc=1.0,
        w_jerk=1e-3,
    )


def selected_calibrated_urdf_path(
    selection_info: CompensationSelectionInfo,
) -> Path | None:
    """Return the calibrated URDF used by a non-fallback KineCal selection.

    Args:
        selection_info: KineCal selection metadata returned by compensation.
    Returns:
        `Path | None` calibrated URDF path when compensation used one.
    """

    if selection_info.used_baseline_fallback:
        return None
    if selection_info.selected_result is None:
        return None
    return selection_info.selected_result.calibrated_urdf_path


def validate_terminal_common_link_pose(
    baseline_urdf_path: str | PathLike[str],
    calibrated_urdf_path: str | PathLike[str],
    nominal_terminal_position_rad: np.ndarray,
    compensated_terminal_position_rad: np.ndarray,
) -> TerminalPoseValidation:
    """Compare the preserved terminal common-link pose across two URDFs.

    Args:
        baseline_urdf_path: Baseline robot URDF path.
        calibrated_urdf_path: Calibrated robot URDF path selected by KineCal.
        nominal_terminal_position_rad: Nominal terminal joint positions [rad].
        compensated_terminal_position_rad: Compensated terminal joint positions [rad].
    Returns:
        `TerminalPoseValidation` with position and orientation error metrics.
    Raises:
        FileNotFoundError: If either URDF path does not exist.
        ValueError: If the URDFs share no common link.
        KeyError: If FK cannot be evaluated for the resolved common link.
    """

    baseline_dynamics = Dynamics(baseline_urdf_path)
    calibrated_dynamics = Dynamics(calibrated_urdf_path)
    common_link_name = _resolve_distal_common_link_name(
        baseline_link_names=baseline_dynamics.link_names,
        calibrated_link_names=calibrated_dynamics.link_names,
    )

    baseline_transform = np.asarray(
        baseline_dynamics.get_transformation_matrix(
            nominal_terminal_position_rad,
            link_name=common_link_name,
        ),
        dtype=float,
    )
    calibrated_transform = np.asarray(
        calibrated_dynamics.get_transformation_matrix(
            compensated_terminal_position_rad,
            link_name=common_link_name,
        ),
        dtype=float,
    )
    position_delta_m = float(
        np.linalg.norm(calibrated_transform[:3, 3] - baseline_transform[:3, 3])
    )
    orientation_delta_rad = float(
        (
            R.from_matrix(calibrated_transform[:3, :3]).inv()
            * R.from_matrix(baseline_transform[:3, :3])
        ).magnitude()
    )
    return TerminalPoseValidation(
        common_link_name=common_link_name,
        baseline_position_m=baseline_transform[:3, 3],
        calibrated_position_m=calibrated_transform[:3, 3],
        position_delta_m=position_delta_m,
        orientation_delta_rad=orientation_delta_rad,
    )


def print_compensation_selection(
    selection_info: CompensationSelectionInfo,
    label: str,
) -> None:
    """Print concise KineCal model-selection metadata.

    Args:
        selection_info: KineCal selection metadata returned by compensation.
        label: Human-readable result label.
    """

    selected_result_dir = (
        selection_info.selected_result.result_dir
        if selection_info.selected_result is not None
        else None
    )
    print(f"\n[{label}]")
    print(f"used_baseline_fallback={selection_info.used_baseline_fallback}")
    print(f"fallback_reason={selection_info.fallback_reason}")
    print(f"selected_result_dir={selected_result_dir}")
    print(
        "closest_socket_distance_m=" f"{selection_info.closest_socket_distance_m:.6f}"
    )


def print_terminal_pose_validation(
    validation: TerminalPoseValidation,
) -> None:
    """Print terminal common-link validation metrics.

    Args:
        validation: Terminal common-link pose validation result.
    """

    print("\n[Terminal Pose Validation]")
    print(f"common_link_name={validation.common_link_name}")
    print(
        "baseline_position_m="
        f"{np.array2string(validation.baseline_position_m, precision=6)}"
    )
    print(
        "calibrated_position_m="
        f"{np.array2string(validation.calibrated_position_m, precision=6)}"
    )
    print(f"position_delta_m={validation.position_delta_m:.6e}")
    print(f"orientation_delta_rad={validation.orientation_delta_rad:.6e}")


def plot_kinecal_example_results(
    reference_trajectory: JointTrajectory,
    compensated_trajectory: JointTrajectory,
    limits: JointStateLimits,
) -> None:
    """Plot nominal and KineCal-compensated joint trajectories.

    Args:
        reference_trajectory: Nominal baseline trajectory.
        compensated_trajectory: KineCal-compensated trajectory.
        limits: Joint position, velocity, and acceleration limits.
    Raises:
        ValueError: If trajectories or limits are not plottable.
    """

    import matplotlib.pyplot as plt
    from reforge_core.util.timing.trajectory_plotting import (
        plot_trajectory_comparison,
    )

    plot_trajectory_comparison(
        reference=reference_trajectory,
        comparison=compensated_trajectory,
        limits=limits,
        reference_label="reference (nominal)",
        comparison_label="compensated",
        title="Reference vs. KineCal-compensated trajectory",
        zoom=True,
        terminal_inset=True,
    )
    plt.show()


def _resolve_distal_common_link_name(
    baseline_link_names: list[str],
    calibrated_link_names: list[str],
) -> str:
    """Return the most distal link name shared by two URDF link lists.

    Args:
        baseline_link_names: Baseline URDF link names in model order.
        calibrated_link_names: Calibrated URDF link names in model order.
    Returns:
        `str` most distal common link name.
    Raises:
        ValueError: If either URDF has no links or the models share no link.
    """

    if not baseline_link_names:
        raise ValueError("baseline URDF must contain at least one link.")
    if not calibrated_link_names:
        raise ValueError("calibrated URDF must contain at least one link.")

    calibrated_link_name_set = set(calibrated_link_names)
    for link_name in reversed(baseline_link_names):
        if link_name in calibrated_link_name_set:
            return link_name

    raise ValueError("Baseline and calibrated URDFs do not share a common link.")
