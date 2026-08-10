"""Compatibility entrypoint for robot command-line operations."""

from __future__ import annotations

from collections.abc import Sequence
from importlib.resources import as_file, files
from pathlib import Path

from reforge_core.calibration import run_helpers
from robot.robot_interface import (
    BOT_ID,
    DEFAULT_IMU_RECORD_FREQUENCY_HZ,
    FULL_STRETCH_JOINTS,
    FULL_STRETCH_POSE_OVERRIDE,
    FULL_STRETCH_QUAT,
    FULL_STRETCH_XYZ,
    MAX_ROBOT_JOINTS_BANDWIDTH,
    ROBOT_MAX_FREQ,
    SIM_DATA_LOCATION_PREFIX,
    URDF_PATH,
    RobotInterface,
)


def main(argv: Sequence[str] | None = None) -> None:
    """Parse robot CLI arguments and dispatch to the SDK-owned implementation.

    May connect to robot hardware, run calibration, or write model files.

    Args:
        argv: Optional command-line arguments. Uses `sys.argv` when omitted.

    Returns:
        `None`.

    Raises:
        SystemExit: If argument parsing fails.
    """
    packaged_urdf = files("robot").joinpath(URDF_PATH)
    with as_file(packaged_urdf) as default_sim_urdf:
        return run_helpers.main(
            robot_interface_class=RobotInterface,
            simulator_configuration=run_helpers.SimulatorConfiguration(
                urdf_path=default_sim_urdf,
                name="My Robot",
                sample_frequency_hz=ROBOT_MAX_FREQ,
                imu_record_frequency_hz=DEFAULT_IMU_RECORD_FREQUENCY_HZ,
                data_folder_prefix=SIM_DATA_LOCATION_PREFIX,
                servo_bandwidth_hz=MAX_ROBOT_JOINTS_BANDWIDTH,
                calibration_start_joints=FULL_STRETCH_JOINTS,
                calibration_start_quat=FULL_STRETCH_QUAT,
                calibration_start_xyz=FULL_STRETCH_XYZ,
                full_stretch_pose_override=FULL_STRETCH_POSE_OVERRIDE,
            ),
            default_robot_id=BOT_ID,
            argv=argv,
            script_path=Path(__file__).resolve(),
        )


if __name__ == "__main__":
    main()
