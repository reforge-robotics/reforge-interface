"""Compatibility entrypoint for robot command-line operations."""

from __future__ import annotations

from collections.abc import Sequence
from contextlib import ExitStack
from importlib.resources import as_file, files
from importlib.resources.abc import Traversable
from pathlib import Path

from reforge_core.calibration import run_helpers
import robot.robot_interface as robot_interface


def _materialize_optional_resource(
    resource_stack: ExitStack,
    package_root: Traversable,
    relative_path: str | None,
) -> Path | None:
    """Materialize an existing package file for the duration of a CLI run.

    ``importlib.resources`` may expose resources from a zip archive, so the
    returned path is valid only while ``resource_stack`` remains open. Missing
    optional files are intentionally represented as ``None`` for public
    packages that do not ship KineCal configuration.

    Args:
        resource_stack: Exit stack that owns the resource extraction lifetime.
        package_root: Root package resource handle.
        relative_path: Package-relative file path, or ``None``.

    Returns:
        A filesystem path for an existing resource, or ``None``.
    """
    if relative_path is None:
        return None
    resource = package_root.joinpath(relative_path)
    if not resource.is_file():
        return None
    return resource_stack.enter_context(as_file(resource))


def main(argv: Sequence[str] | None = None) -> None:
    """Parse robot CLI arguments and dispatch to the SDK-owned implementation.

    May connect to robot hardware, run calibration, or write model files.

    Args:
        argv: Optional command-line arguments. Uses `sys.argv` when omitted.

    Raises:
        SystemExit: If argument parsing fails.
    """
    packaged_resources = files("robot")
    urdf_path = robot_interface.URDF_PATH
    packaged_urdf = packaged_resources.joinpath(urdf_path)
    if not packaged_urdf.is_file():
        raise FileNotFoundError(f"Robot URDF resource not found: {urdf_path}")

    data_location_prefix = robot_interface.DATA_LOCATION_PREFIX
    robot_max_frequency_hz = robot_interface.ROBOT_MAX_FREQ
    imu_record_frequency_hz = getattr(
        robot_interface,
        "DEFAULT_IMU_RECORD_FREQUENCY_HZ",
        robot_max_frequency_hz,
    )
    simulator_data_location_prefix = getattr(
        robot_interface,
        "SIM_DATA_LOCATION_PREFIX",
        str(Path(data_location_prefix) / "sim"),
    )

    # The private facade exports selected integration paths. An unchanged
    # public root adapter has no private spec, so only its conventional root
    # config names are considered, and only when those files are packaged.
    private_resource_spec = getattr(robot_interface, "_INTEGRATION_RESOURCE_SPEC", None)
    kinecal_config_path: str | None
    kinecal_default_config_path: str | None
    kinecal_probe_params_path: str | None
    if private_resource_spec is None:
        kinecal_config_path = "config/kinecal_config.toml"
        kinecal_default_config_path = "config/kinecal_config_default.toml"
        kinecal_probe_params_path = "config/probe_params.toml"
    else:
        kinecal_config_path = getattr(robot_interface, "KINECAL_CONFIG_PATH", None)
        kinecal_default_config_path = getattr(
            robot_interface, "KINECAL_DEFAULT_CONFIG_PATH", None
        )
        kinecal_probe_params_path = getattr(
            robot_interface, "KINECAL_PROBE_PARAMS_PATH", None
        )

    with ExitStack() as resource_stack:
        default_sim_urdf = resource_stack.enter_context(as_file(packaged_urdf))
        default_kinecal_config_path = _materialize_optional_resource(
            resource_stack, packaged_resources, kinecal_config_path
        )
        default_kinecal_restore_config_path = _materialize_optional_resource(
            resource_stack, packaged_resources, kinecal_default_config_path
        )
        kinecal_probe_parameters_path = _materialize_optional_resource(
            resource_stack, packaged_resources, kinecal_probe_params_path
        )
        return run_helpers.main(
            robot_interface_class=robot_interface.RobotInterface,
            simulator_configuration=run_helpers.SimulatorConfiguration(
                urdf_path=default_sim_urdf,
                name="My Robot",
                sample_frequency_hz=robot_max_frequency_hz,
                imu_record_frequency_hz=imu_record_frequency_hz,
                data_folder_prefix=simulator_data_location_prefix,
                servo_bandwidth_hz=robot_interface.MAX_ROBOT_JOINTS_BANDWIDTH,
                calibration_start_joints=robot_interface.FULL_STRETCH_JOINTS,
                calibration_start_quat=robot_interface.FULL_STRETCH_QUAT,
                calibration_start_xyz=robot_interface.FULL_STRETCH_XYZ,
                full_stretch_pose_override=(robot_interface.FULL_STRETCH_POSE_OVERRIDE),
            ),
            default_robot_id=robot_interface.BOT_ID,
            argv=argv,
            script_path=Path(__file__).resolve(),
            default_kinecal_config_path=default_kinecal_config_path,
            default_kinecal_restore_config_path=default_kinecal_restore_config_path,
            kinecal_probe_parameters_path=kinecal_probe_parameters_path,
            kinecal_output_root=(
                Path(data_location_prefix).expanduser() / "kinecal" / "datacol"
            ),
            kinecal_taught_tcps_cache_path=(
                Path(data_location_prefix).expanduser()
                / "kinecal"
                / "recent_taught_tcps.json"
            ),
            kinecal_source_urdf_path=default_sim_urdf,
        )


if __name__ == "__main__":
    main()
