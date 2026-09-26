"""Compatibility entrypoint for robot command-line operations."""

from __future__ import annotations

import sys
from collections.abc import Sequence
from contextlib import ExitStack
from importlib.resources import as_file, files
from importlib.resources.abc import Traversable
from pathlib import Path
from tempfile import TemporaryDirectory

from reforge_core.calibration import run_helpers
import robot.robot_interface as robot_interface
from robot import split_urdf


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


def _ensure_bimanual_urdfs(
    source: Path, selected_output: Path, *, use_left: bool
) -> None:
    """Generate missing per-arm URDFs before calibration opens the selected arm.

    The selected output comes from ``robot_interface.URDF_PATH``. Passing it to
    the splitter explicitly keeps the generated model and calibration model in
    sync even when an integration uses a nonstandard output filename.
    """
    source = source.resolve()
    selected_output = selected_output.resolve()
    other_side = "right" if use_left else "left"
    other_output = source.with_name(f"{source.stem}-{other_side}{source.suffix}")
    left_output, right_output = (
        (selected_output, other_output)
        if use_left
        else (other_output, selected_output)
    )
    if left_output == right_output or source in {left_output, right_output}:
        raise ValueError("Bimanual source, left output, and right output must differ.")

    left_exists = left_output.is_file()
    right_exists = right_output.is_file()
    if left_exists and right_exists:
        return
    if not source.is_file():
        raise FileNotFoundError(f"Bimanual source URDF not found: {source}")

    # The splitter writes both arms. Redirect an existing arm to a disposable
    # path so a partial split can be completed without replacing that file.
    with TemporaryDirectory(prefix="robot-split-") as temp_dir:
        temporary = Path(temp_dir)
        split_left = temporary / left_output.name if left_exists else left_output
        split_right = temporary / right_output.name if right_exists else right_output
        try:
            status = split_urdf.main(
                [
                    "split",
                    str(source),
                    "--left-output",
                    str(split_left),
                    "--right-output",
                    str(split_right),
                ]
            )
        except split_urdf.SplitError as exc:
            raise SystemExit(f"error: {exc}") from exc
        if status != 0:
            raise RuntimeError(f"Bimanual URDF split failed with status {status}.")
    if not left_output.is_file() or not right_output.is_file():
        raise RuntimeError("Bimanual URDF split did not create both arm models.")


def main(argv: Sequence[str] | None = None) -> None:
    """Parse robot CLI arguments and dispatch to the SDK-owned implementation.

    May connect to robot hardware, run calibration, or write model files.

    Args:
        argv: Optional command-line arguments. Uses `sys.argv` when omitted.

    Raises:
        SystemExit: If argument parsing fails.
    """
    # The calibration routes never close the interface they open, so track
    # every live robot and close it on the way out (normal return, error, or
    # Ctrl-C). Closing returns the arm to rest before disabling it.
    opened: list[robot_interface.RobotInterface] = []

    def open_robot_interface(**kwargs) -> robot_interface.RobotInterface:
        opened_interface = robot_interface.RobotInterface(**kwargs)
        opened.append(opened_interface)
        return opened_interface

    cli_args = list(sys.argv[1:] if argv is None else argv)
    bimanual = bool(cli_args and cli_args[0] == "calibrate" and "--bimanual" in cli_args)
    if bimanual:
        cli_args = [argument for argument in cli_args if argument != "--bimanual"]
        # Validate the SDK options before asking the operator to split a URDF.
        # argparse exits here for --help, so help needs no generated URDF.
        run_helpers.build_parser(
            default_robot_id=robot_interface.BOT_ID
        ).parse_args(cli_args)

    packaged_resources = files("robot")
    urdf_path = robot_interface.URDF_PATH
    if bimanual:
        package_dir = Path(__file__).resolve().parent
        _ensure_bimanual_urdfs(
            package_dir / robot_interface.BASE_URDF_PATH,
            package_dir / urdf_path,
            use_left=robot_interface.USE_LEFT,
        )
    packaged_urdf = packaged_resources.joinpath(urdf_path)
    if not packaged_urdf.is_file():
        raise FileNotFoundError(f"Robot URDF resource not found: {urdf_path}")

    try:
        with as_file(packaged_urdf) as default_sim_urdf:
            return run_helpers.main(
                robot_interface_class=open_robot_interface,
                simulator_configuration=run_helpers.SimulatorConfiguration(
                    urdf_path=default_sim_urdf,
                    name="My Robot",
                    sample_frequency_hz=robot_interface.ROBOT_MAX_FREQ,
                    imu_record_frequency_hz=robot_interface.DEFAULT_IMU_RECORD_FREQUENCY_HZ,
                    data_folder_prefix=robot_interface.SIM_DATA_LOCATION_PREFIX,
                    servo_bandwidth_hz=robot_interface.MAX_ROBOT_JOINTS_BANDWIDTH,
                    calibration_start_joints=robot_interface.FULL_STRETCH_JOINTS,
                    calibration_start_quat=robot_interface.FULL_STRETCH_QUAT,
                    calibration_start_xyz=robot_interface.FULL_STRETCH_XYZ,
                    full_stretch_pose_override=robot_interface.FULL_STRETCH_POSE_OVERRIDE,
                ),
                default_robot_id=robot_interface.BOT_ID,
                argv=cli_args,
                script_path=Path(__file__).resolve(),
            )
    finally:
        for opened_interface in reversed(opened):
            opened_interface.close()

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
