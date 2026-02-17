# src/robots/run.py
# Author: Reforge Robotics (Nosa Edoimioya)
# Description: Entry point for running various operations on the robot
# Version: 1.0
import argparse
import traceback
from robot.robot_shell import (
    DEFAULT_MAX_DISP,
    DEFAULT_MAX_VEL,
    DEFAULT_MAX_ACC,
    PLACEHOLDER_IP,
    DEFAULT_SYSID_ANGLES,
    DEFAULT_SYSID_RADII,
    DEFAULT_HOME_SIGN,
    DEFAULT_FIRST_POSE,
    DEFAULT_ROBOT_FREQ,
    DEFAULT_AXES_COMMANDED,
)
from util.Utility import (
    DEFAULT_SINE_MIN_FREQ,
    DEFAULT_SINE_MAX_FREQ,
    DEFAULT_FREQ_SPACING,
    DEFAULT_SINE_CYCLES,
    DEFAULT_DWELL_TIME,
)
from robot.robot_interface import RobotInterface, BOT_ID
from calibration.api import ReforgeAPIManager

from unit_tests.vibration_test import run_vibration_test


def _build_parser() -> argparse.ArgumentParser:
    """Create the command-line argument parser for robot operations.

    Args:
        None.

    Returns:
        `argparse.ArgumentParser` configured with subcommands and options.

    Side Effects:
        None.

    Raises:
        None.

    Preconditions:
        None.
    """
    parser = argparse.ArgumentParser(
        description="Robot CLI for connecting to robot, calibration, system identification, vibration test, or fine-tuning."
    )
    sub = parser.add_subparsers(
        dest="route",
        help="Available commands: 'connect_test', 'calibrate', 'identify', 'vibration_test', 'fine_tune'",
        required=True,
    )

    # ======================== Route: connect_test ==================================
    connect_test = sub.add_parser(
        "connect_test", help="Test connection to the robot without motion."
    )
    connect_test.add_argument("robot_ip", help="Robot IP address")
    connect_test.add_argument(
        "--local_ip",
        help="Local machine IP address, if necessary",
        default=PLACEHOLDER_IP,
    )
    connect_test.add_argument(
        "--sdk_token",
        help="API token to use robot's SDK, if necessary",
        default=PLACEHOLDER_IP,
    )
    connect_test.add_argument(
        "--robot_id", help="Reforge robot ID, if necessary", default=""
    )

    # ======================== Route: calibrate =====================================
    calibrate = sub.add_parser("calibrate", help="Run robot calibration routine.")
    calibrate.add_argument("robot_ip", help="Robot IP address")
    calibrate.add_argument(
        "--local_ip",
        dest="local_ip",
        type=str,
        default=PLACEHOLDER_IP,
        help="Local machine IP address, if necessary",
    )
    calibrate.add_argument(
        "--sdk_token",
        dest="sdk_token",
        type=str,
        default=PLACEHOLDER_IP,
        help="API token to use robot's SDK, if necessary",
    )
    calibrate.add_argument(
        "--robot_id",
        dest="robot_id",
        type=str,
        default=BOT_ID,
        help="Reforge robot ID, if necessary",
    )
    # timing parameters
    calibrate.add_argument(
        "--freq",
        dest="samp_freq",
        type=float,
        default=DEFAULT_ROBOT_FREQ,
        help="Sampling frequency (>200 Hz, default: %(default)s)",
    )
    # trajectory limits (None means use dynamic default)
    calibrate.add_argument(
        "--mdisp",
        dest="max_disp",
        type=float,
        default=DEFAULT_MAX_DISP,
        help="Max displacement for trajectory (default: %(default)s rad)",
    )
    calibrate.add_argument(
        "--mvel",
        dest="max_vel",
        type=float,
        default=DEFAULT_MAX_VEL,
        help="Max velocity for trajectory (default: %(default)s rad/s)",
    )
    calibrate.add_argument(
        "--macc",
        dest="max_acc",
        type=float,
        default=DEFAULT_MAX_ACC,
        help="Max acceleration for trajectory (default: %(default)s rad/s^2) ",
    )
    # scan parameters
    calibrate.add_argument(
        "--nv",
        dest="num_angles",
        type=int,
        default=DEFAULT_SYSID_ANGLES,  # this is V
        help="Number of angles to sweep (default: %(default)s)",
    )
    calibrate.add_argument(
        "--nr",
        dest="num_radii",
        type=int,
        default=DEFAULT_SYSID_RADII,  # this is R
        help="Number of radii to sweep (default: %(default)s)",
    )
    calibrate.add_argument(
        "--minfreq",
        dest="min_freq",
        type=float,
        default=DEFAULT_SINE_MIN_FREQ,
        help=f"Min frequency [Hz] (default: {DEFAULT_SINE_MIN_FREQ})",
    )
    calibrate.add_argument(
        "--maxfreq",
        dest="max_freq",
        type=float,
        default=DEFAULT_SINE_MAX_FREQ,
        help=f"Max frequency [Hz] (default: {DEFAULT_SINE_MAX_FREQ})",
    )
    calibrate.add_argument(
        "--freqspace",
        dest="freq_space",
        type=float,
        default=DEFAULT_FREQ_SPACING,
        help=f"Frequency spacing [Hz] (default: {DEFAULT_FREQ_SPACING})",
    )
    calibrate.add_argument(
        "--sine_cycles",
        dest="sine_cycles",
        type=int,
        default=DEFAULT_SINE_CYCLES,
        help=f"Number of sine cycles per pose (default: {DEFAULT_SINE_CYCLES})",
    )
    calibrate.add_argument(
        "--dwell",
        dest="dwell",
        type=float,
        default=DEFAULT_DWELL_TIME,
        help=f"Post-sweep dwell [s] (default: {DEFAULT_DWELL_TIME})",
    )
    calibrate.add_argument(
        "--axes",
        dest="num_axes",
        type=int,
        default=DEFAULT_AXES_COMMANDED,
        help="Number of axes to command (default: %(default)s)",
    )
    calibrate.add_argument(
        "--home",
        dest="home_sign",
        type=int,
        choices=[-1, 1],
        default=DEFAULT_HOME_SIGN,
        help="Shoulder home direction -1 or 1 (default: %(default)s)",
    )
    calibrate.add_argument(
        "--first_pose",
        dest="start_pose",
        type=int,
        default=DEFAULT_FIRST_POSE,
        help="Index of the first pose (default: %(default)s)",
    )
    # automatic identification/fine-tuning after calibration
    calibrate.add_argument(
        "--identify",
        dest="identify_api_token",
        type=str,
        help="API token for HTTP request to Reforge Cloud to run system identification after calibration",
    )
    calibrate.add_argument(
        "--fine_tune",
        dest="fine_tune_api_token",
        type=str,
        help="API token for HTTP request to Reforge Cloud to run model fine-tuning after calibration",
    )

    # ======================== Route: identify =====================================
    identify = sub.add_parser("identify", help="Run system identification routine.")
    identify.add_argument(
        "identify_api_token", help="API token for HTTP request to Reforge Cloud."
    )
    identify.add_argument("robot_id", help="Reforge robot ID.")
    identify.add_argument(
        "data_folder", help="Folder containing calibration data CSVs."
    )

    # ======================== Route: vibration_test =====================================
    vibration_test = sub.add_parser(
        "vibration_test", help="Run vibration test to quantify controller performance."
    )
    vibration_test.add_argument("robot_ip", help="Robot IP address")
    vibration_test.add_argument(
        "data_folder",
        help="Folder containing most recent calibration data CSVs. \
                                    Random poses from the calibration data are used for the vibration test.",
    )
    vibration_test.add_argument(
        "--local_ip",
        dest="local_ip",
        type=str,
        default=PLACEHOLDER_IP,
        help="Local machine IP address, if necessary",
    )
    vibration_test.add_argument(
        "--sdk_token",
        dest="sdk_token",
        type=str,
        default=PLACEHOLDER_IP,
        help="API token to use robot's SDK, if necessary",
    )
    # timing parameters
    vibration_test.add_argument(
        "--freq",
        dest="samp_freq",
        type=float,
        default=DEFAULT_ROBOT_FREQ,
        help="Sampling frequency (>200 Hz, default: %(default)s)",
    )
    # trajectory limits (None means use dynamic default)
    vibration_test.add_argument(
        "--mdisp",
        dest="max_disp",
        type=float,
        default=DEFAULT_MAX_DISP,
        help="Max displacement for trajectory (default: %(default)s rad)",
    )
    vibration_test.add_argument(
        "--mvel",
        dest="max_vel",
        type=float,
        default=DEFAULT_MAX_VEL,
        help="Max velocity for trajectory (default: %(default)s rad/s)",
    )
    vibration_test.add_argument(
        "--macc",
        dest="max_acc",
        type=float,
        default=DEFAULT_MAX_ACC,
        help="Max acceleration for trajectory (default: %(default)s rad/s^2) ",
    )

    # ======================== Route: fine_tune =====================================
    fine_tune = sub.add_parser(
        "fine_tune",
        help="Manually run fine-tuning on the robot model with the given data folder.",
    )
    fine_tune.add_argument(
        "fine_tune_api_token", help="API token for HTTP request to Reforge Cloud."
    )
    fine_tune.add_argument("robot_id", help="Reforge robot ID.")
    fine_tune.add_argument(
        "data_folder", help="Folder containing calibration data CSVs."
    )

    return parser


def route_user_input(args: argparse.Namespace) -> None:
    """Route command-line arguments to the appropriate operation.

    Args:
        args: Parsed command-line arguments.

    Returns:
        `None`.

    Side Effects:
        May connect to hardware, run calibration, or write model files.

    Raises:
        None.

    Preconditions:
        `args.route` is a valid subcommand.
    """
    if args.route == "connect_test":
        try:
            robot_interface = RobotInterface(
                robot_ip=args.robot_ip,
                local_ip=args.local_ip,
                sdk_token=args.sdk_token,
                robot_id=args.robot_id,
            )
            print(
                f"✅ Successfully connected to robot {robot_interface.name} at IP address {args.robot_ip}."
            )
        except Exception as e:
            print(
                f"❌ Failed to connect to robot at IP address {args.robot_ip}: {str(e)}"
            )
    elif args.route == "calibrate":
        try:
            robot_interface = RobotInterface(
                robot_ip=args.robot_ip,
                local_ip=args.local_ip,
                sdk_token=args.sdk_token,
                robot_id=args.robot_id,
            )

            data_folder = robot_interface.calibrate_robot(
                Ts=1 / args.samp_freq,
                axes_to_command=(
                    args.num_axes if args.num_axes else robot_interface.num_joints
                ),
                max_disp=args.max_disp,
                max_vel=args.max_vel,
                max_acc=args.max_acc,
                nV=args.num_angles,
                nR=args.num_radii,
                min_sine_freq=args.min_freq,
                max_sine_freq=args.max_freq,
                sine_freq_spacing=args.freq_space,
                num_sine_cycles=args.sine_cycles,
                dwell_btw_sine=args.dwell,
                start_pose=args.start_pose,
                home_sign=args.home_sign,
            )

            # Optional: Call system identification or fine-tuning after calibration
            if args.fine_tune_api_token:
                api_manager = ReforgeAPIManager(
                    reforge_api_token=args.fine_tune_api_token, robot_id=args.robot_id
                )
                # TO-DO: Edit the run_fine_tuning function
                return api_manager.run_fine_tuning(data_folder=data_folder)
                pass
            elif args.identify_api_token:
                api_manager = ReforgeAPIManager(
                    reforge_api_token=args.identify_api_token, robot_id=args.robot_id
                )
                # TO-DO: Edit the run_identification function
                return api_manager.run_identification(data_folder=data_folder)
            return
        except Exception:
            traceback.print_exc()
    elif args.route == "identify":
        api_manager = ReforgeAPIManager(
            reforge_api_token=args.identify_api_token, robot_id=args.robot_id
        )
        # TO-DO: Edit the run_identification function
        return api_manager.run_identification(data_folder=args.data_folder)
    elif args.route == "vibration_test":
        try:
            robot_interface = RobotInterface(
                robot_ip=args.robot_ip,
                local_ip=args.local_ip,
                sdk_token=args.sdk_token,
                robot_id=args.robot_id,
            )

            return run_vibration_test(
                robot_interface=robot_interface,
                local_data_location=args.data_folder,
                Ts=1 / args.samp_freq,
                max_disp=args.max_disp,
            )

        except Exception:
            traceback.print_exc()
    elif args.route == "fine_tune":
        api_manager = ReforgeAPIManager(
            reforge_api_token=args.fine_tune_api_token, robot_id=args.robot_id
        )
        return api_manager.run_fine_tuning(data_folder=args.data_folder)
        pass
    else:
        raise ValueError("Invalid route specified.")


def main() -> None:
    """Entry point for robot command-line operations.

    Returns:
        `None`.

    Side Effects:
        Parses command-line arguments and dispatches actions.

    Raises:
        SystemExit: If argument parsing fails.

    Preconditions:
        None.
    """
    parser = _build_parser()
    args = parser.parse_args()

    route_user_input(args=args)


if __name__ == "__main__":
    main()
