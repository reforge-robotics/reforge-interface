# reforge-interface

> [!WARNING]
> The DENSO COBOTTA PRO integration has not yet been validated on physical hardware.

Robot interface code for integrating external robot SDKs with Reforge calibration and identification workflows.

## Overview

`reforge-interface` provides a robot adapter layer that lets you:

- connect your robot SDK to a consistent interface,
- run calibration data collection,
- run identification / fine-tuning against Reforge Cloud,
- and run vibration tests for controller evaluation.

The robot package is defined and built from `src/robot/`. The repository root contains deployment scripts, skill templates, docs, and UI support files.

## Repository Layout

```text
reforge-interface/
├── README.md
├── docker_scripts/              # wrappers for running the Docker image
├── docs/                        # supporting docs
├── skill-templates/             # agent skill templates
├── ui-widget/                   # web widget support
└── src/
    └── robot/
        ├── Dockerfile
        ├── pyproject.toml       # robot package metadata
        ├── requirements.txt     # robot runtime dependencies
        ├── run.py               # CLI entrypoint
        ├── robot_interface.py   # SDK integration target
        ├── robot_base.py        # abstract base + defaults
        ├── ros_manager.py       # optional ROS trajectory publisher
        ├── urdf/                # place robot URDF files here
        └── data/                # calibration outputs
```

## Quick Start

1. Add your robot URDF to `src/robot/urdf/`.
2. Add your robot SDK dependency to `src/robot/requirements.txt` or make it importable in the environment.
3. Integrate the SDK in `src/robot/robot_interface.py` by replacing all `# {~.~}` markers.
4. Validate the integration.

```bash
python3 -m py_compile src/robot/robot_interface.py
rg "\{~\.~\}" src/robot/robot_interface.py
```

5. Install the robot package from `src/robot`.

```bash
python3 -m venv .venv
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash # If your robot integration needs ROS.
pip install -r src/robot/requirements.txt
pip install -e src/robot
```

Or build the Docker image from the robot package directory:

```bash
docker build -t reforge-interface:latest src/robot
```

## 5-Minute Happy Path

Use this when you want a fast first calibration run with minimal options.

```bash
# 1) Add your robot URDF.
cp /path/to/my_robot.urdf src/robot/urdf/my_robot.urdf

# 2) Integrate SDK in src/robot/robot_interface.py.
#    - set URDF_PATH="urdf/my_robot.urdf"
#    - replace all # {~.~} markers

# 3) Validate interface wiring.
python3 -m py_compile src/robot/robot_interface.py
rg "\{~\.~\}" src/robot/robot_interface.py

# 4) Install.
python3 -m venv .venv
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash # If your robot integration needs ROS.
pip install -r src/robot/requirements.txt
pip install -e src/robot

# 5) Test connection without trajectory execution.
python3 -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>

# 6) Run calibration.
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200
```

Docker wrappers are available for robot control boxes:

```bash
docker build -t reforge-interface:latest src/robot

chmod +x docker_scripts/run_connect_test.sh
./docker_scripts/run_connect_test.sh <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>

chmod +x docker_scripts/run_calibrate.sh
./docker_scripts/run_calibrate.sh <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200
```

## Robot Integration Flow

1. Set constants in `src/robot/robot_interface.py`:
- `BOT_ID`, `URDF_PATH`, `ROBOT_MAX_FREQ`
- `HOME_SHOULDER_ANGLE`, `HOME_XYZ`, `HOME_QUAT`, `HOME_JOINTS`
- `IS_DEGREES`, `DATA_LOCATION_PREFIX`

2. Add SDK client setup in `RobotInterface.__init__`.
3. Implement required SDK-bound methods:
- `__get_joint_positions`
- `__get_tcp_pose`
- `move_to_joint`
- `move_to_pose`
- `publish_and_record_joint_positions`

4. Keep units and schemas consistent:
- internal joint units in radians,
- pose format `[x, y, z, qx, qy, qz, qw]`,
- data log keys required by calibration pipeline.

See `src/robot/README.md` for the detailed contract.

## CLI Usage

Use module invocation after editable install:

```bash
python3 -m robot.run --help
```

The package also exposes a console script:

```bash
reforge-robot --help
```

### Connection test

```bash
python3 -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>
./docker_scripts/run_connect_test.sh <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>
```

### Calibration

```bash
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200
./docker_scripts/run_calibrate.sh <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200
```

Optional cloud actions immediately after calibration:

```bash
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200 --identify <api_token> --reforge_robot_id <reforge_robot_id>
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200 --fine_tune <api_token> --reforge_robot_id <reforge_robot_id>
```

### Identification and fine-tuning

```bash
python3 -m robot.run identify <api_token> <reforge_robot_id> <data_folder>
./docker_scripts/run_identify.sh <api_token> <reforge_robot_id> <data_folder>

python3 -m robot.run fine_tune <api_token> <reforge_robot_id> <data_folder>
./docker_scripts/run_fine_tune.sh <api_token> <reforge_robot_id> <data_folder>
```

### Vibration test

```bash
python3 -m robot.run vibration_test <robot_ip> <data_folder> --robot_id <robot_id>
./docker_scripts/run_vibration_test.sh <robot_ip> <data_folder> --robot_id <robot_id>
```

## Validation Checklist

- `python3 -m py_compile src/robot/robot_interface.py` passes.
- `rg "\{~\.~\}" src/robot/robot_interface.py` returns no matches.
- `URDF_PATH` points to a real file under `src/robot/urdf/`.
- Joint count returned by SDK is compatible with URDF model count.
- Calibration starts without unit, schema, or connection errors.

## Troubleshooting

- SDK import fails:
  - confirm package is installable and import path matches SDK docs.
- Robot does not move:
  - verify control mode, motor enable/unbrake, and any SDK session requirements.
- Motion appears wrong:
  - verify `IS_DEGREES` and conversion boundaries.
- Missing telemetry fields:
  - preserve required output keys with empty values instead of removing keys.

## Additional Documentation

- Robot integration details: `src/robot/README.md`
- Skill-based integration guide: `docs/robot-sdk-skill-user-guide.md`
