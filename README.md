# Reforge Interface for UFACTORY xArm

This branch provides the UFACTORY UF850 adapter for Reforge calibration,
identification, KineCal, Shaper, and Joint Tracker workflows. It targets
`reforge-core>=2.0.15,<3` and controls the robot through the UFACTORY xArm
Python SDK.

## Repository layout

```text
reforge-interface/
├── Dockerfile
├── pyproject.toml
├── requirements.txt
└── src/robot/
    ├── config/                 # KineCal configuration
    ├── example_usage/          # Public product examples and example models
    ├── meshes/uf850/           # UF850 visual and collision meshes
    ├── urdf/uf850.urdf         # Robot model
    ├── robot_interface.py      # UFACTORY hardware adapter
    ├── ros_manager.py          # Optional ROS 2 trajectory helper
    └── run.py                  # Reforge CLI entrypoint
```

Runtime calibration data and generated calibration reports are intentionally
not included in the repository.

## Installation

Python 3.11 or newer is required.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -e .
```

The editable install also provides the `reforge-robot` command.

## Connection smoke test

Release the emergency stop and follow all normal UFACTORY safety procedures
before connecting. The connection test initializes the SDK but does not execute
a calibration trajectory.

```bash
python -m robot.run connect_test <robot_ip> --robot_id <robot_id>
```

Equivalent installed command:

```bash
reforge-robot connect_test <robot_ip> --robot_id <robot_id>
```

## Common workflows

Inspect the current CLI before running hardware operations:

```bash
python -m robot.run --help
python -m robot.run calibrate --help
```

Run system identification calibration:

```bash
python -m robot.run calibrate <robot_ip> \
  --robot_id <robot_id> \
  --freq 200
```

Run interactive KineCal collection:

```bash
python -m robot.run kinecal <robot_ip>
```

The editable KineCal configuration is
`src/robot/config/kinecal_config.toml`. The adjacent default file can be used
to restore the baseline configuration.

Run model identification or fine-tuning on an existing data folder:

```bash
python -m robot.run identify <api_token> <robot_id> <data_folder>
python -m robot.run fine_tune <api_token> <robot_id> <data_folder>
```

Never commit API tokens, robot credentials, or generated calibration data.

## Docker

```bash
docker build -t reforge-interface:latest .
./docker_scripts/run_connect_test.sh <robot_ip> --robot_id <robot_id>
```

The scripts under `docker_scripts/` mount calibration data from the host so it
is not stored in the image.

## Adapter contract

`src/robot/robot_interface.py` implements the abstract methods required by
`reforge_core.hw_interfaces.arm_client.ArmClient`:

- `command_move_j`
- `command_move_pose`
- `command_servo_j`
- `enter_position_mode`
- `enter_servo_mode`
- `get_joint_state`
- `get_tcp_pose`
- the `in_sim_mode` and `urdf_path` properties

Joint positions and velocities cross the Reforge boundary in radians. TCP
positions use meters, and TCP orientations use `[qx, qy, qz, qw]` quaternions.

See `src/robot/README.md` for integration details and `python -m robot.run
<command> --help` for the authoritative command options.

## Development checks

```bash
python -m compileall -q src/robot
python -m pytest -q
```

Hardware motion cannot be validated in CI. Perform the connection smoke test
before any trajectory-producing command, then validate motion on a guarded
robot at reduced speed.
