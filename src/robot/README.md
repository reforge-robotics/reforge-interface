# Robot Integration Guide

This folder contains the robot adapter layer used by Reforge calibration and identification pipelines.

## Purpose

Implement one concrete robot adapter in `robot_interface.py` that satisfies
`reforge_core.hw_interfaces.arm_client.ArmClient`.

The calibration flow expects strict method signatures, units, and telemetry
dimensions. `reforge-core` owns the recorded data schema.

## Key Files

- `pyproject.toml`: Robot package metadata.
- `requirements.txt`: Robot runtime dependencies and SDK additions.
- `Dockerfile`: Container build for the robot package.
- `robot_interface.py`: SDK-specific implementation target.
- `run.py`: CLI routes (`connect_test`, `calibrate`, `identify`, `fine_tune`, `vibration_test`).
- `ros_manager.py`: Optional ROS 2 adapter for robots that require ROS control.
- `urdf/`: Robot URDF files.
- `config/`: Robot configuration files, including the editable kinecal config and default restore template.
- `data/`: Output data from calibration runs.

## Integration Contract

### Required methods in `RobotInterface`

- `command_move_j(...) -> int`
- `command_move_pose(...) -> int`
- `command_servo_j(...) -> int`
- `enter_position_mode()`
- `enter_servo_mode()`
- `get_joint_state() -> tuple[list[float], list[float], list[float]]`
- `get_tcp_pose() -> list[float]`

`get_joint_state` returns positions, velocities, and efforts as `(q, qd, tau)`.
Prefer one atomic SDK state snapshot when available.

Trajectory execution, arm/IMU timestamp alignment, and calibration data-log
construction are handled by `reforge-core`, not by the SDK adapter.

## How To Integrate a New SDK

1. Add URDF file to `urdf/`.
2. Add SDK dependency in `requirements.txt` (or make local package importable).
3. Open `robot_interface.py` and replace all `# {~.~}` sections.
4. Set robot constants:
- `BOT_ID`, `URDF_PATH`, `ROBOT_MAX_FREQ`
- `FULL_STRETCH_XYZ`, `FULL_STRETCH_QUAT`, `FULL_STRETCH_JOINTS`
- `FULL_STRETCH_POSE_OVERRIDE`
- `IS_DEGREES`, `DATA_LOCATION_PREFIX`, `DEFAULT_TCP_PAYLOAD`
5. Implement SDK client setup in `RobotInterface.__init__`.
6. Implement all required methods with SDK calls.
7. Keep `USE_REFORGE_IMU=True` for the standard Reforge USB IMU path.
8. Validate and run a connection test before motion or full calibration.

## Units and Frames

- Internal joint representation should be radians.
- Use `IS_DEGREES=True` only when SDK APIs are degree-based; convert at SDK boundaries.
- TCP pose format is `[x, y, z, qx, qy, qz, qw]`.
- Payload and IMU settings:
  - `--tcp_payload` uses same units as URDF inertias.
  - `--tcp_payload_com_x/y/z` are in meters, TCP frame.
  - `--imu_to_tcp_x/y/z` are the measured IMU-to-TCP translation in meters,
    resolved in the IMU frame.

## Reforge IMU

The default interface initializes the Reforge IMU through
`ArmClient.initialize_arm_imu_manager()`. Do not read a vendor IMU or combine
IMU and arm samples in `RobotInterface`.

Only override `create_robot_imu_recorder()` when explicitly supporting a
vendor-native IMU with `use_reforge_imu=False`.

## Commands

### 5-Minute Adapter Happy Path

```bash
# From repository root.
pip install -r src/robot/requirements.txt
pip install -e src/robot

# Or from this folder.
pip install -r requirements.txt
pip install -e .

# 1) Place URDF
cp /path/to/my_robot.urdf src/robot/urdf/my_robot.urdf

# 2) Implement adapter in src/robot/robot_interface.py
#    - set URDF_PATH="urdf/my_robot.urdf"
#    - replace all # {~.~} sections

# 3) Validate
python -m py_compile src/robot/robot_interface.py
rg "\{~\.~\}" src/robot/robot_interface.py

# 4) Connection smoke test
python -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>

# 5) Calibration run
python -m robot.run calibrate <robot_ip> --robot_id <robot_id> --freq 200
```

### Build Docker image

From the repository root:

```bash
docker build -t reforge-interface:latest src/robot
```

From this folder:

```bash
docker build -t reforge-interface:latest .
```

### Validate adapter

```bash
python -m py_compile src/robot/robot_interface.py
rg "\{~\.~\}" src/robot/robot_interface.py
```

### Test connection (no calibration trajectory)

```bash
python -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>
```

### Run calibration

```bash
python -m robot.run calibrate <robot_ip> --robot_id <robot_id> --freq 200
```

### Run kinecal data collection

```bash
python -m robot.run kinecal <robot_ip>
```

By default, this uses `src/robot/config/kinecal_config.toml`. To restore defaults, copy `src/robot/config/kinecal_config_default.toml` over that file.

### Run identification / fine-tune manually

```bash
python -m robot.run identify <identify_api_token> <robot_id> <data_folder>
python -m robot.run fine_tune <fine_tune_api_token> <robot_id> <data_folder>
```

## Common Pitfalls

1. Joint count mismatch
- Select arm-only SDK telemetry when the URDF models only the arm.
- The interface initialization requires the SDK and URDF actuated joint counts
  to match exactly.

2. Degree/radian mismatch
- Incorrect conversion causes wrong motion and unstable trajectories.

3. Missing control-state setup
- Some SDKs require explicit teleop/ROS enable, motor enable, or unbrake before motion commands.

4. Reforge IMU not detected
- Confirm USB access and `DEFAULT_IMU_COMM_MODE`.
- Confirm IMU recording frequency does not exceed the supported arm sampling
  frequency.

## Developer Notes

- Avoid modifying pre-defined calibration workflow methods unless integration requires it.
- Keep changes focused to SDK-specific placeholders and required method implementations.
