# Robot Integration Guide

This folder contains the robot adapter layer used by Reforge calibration and identification pipelines.

## Purpose

Implement one concrete robot adapter in `robot_interface.py` that satisfies the interface contract defined in `robot_base.py`.

The calibration flow expects strict method signatures, units, and logged data schema.

## Key Files

- `src/robot/robot_interface.py`: SDK-specific implementation target.
- `src/robot/robot_base.py`: Abstract robot interface, defaults, and shared utilities.
- `src/robot/run.py`: CLI routes (`connect_test`, `calibrate`, `identify`, `fine_tune`, `vibration_test`).
- `src/robot/ros_manager.py`: Optional ROS-based trajectory publishing helper.
- `src/robot/urdf/`: Robot URDF files.
- `src/robot/data/`: Output data from calibration runs.

## Integration Contract

### Required methods in `RobotInterface`

- `__get_joint_positions(self) -> List`
- `__get_tcp_pose(self) -> List`
- `move_to_joint(self, target_joint: Tuple[float, ...]) -> None`
- `move_to_pose(self, target_quat: List[float], target_xyz: List[float]) -> None`
- `publish_and_record_joint_positions(...) -> Deque[Dict]`

### Data log schema

`publish_and_record_joint_positions` must return `data_log` rows with keys:

- `cmd_time`
- `input_positions`
- `output_positions`
- `velocities`
- `efforts`
- `imu_time`
- `linear_acceleration`
- `angular_velocity`
- `orientation`

If a field is unavailable, keep the key and set an empty value.

## How To Integrate a New SDK

1. Add URDF file to `src/robot/urdf/`.
2. Add SDK dependency in `requirements.txt` (or make local package importable).
3. Open `src/robot/robot_interface.py` and replace all `# {~.~}` sections.
4. Set robot constants:
- `BOT_ID`, `URDF_PATH`, `ROBOT_MAX_FREQ`
- `HOME_SHOULDER_ANGLE`, `HOME_XYZ`, `HOME_QUAT`, `HOME_JOINTS`, `HOME_POSE_OVERRIDE`
- `IS_DEGREES`, `DATA_LOCATION_PREFIX`
5. Implement SDK client setup in `RobotInterface.__init__`.
6. Implement all required methods with SDK calls.
7. Validate and run connection test before full calibration.

## Units and Frames

- Internal joint representation should be radians.
- Use `IS_DEGREES=True` only when SDK APIs are degree-based; convert at SDK boundaries.
- TCP pose format is `[x, y, z, qx, qy, qz, qw]`.
- Payload settings:
  - `--tcp_payload` uses same units as URDF inertias.
  - `--tcp_payload_com_x/y/z` are in meters, TCP frame.

## Commands

### 5-Minute Adapter Happy Path

```bash
# From repository root
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

### Run identification / fine-tune manually

```bash
python -m robot.run identify <identify_api_token> <robot_id> <data_folder>
python -m robot.run fine_tune <fine_tune_api_token> <robot_id> <data_folder>
```

## Common Pitfalls

1. Joint count mismatch
- Hardware may expose extra joints (for example grippers).
- Truncate extras to URDF joint count, but fail when fewer than expected joints are returned.

2. Degree/radian mismatch
- Incorrect conversion causes wrong motion and unstable trajectories.

3. Missing control-state setup
- Some SDKs require explicit teleop/ROS enable, motor enable, or unbrake before motion commands.

4. Schema drift in recorded data
- Do not rename or remove required `data_log` keys.

## Developer Notes

- Avoid modifying pre-defined calibration workflow methods unless integration requires it.
- Keep changes focused to SDK-specific placeholders and required method implementations.
