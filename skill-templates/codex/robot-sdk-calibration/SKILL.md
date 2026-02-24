---
name: robot-sdk-calibration
description: Integrate a robot SDK into reforge-interface for calibration and system identification. Use when a user provides a robot SDK package and URDF and needs src/robot/robot_interface.py wired to their hardware API, including required methods, unit handling, and data logging shape compatibility.
---

# Robot SDK Calibration Integration (Codex)

Implement robot SDK integration in `src/robot/robot_interface.py` for calibration workflows.

## Inputs required

- Robot SDK location (pip package name, git URL, or local path)
- URDF file path under `src/robot/urdf/`
- Home constants and units:
  - `HOME_SHOULDER_ANGLE`, `HOME_XYZ`, `HOME_QUAT`, `HOME_JOINTS`, `IS_DEGREES`
- Connection requirements (`robot_ip`, token, local IP)
- Telemetry availability for joint states and optional IMU

## Workflow

1. Verify SDK importability.
- If the SDK is in `requirements.txt`, ensure import path matches package docs.
- If local package, ensure repo-relative install path works.

2. Update robot constants in `src/robot/robot_interface.py`.
- Replace all `# {~.~}` placeholders that define robot identity, URDF path, sample frequency, home constants, and units.

3. Implement SDK connection in `RobotInterface.__init__`.
- Instantiate the SDK client.
- Enable required control mode (ROS or native command mode).
- Set robot operational state if needed (unbrake, enable motors, clear faults).
- Keep `sim` behavior unchanged.

4. Implement required methods using SDK APIs.
- `__get_joint_positions`
- `__get_tcp_pose`
- `move_to_joint`
- `move_to_pose`
- `publish_and_record_joint_positions`

5. Enforce interface contracts.
- Return radians internally unless converting at boundaries.
- Ensure joint count equals URDF model joint count (truncate extras, reject fewer).
- Return pose as `[x, y, z, qx, qy, qz, qw]`.
- Build `data_log` deque rows with keys:
  - `cmd_time`, `input_positions`, `output_positions`, `velocities`, `efforts`, `imu_time`, `linear_acceleration`, `angular_velocity`, `orientation`
- If a field is unavailable, keep empty value but preserve row.

6. Validate.
- Run static check: `python -m py_compile src/robot/robot_interface.py`
- Run a short simulation or dry-run calibration call without hardware motion when possible.
- Confirm no remaining `{~.~}` markers.

## Output requirements

- Modified `src/robot/robot_interface.py` with all required sections integrated.
- Updated `requirements.txt` (if needed).
- Brief summary listing SDK-specific assumptions and any unsupported telemetry fields.
