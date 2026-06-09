---
name: robot-sdk-calibration
description: Integrate a robot SDK into reforge-interface for calibration and system identification. Use when a user provides a robot SDK package and URDF and needs src/robot/robot_interface.py wired to the ArmClient command and telemetry contract.
---

# Robot SDK Calibration Integration (Codex)

Implement robot SDK integration in `src/robot/robot_interface.py` for calibration workflows.

## Inputs required

- Robot SDK location (pip package name, git URL, or local path)
- URDF file path under `src/robot/urdf/`
- Full-stretch constants and units:
  - `FULL_STRETCH_XYZ`, `FULL_STRETCH_QUAT`, `FULL_STRETCH_JOINTS`, `IS_DEGREES`
- Connection requirements (`robot_ip`, token, local IP)
- Telemetry availability for joint positions, velocities, and efforts

## Workflow

1. Verify SDK importability.
- If the SDK is in `src/robot/requirements.txt`, ensure import path matches package docs.
- If local package, ensure repo-relative install path works.

2. Update robot constants in `src/robot/robot_interface.py`.
- Replace all `# {~.~}` placeholders that define robot identity, URDF path,
  sample frequency, full-stretch constants, IMU selection, and units.

3. Implement SDK connection in `RobotInterface.__init__`.
- Instantiate the SDK client.
- Enable required control mode (ROS or native command mode).
- Set robot operational state if needed (unbrake, enable motors, clear faults).
- Keep `sim` behavior unchanged.

4. Implement required methods using SDK APIs.
- `command_move_j`
- `command_move_pose`
- `command_servo_j`
- `enter_position_mode`
- `enter_servo_mode`
- `get_joint_state`
- `get_tcp_pose`

5. Enforce interface contracts.
- Return radians internally unless converting at boundaries.
- Ensure SDK arm telemetry count equals the actuated URDF model joint count.
- Return pose as `[x, y, z, qx, qy, qz, qw]`.
- Return joint state as `(q, qd, tau)`.
- Keep `USE_REFORGE_IMU=True` unless the user explicitly requests a
  vendor-native `ImuRecorder`.
- Leave streaming-loop timing, IMU alignment, and data logging to `reforge-core`.

6. Validate.
- Run static check: `python -m py_compile src/robot/robot_interface.py`
- Run a short simulation or dry-run calibration call without hardware motion when possible.
- Confirm no remaining `{~.~}` markers.

## Output requirements

- Modified `src/robot/robot_interface.py` with all required sections integrated.
- Updated `src/robot/requirements.txt` and `src/robot/pyproject.toml` (if needed).
- Brief summary listing SDK-specific assumptions and any unsupported telemetry fields.
