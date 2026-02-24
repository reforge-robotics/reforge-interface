---
name: robot-sdk-calibration
description: Integrate a robot SDK into reforge-interface for calibration and system identification. Use when a project contains src/robot/robot_interface.py and the user needs SDK-specific implementation of required robot methods plus URDF and dependency wiring.
---

# Robot SDK Calibration Integration (Claude Code)

Use this skill to implement robot SDK support in `src/robot/robot_interface.py`.

## Required project expectations

- URDF is present in `src/robot/urdf/`
- SDK is pip-installable or available as a local package in repo
- SDK docs expose APIs for:
  - Reading joint positions
  - Reading TCP pose
  - Commanding joint or Cartesian motion
  - (Optional) reading effort/current and IMU data

## Execution steps

1. Inspect `src/robot/robot_interface.py` and replace all `# {~.~}` markers.
2. Add SDK import and client initialization in `RobotInterface.__init__`.
3. Keep simulator path (`robot_ip == "sim"`) unchanged.
4. Implement required methods:
- `_get_joint_positions`
- `_get_tcp_pose`
- `move_to_joint`
- `move_to_pose`
- `publish_and_record_joint_positions`
5. Preserve expected units and data schemas:
- Internal joint units in radians
- TCP pose output `[x, y, z, qx, qy, qz, qw]`
- `data_log` deque rows with required keys:
  - `cmd_time`, `input_positions`, `output_positions`, `velocities`, `efforts`, `imu_time`, `linear_acceleration`, `angular_velocity`, `orientation`
6. Update dependency source (`requirements.txt` or local path reference).
7. Validate with:
- `python -m py_compile src/robot/robot_interface.py`
- Search for unresolved markers: `rg "\{~\.~\}" src/robot/robot_interface.py`

## Guardrails

- Do not change pre-defined calibration flow methods unless needed for SDK compatibility.
- Do not alter expected output keys or their semantics.
- If telemetry is missing, return empty fields rather than changing schema.
- Document assumptions in final summary.
