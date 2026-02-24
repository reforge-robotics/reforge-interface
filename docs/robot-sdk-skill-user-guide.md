# Robot SDK Auto-Integration Skills

This guide explains how to let AI agents automatically integrate a robot SDK into `reforge-interface` for calibration.

## What users need to provide

1. URDF file
- Add your robot URDF to `src/robot/urdf/`.

2. Robot SDK
- Add your SDK dependency in `requirements.txt`, or
- Add a local SDK package in the repo that can be imported by Python.

3. Robot-specific values
- Robot ID (if applicable)
- Sampling frequency (Hz)
- Home pose values (`HOME_XYZ`, `HOME_QUAT`, `HOME_JOINTS`)
- Unit convention (degrees vs radians)
- Connection details (IP/token/local IP)

## What the integration skill updates

The skill updates `src/robot/robot_interface.py` by replacing all `# {~.~}` sections:

- SDK imports
- Robot constants (`BOT_ID`, `URDF_PATH`, `ROBOT_MAX_FREQ`, home constants)
- SDK client initialization in `__init__`
- Required robot methods:
  - `_get_joint_positions`
  - `_get_tcp_pose`
  - `move_to_joint`
  - `move_to_pose`
  - `publish_and_record_joint_positions`

It preserves the existing calibration pipeline and only changes SDK-dependent parts.

## Required output contract

The integrated interface must keep these contracts:

- Joint values are handled internally in radians
- TCP pose is returned as `[x, y, z, qx, qy, qz, qw]`
- `publish_and_record_joint_positions` returns deque rows with keys:
  - `cmd_time`
  - `input_positions`
  - `output_positions`
  - `velocities`
  - `efforts`
  - `imu_time`
  - `linear_acceleration`
  - `angular_velocity`
  - `orientation`

If a telemetry field is unavailable, the field remains present with an empty value.

## Codex Agent skill setup

1. Create skill folder in Codex home:
- `$CODEX_HOME/skills/robot-sdk-calibration/`

2. Add `SKILL.md`:
- Use `skill-templates/codex/robot-sdk-calibration/SKILL.md` from this repo.

3. Optional UI metadata:
- Add `agents/openai.yaml` if your Codex environment uses skill chips/UI metadata.

4. Trigger examples:
- `Use robot-sdk-calibration to integrate my xArm SDK in this repo.`
- `$robot-sdk-calibration integrate SDK from requirements.txt and urdf/my_robot.urdf`

## Claude Code skill setup

1. Create project skill folder:
- `.claude/skills/robot-sdk-calibration/`

2. Add `SKILL.md`:
- Use `skill-templates/claude/robot-sdk-calibration/SKILL.md` from this repo.

3. Trigger examples:
- `Use the robot-sdk-calibration skill to integrate my SDK using src/robot/urdf/my_robot.urdf`
- `Use robot-sdk-calibration to wire src/robot/robot_interface.py from the SDK docs I provided.`

## Recommended user prompt template

Use this prompt after adding URDF + SDK:

```text
Integrate my robot SDK into src/robot/robot_interface.py for calibration.

Context:
- URDF path: src/robot/urdf/<my_robot>.urdf
- SDK import: <sdk import path>
- SDK install source: <requirements entry or local path>
- Robot IP mode: <ip or sim>
- Units: <degrees|radians>
- Home constants:
  - HOME_SHOULDER_ANGLE=...
  - HOME_XYZ=[...]
  - HOME_QUAT=[...]
  - HOME_JOINTS=[...]

Requirements:
- Replace all {~.~} sections only where needed.
- Keep existing calibration flow intact.
- Implement required methods and output data_log keys exactly.
- Run py_compile and report assumptions.
```

## Validation checklist for users

- `python -m py_compile src/robot/robot_interface.py`
- `rg "\{~\.~\}" src/robot/robot_interface.py` returns no matches
- URDF path resolves correctly
- Joint count from SDK matches URDF model joints (or extra joints are safely truncated)
- Calibration run can start without schema or unit errors

## Common issues

1. Joint units mismatch
- Symptom: unstable motion or incorrect home movement.
- Fix: set `IS_DEGREES` correctly and convert only at SDK boundaries.

2. Extra joints from hardware (gripper/fixed joints)
- Symptom: length mismatch errors.
- Fix: truncate to URDF joint count while validating minimum expected count.

3. Missing IMU or effort data
- Symptom: recorder fields absent.
- Fix: keep required keys with empty values instead of removing keys.

4. SDK motion mode not enabled
- Symptom: commands succeed in code but robot does not move.
- Fix: add SDK-specific enable/unbrake/control-state setup in `__init__`.
