# Robot SDK Auto-Integration Skills

This guide explains how to let AI agents automatically integrate a robot SDK into `reforge-interface` for calibration.

## What users need to provide

1. URDF file
- Add your robot URDF to `src/robot/urdf/`.

2. Robot SDK
- Add your SDK dependency in `src/robot/requirements.txt`, or
- Add a local SDK package in the repo that can be imported by Python.

3. Robot-specific values
- Robot ID (if applicable)
- Sampling frequency (Hz)
- Full-stretch calibration values (`FULL_STRETCH_XYZ`,
  `FULL_STRETCH_QUAT`, `FULL_STRETCH_JOINTS`)
- Unit convention (degrees vs radians)
- Connection details (IP/token/local IP)

## What the integration skill updates

The skill updates `src/robot/robot_interface.py` by replacing all `# {~.~}` sections:

- SDK imports
- Robot constants (`BOT_ID`, `URDF_PATH`, `ROBOT_MAX_FREQ`, full-stretch constants)
- SDK client initialization in `__init__`
- Required robot methods:
  - `command_move_j`
  - `command_move_pose`
  - `command_servo_j`
  - `enter_position_mode`
  - `enter_servo_mode`
  - `get_joint_state`
  - `get_tcp_pose`

It preserves the existing calibration pipeline and only changes SDK-dependent parts.

## Required interface contract

The integrated interface must keep these contracts:

- Joint values are handled internally in radians
- TCP pose is returned as `[x, y, z, qx, qy, qz, qw]`
- `get_joint_state` returns `(q, qd, tau)` with one value per actuated URDF joint
- Command and mode methods return `0` on success unless the SDK exposes a
  meaningful integer status
- `reforge-core` owns streaming-loop timing, IMU alignment, and data-log schemas
- The default sensor path is the Reforge USB IMU (`USE_REFORGE_IMU=True`)

## Codex Agent skill setup

1. Create skill folder in Codex home:
- `$CODEX_HOME/skills/robot-sdk-calibration/`

2. Add `SKILL.md`:
- Use `skill-templates/codex/robot-sdk-calibration/SKILL.md` from this repo.

3. Optional UI metadata:
- Add `agents/openai.yaml` if your Codex environment uses skill chips/UI metadata.

4. Trigger examples:
- `Use robot-sdk-calibration to integrate my xArm SDK in this repo.`
- `$robot-sdk-calibration integrate SDK from src/robot/requirements.txt and src/robot/urdf/my_robot.urdf`

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
- SDK install source: <src/robot/requirements.txt entry or local path>
- Robot IP mode: <ip or sim>
- Units: <degrees|radians>
- Home constants:
  - FULL_STRETCH_XYZ=[...]
  - FULL_STRETCH_QUAT=[...]
  - FULL_STRETCH_JOINTS=[...]
- IMU: Reforge USB IMU

Requirements:
- Replace all {~.~} sections only where needed.
- Keep existing calibration flow intact.
- Implement the current ArmClient methods exactly.
- Leave arm/IMU recording and alignment to reforge-core.
- Run py_compile and report assumptions.
```

## Validation checklist for users

- `python -m py_compile src/robot/robot_interface.py`
- `rg "\{~\.~\}" src/robot/robot_interface.py` returns no matches
- URDF path resolves correctly
- Joint count from SDK matches the actuated URDF model joints
- Reforge IMU is detected over the configured communication mode
- Calibration run can start without schema or unit errors

## Common issues

1. Joint units mismatch
- Symptom: unstable motion or incorrect home movement.
- Fix: set `IS_DEGREES` correctly and convert only at SDK boundaries.

2. Extra joints from hardware (gripper/fixed joints)
- Symptom: length mismatch errors.
- Fix: use the SDK's arm-only telemetry API or select a matching URDF.

3. Reforge IMU unavailable
- Symptom: calibration cannot initialize sensor recording.
- Fix: check USB access, communication mode, physical connection, and supported
  recording frequency.

4. SDK motion mode not enabled
- Symptom: commands succeed in code but robot does not move.
- Fix: add SDK-specific enable/unbrake/control-state setup in `__init__`.
