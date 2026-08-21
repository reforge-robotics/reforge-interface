# UR5e Live Adapter (Part 1)

Live-hardware `ArmClient` implementation for a Universal Robots UR5e, via
`ur_rtde` (`rtde_control.RTDEControlInterface` / `rtde_receive.RTDEReceiveInterface`).
Covers Part 1 of the take-home only — see Limitations.

## Architecture

- `RobotInterface` (`robot_interface.py`) implements `reforge_core`'s abstract
  `ArmClient` contract. It is **live-only**: `robot_ip == "sim"` is rejected
  here. Simulator mode constructs a separate, Reforge-internal class
  (`SimRobotInterface`) through `reforge_core.calibration.run_helpers` —
  `RobotInterface` is never involved when running in `sim`.
- `_RtdeClients` bundles both SDK objects (`control`, `receive`) behind the
  single `self.robot` slot `ArmClient._require_connected_arm()` expects. Both
  are constructor-injectable (`rtde_c`/`rtde_r`) so tests never open a socket.
- `_speed_pct_to_joint_sa`/`_speed_pct_to_linear_sa` map Reforge's 0-100
  `speed` percentage to RTDE's SI `speed`/`acceleration` arguments.
- The URDF (`urdf/ur5e.urdf`) feeds Reforge's own `Dynamics`/Pinocchio model
  for joint-count validation and kinematics; sourced from Universal Robots'
  official `Universal_Robots_ROS2_Description` repo (BSD-3-Clause).

## Setup

```bash
docker compose build
```

Rebuilds the Ubuntu 24.04 dev container; `ur-rtde==1.6.5` installs
automatically from `src/robot/requirements.txt`.

## Exact commands

```bash
# Tests (no robot required — dependency-injected fakes)
docker compose run --rm app bash -c "python -m pytest src/robot/tests/ -v"

# Compile check
docker compose run --rm app python -m py_compile src/robot/robot_interface.py

# Against real hardware or URSim (not yet run — see Limitations)
python -m robot.run connect_test <robot_ip>
python -m robot.run calibrate <robot_ip> --robot_id <id> --freq 125
```

## Frame transform

RTDE already reports TCP pose in the robot base frame, matching Reforge's
convention — **no spatial transform is needed** for Part 1. The only
conversion is representational: RTDE's rotation vector ↔ Reforge's
`[qx, qy, qz, qw]` quaternion, via `scipy.spatial.transform.Rotation` (both
directions implemented; round-trip tested near zero and near π rad).

## Timing policy

- `ROBOT_MAX_FREQ = 125` Hz: RTDE's own interface cap (Universal Robots RTDE
  guide), independent of the controller's internal 500 Hz servo loop.
- `command_servo_j` computes its own command period
  (`1 / self.max_sampling_frequency_hz`) and passes it as `servoJ`'s `time`
  argument; `ArmClient.stream_joint_positions` (base class, unmodified)
  already paces calls to it externally at that rate.
- `servoJ`'s `lookahead_time=0.1s`, `gain=800`: taken directly from
  `ur_rtde`'s own official `servoj_example.py`, not guessed.

## Units

| Quantity | Unit | Source |
|---|---|---|
| Joint position/velocity | rad, rad/s | RTDE native — no conversion applied |
| Joint effort | N·m | `RTDEControlInterface.getJointTorques()` (gravity/friction-corrected) |
| TCP position | m | RTDE native |
| TCP orientation | `[qx, qy, qz, qw]` | converted from RTDE rotation vector |
| `speed` | 0–100% → SI | linearly mapped onto `ur_rtde`'s own `moveJ`/`moveL` defaults as the 100% ceiling (≈33% of the UR5e's 180°/s hard joint-velocity limit) |

## Assumptions

- URDF sourced from Universal Robots' official repo (not supplied with the
  starter repo); attribution/license preserved in the file header.
- `FULL_STRETCH_*` home pose computed via forward kinematics against that
  URDF, not hand-derived — but not yet validated against a physical robot.
- Speed/accel ceiling anchored to the SDK's own documented defaults rather
  than the datasheet hard limit — a judgment call, open to revision.
- RTDE has no explicit "mode" concept: `enter_position_mode` calls
  `servoStop()` (required before point-to-point moves); `enter_servo_mode` is
  a liveness check only, since `servoJ` needs no separate enable step.

## Limitations

- **Not yet tested against real hardware or URSim** (Universal Robots' own
  RTDE-speaking simulator) — all verification so far uses injected fakes.
- **Part 2 (MuJoCo) and Part 3 (Shaper proof) not started.** The composition
  point they require (`construct_robot_interface`, `SimulatorConfiguration`,
  the `--sim-engine` CLI flag) lives inside the installed `reforge_core`
  package, not this repo's editable source — not achievable against the
  current checkout.
- Cosmetic: robot display name still reads `"My Robot"`
  (`robot_interface.py:224`), not yet set to something UR5e-specific.
