# Yaskawa ACU SDK integration

This private package is the software-qualified Yaskawa integration boundary for issue
[#343](https://github.com/reforge-robotics/reforge-core/issues/343). It validates
YNX1000/NEX7 discovery and mode responses, consumes synchronized feedback, and
exposes joint, torque, and TCP state in Reforge units. Phase 4 adds queued
joint motion and an incremental servo session behind an injected ACU bridge;
the client never changes controller mode or servo power.

`contract.py` validates the vendor field contract, fixed-width axis mapping,
six-axis rotary layout, response statuses, timestamps, and units.
`read_only.py` owns connection and stream lifecycle, bounded caching,
stale/reordered rejection, velocity derivation after the tick period is known,
and clean shutdown. `motion.py` mirrors the exact target/receive/motion and
incremental task RPCs, validates mode/Remote plus explicit operator servo
confirmation, discovered joint limits, queue status, and bounded incremental
deltas. The supplied SDK has no servo-state getter, so the adapter never
assumes one and never powers servos implicitly.
`diagnostic.py` collects a bounded single-rate diagnostic.
`preflight.py` validates the multi-rate evidence returned by the ACU package and
derives comparable timing and process-load metrics without automatically
choosing a production rate.

`grpc_transport.py` is the external Python client for the Reforge-owned bridge.
It validates the contract and SDK versions, merges the three raw streams only
at equal controller timestamps, and opens an exclusive motion lease lazily on
the first mutating command. A transport failure invalidates the local lease so
it cannot be silently reused. The default `RobotInterface` constructs this
transport from the connection argument, so the staged public package no longer
requires a private checkout or dependency injection.

The controller timestamp counts I/O cycles rather than seconds. Joint velocity
and controller-aligned Unix time therefore remain unavailable until an
authorized preflight establishes `controller_tick_period_s`. Host monotonic
time is used only for cache staleness. The candidate Euler convention remains
diagnostic-only until a known-pose result confirms it.

The ACU base image is AArch64/Python 3.9, while this repository requires Python
3.11. `acu_app/` therefore contains the read-only recorder and one narrow,
versioned C++ gRPC bridge for discovery, telemetry, and joint motion. Its
build graph includes SystemInfo, ModeGet, RealtimeMonitor, and Motion protobufs;
the repo-owned contract exposes no servo-power API. The reviewed ACU package
binds the bridge to `0.0.0.0:55051` and declares the host/container mapping
`55051:55051` in `forwarding_ports`, as required by the Yaskawa ACU
port-forwarding contract.
The connection argument therefore names the controller's approved private
network address, such as `<controller-ip>:55051`. The bridge is unauthenticated;
do not expose this forwarded port outside the trusted controller network.
Generated Python bindings for the Reforge-owned contract are tracked under
`vendor/`. Generated Yaskawa SDK bindings, build outputs, and the ACU archive
remain ignored and outside Git.

Yaskawa SDK archives, toolchains, controller images, manuals, credentials, and
other proprietary artifacts must not be committed here. Hashes and private-use
provenance are recorded in `PROVENANCE.md`. The supplied NEX07C00 URDF and its
referenced visual/collision meshes are tracked under `urdf/NEX07C00/`; their
source hash and the reported public-redistribution approval are recorded in
that directory's `PROVENANCE.md`.

## Customer-candidate settings

`connect_test` needs only the forwarded endpoint. Sayre Jeannet measured about
`1.00005e-6` seconds per controller timestamp tick on the qualification
controller; the value below is the reviewed rounded setting for that system.
Measure it again when the controller or RobotControlService changes. The
feedback rate remains an explicit deployment choice; leave the command-rate
override unset unless a separately qualified motion procedure specifies it:

```bash
export YASKAWA_CONTROLLER_TICK_PERIOD_S=1e-6
export YASKAWA_FEEDBACK_RATE_HZ=25
unset YASKAWA_COMMAND_RATE_HZ
```

Live motion additionally requires the operator to confirm servo power with the
exact acknowledgement below. The interface reads it at each motion gate and
does not power servos itself.

```bash
export YASKAWA_SERVO_POWER_CONFIRMED=I_CONFIRM_SERVOS_ARE_ON
```

The adapter loads the supplied `NEX07C00/NEX07C00.urdf`. Availability of this
geometry does not itself validate the installed controller variant, SDK joint
ordering/sign conventions, discovered limits, payload/tool, full-stretch pose,
workspace, or trajectory; those remain Phase 6 gates.

## Authorized controller setup and calibration

Do not perform these steps without an approved access window and a Yaskawa
operator. The controller must remain on a trusted private network. Follow the
Yaskawa ACU manual if it conflicts with this summary.

These commands assume an exported `yaskawa-sdk` checkout, where this runbook is
`src/robot/README.md` and the selected Yaskawa adapter is
`src/robot/robot_interface.py`. Do not run them directly from the private
multi-integration `reforge-core` checkout, whose launcher may intentionally
select a different adapter.

1. Verify the private `reforge-yaskawa-bridge-0.4.1.tar` archive. Its SHA-256
   must be
   `ef738eb289fe775525b60efa0747f206417823782b250087e7e6cb4334e0256c`.
   Do not extract the archive.
2. In ACU System Manager, import the archive as an ACU application. Confirm its
   metadata declares `forwarding_ports` as `55051:55051`, leave automatic
   startup disabled, and start it manually using the approved controller
   procedure.
3. Confirm `yaskawa_bridge_v1` remains running and can reach the in-controller
   `RobotControlService:50300` endpoint. Starting the bridge neither changes
   Teach/Play mode nor enables servo power.
4. From a fresh Python 3.11+ environment on the test computer, install the
   interface and verify the connection without sending motion:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r src/robot/requirements.txt
   pip install -e src/robot
   export YASKAWA_CONTROLLER_TICK_PERIOD_S=1e-6
   python3 -m robot.run connect_test <controller-ip>:55051
   ```

5. Before calibration, have the operator verify the robot-specific joint
   mapping, limits, workspace clearance, tool/payload, Play mode, Remote state,
   servo state, and stop procedure. Then set the explicit motion gate:

   ```bash
   export YASKAWA_SERVO_POWER_CONFIRMED=I_CONFIRM_SERVOS_ARE_ON
   export YASKAWA_FEEDBACK_RATE_HZ=25
   unset YASKAWA_COMMAND_RATE_HZ
   ```

6. Run the user-selected 200 Hz calibration procedure with the reachability
   envelope used during the Yaskawa integration:

   ```bash
   python3 -m robot.run calibrate <controller-ip>:55051 \
     --freq 200 --nv 1 --nr 1 \
     --min_radius_scale 0.8 --max_radius_scale 0.8
   ```

The default IMU communication mode is `virtual`, which permits robot-motion and
workflow qualification without a connected Muse device. It does not produce
physical IMU measurements and does not establish a valid Shaper calibration.
`--freq 200` is the requested calibration sample rate; it does not qualify or
set a 200 Hz low-level command transport rate. The earlier hardware exchange
reported `SetIncrementMove` latencies above a 200 Hz cycle period, so the
operator must treat timing-guard activation as a stop condition and preserve
the logs rather than disabling the guard.

## Software verification

From the repository root, use the existing Python 3.11 environment:

```bash
pytest -q \
  src/tests/test_yaskawa_contract.py \
  src/tests/test_yaskawa_read_only.py \
  src/tests/test_yaskawa_preflight.py \
  src/tests/test_yaskawa_motion.py \
  src/tests/test_yaskawa_bridge_contract.py \
  src/tests/test_yaskawa_grpc_transport.py \
  src/robot/tests/test_catalog_entries.py
scripts/check.sh
```

Build and packaging instructions are in `acu_app/README.md`. Controller
installation and data capture require a separately approved access window and
Yaskawa operator. The `dry_run_joint_move()` report is the only Phase 4
preflight command intended for software-only use; it publishes no motion.
Live commands require a separately approved procedure.
