# Standard Bots real-hardware qualification harness

This standalone ROS 2 Jazzy C++ package implements Phase B of the real-hardware
qualification plan. It generates the frozen pose-10/axis-0 trial, calls the
installed `ReforgeShaper::ros2::ProcessTrajectory` interface exactly once for
the shaped case, verifies direct-native parity, and writes complete-message
evidence.

Dry-run is the default. That path does not initialize ROS, create a node or
publisher, inspect the robot endpoint, or perform network I/O:

```bash
standard_bots_qualification_cli \
  --assets-dir src/robot \
  --output-dir qualification-output
```

The output contains unshaped and shaped trajectory CSVs and immutable result
manifests. The frozen input is `config/phase_a_trial.json`.

## Safety boundary

Real execution requires all of the following independently:

- `--execute` and exactly one `--trial unshaped|shaped`;
- explicit endpoint, robot ID, command, joint-state, and IMU topics;
- a read-only Standard Bots SDK probe that reports ROS control `Enabled`;
- the frozen Phase A 0.02 rad current-pose, 0.05 rad following-error,
  1.25 rad/s velocity, and 2.0 rad/s² acceleration limits;
- fresh ordered `joint0` through `joint5` state, fresh IMU, active control state,
  and an expected command subscriber;
- valid finite message fields, 0.005 s timestamps, and all local limits; and
- an exact terminal confirmation after the endpoint, robot ID, topic, workspace,
  observer, and E-stop checklist is displayed.

The executable publishes one complete `JointTrajectory`, never one point per
host tick. A trial process performs only one publication. Run the unshaped and
shaped cases as separate invocations and inspect the evidence between them.

Read-only preflight uses `--preflight` with the explicit identity and topics.
It initializes ROS and observes graph/telemetry state but never calls publish.

## Clean Docker workflow

Place the verified
`standardbots-2.20260731.17-py3-none-any.whl` beside the extracted run 39
Debian packages, then build and run the no-network dry run. The build helper and
image independently reject a wheel whose SHA-256 differs from the frozen Phase
E value.

```bash
qualification/standard_bots/docker/build.sh /path/to/run-39/dist
qualification/standard_bots/docker/dry-run.sh
```

The build installs only the local `2.0.9-908` base and ROS companion packages
and the pinned `standardbots==2.20260731.17` wheel. It verifies the installed
SDK metadata and imports the control-state probe symbols during the build.
The dry-run container is pinned to `linux/amd64`, uses a read-only root
filesystem and `--network none`, and applies a seccomp profile that rejects the
socket API. Only the mounted evidence directory and an ephemeral `/tmp` are
writable. Use `docker/preflight.sh` and `docker/execute.sh` only on the approved
Ubuntu robot computer after Phase C and Phase D acceptance. Those later
commands use that same SDK from the runtime image; no virtual-environment mount
or package download is permitted. Neither example contains credentials;
Standard Bots must provide its own robot ID and the SDK token must enter only
through the environment.
