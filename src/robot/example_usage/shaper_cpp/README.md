# Standard Bots C++ Shaper example

This hardware-free example loads the repository's selected native model bundle
and `modelone.urdf`, shapes one complete 1,053-point trajectory through the
installed `ReforgeShaper` and `ReforgeShaperRos2` CMake packages, checks native
and ROS adapter parity, and exercises the customer transport boundary.

The transport publishes one `trajectory_msgs/msg/JointTrajectory` containing one
point per 0.005-second sample. At that boundary only, the point's
`time_from_start` is filled with the current ROS clock timestamp because that is
the receiving bridge's wire convention. The complete trajectory retains normal
zero-based durations and is validated before any point is emitted.

The dry run invokes the transport's publish callback and does not initialize a
ROS graph or publish to a ROS topic. An application that owns a ROS publisher
can provide that callback and let the transport's default monotonic scheduler
drive the 0.005-second stream.

## Install, build, and dry-run

On Ubuntu 24.04 with ROS 2 Jazzy sourced, the install helper first configures
Reforge's signed public APT source using
`https://reforge-robotics.github.io/reforge-core-cpp/setup.sh`, then installs
the pinned base and ROS 2 companion packages:

```bash
./install_reforge_shaper.sh
source /opt/ros/jazzy/setup.bash
./run_shaper_example.sh
```

The dry run performs no ROS graph, network, or robot I/O. Expected output is:

```text
ReforgeShaper package: <installed-version>
Model: Standard Bots Model One (shaper_models.native.json)
Shaping time: <milliseconds> ms
Validation: pass; native/ROS parity: pass
Transport: 1053 ordered points at 0.005 s.
```

To use a different build directory or explicit asset paths:

```bash
./run_shaper_example.sh <build-dir> <model-directory> <modelone.urdf>
```

The model/URDF pair and hashes are recorded in
`src/robot/models/current/shaper/standard_bots_model_manifest.json`.
