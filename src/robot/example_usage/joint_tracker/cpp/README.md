# Joint Tracker C++ example

This hardware-free example uses the installed
`ReforgeJointTracker::joint_tracker` CMake target. It does not connect to a
robot, require a vendor SDK, or fall back to a private Reforge source tree.

## Install and build

Install a `reforge-core-joint-tracker` package compatible with the Reforge SDK
selected by this public branch, plus CMake and a C++17 compiler. Then configure
from this directory; the package provides the
`ReforgeJointTracker::joint_tracker` target:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

Run all three deterministic controller examples without plots:

```bash
./build/joint_tracker_example_usage --no-plots \
  --metrics-json /tmp/joint_tracker_cpp_metrics.json
```

Use `--save-plots <directory>` to write the command/response figures and their
CSV inputs. Without `--no-plots`, Matplotlib opens the figures after saving
them. Plot rendering requires Python 3 and Matplotlib at runtime; they are not
C++ build dependencies.

Configuration fails with an actionable error if the selected SDK does not
provide its exported CMake target or the optimizer API required by Example 2B.
