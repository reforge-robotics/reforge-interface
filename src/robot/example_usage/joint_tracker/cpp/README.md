# Joint Tracker C++ Example Usage

This hardware-free example mirrors the Python Joint Tracker example:

- **Example 1** optimizes a complete trajectory before execution.
- **Example 2A** appends a known trajectory and consumes optimized windows.
- **Example 2B** appends exactly one sample per online-planner update while
  one-sample windows are consumed. Its simulation pauses at source thresholds
  56, 111, and 170 for 3, 3, and 4 output updates, respectively, so the
  held-reference recovery path is visible in the metrics.

The example uses only the public `ReforgeJointTracker::joint_tracker` target and
does not require `reforge-interface`, a robot-vendor SDK, or live hardware.

## Install prerequisites

Install the SDK package and, for plots, Python Matplotlib:

```bash
sudo apt install reforge-core-joint-tracker cmake g++ python3 python3-matplotlib
```

The example CMake file first looks for the installed
`ReforgeJointTracker::joint_tracker` target. Reforge source-tree developers
building against this checkout instead of the published package can opt into the
local source fallback with:

```bash
cmake -S . -B build -DREFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK=OFF
```

The source fallback also needs the repository's locked Clarabel C adapter and
the header-only `nlohmann-json==3.12.0` package. Materialize/build Clarabel with
the native SDK workflow, then pass both dependency locations explicitly:

```bash
cmake -S . -B build \
  -DREFORGE_JOINT_TRACKER_EXAMPLE_REQUIRE_INSTALLED_SDK=OFF \
  -DREFORGE_SHAPER_CLARABEL_ROOT=/path/to/Clarabel.cpp \
  -DREFORGE_NLOHMANN_JSON_INCLUDE_DIR=/path/to/nlohmann_json/include
```

## Build and run

From this directory:

```bash
cmake -S . -B build
cmake --build build --parallel
./build/joint_tracker_example_usage
```

Headless CI run with machine-readable metrics:

```bash
./build/joint_tracker_example_usage --no-plots \
  --metrics-json /tmp/joint_tracker_cpp_metrics.json
```

Save the two command/response figures without opening a window:

```bash
./build/joint_tracker_example_usage --no-plots \
  --save-plots /tmp/joint_tracker_cpp_plots
```

Without `--no-plots`, Matplotlib opens the figures after saving them. `--save-plots`
also leaves the generated CSV and Python render script in the output directory so
users can inspect or rerun the exact plot input data. It still renders figures
when combined with `--no-plots`; this makes automated image verification
deterministic.
