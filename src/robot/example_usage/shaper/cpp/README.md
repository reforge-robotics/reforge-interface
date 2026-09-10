# Hardware-free C++ Covalent Shaper example

This example runs all four controller modes without connecting to a robot. It
is an installed-package consumer: CMake discovers the C++ SDK bundled in the
installed `reforge-core` wheel with
`find_package(ReforgeShaper CONFIG REQUIRED)` and links the public
`ReforgeShaper::backend` target. The Python extension and C++ SDK therefore come
from one versioned artifact. A Reforge source checkout and loader-path overrides
are not required.

The wheel provides the same-version Python native extension and relocatable C++
SDK used by the parity tests. The example uses Python and Matplotlib only to
render plots; Python development headers and NumPy are not build dependencies.
The checked-in `plotting/matplotlib_renderer.py` script is copied beside the
executable at build time and invoked only when plots are requested.

## Install

Create an environment and install the wheel plus the example-only plotting
dependency:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade reforge-core matplotlib
python -m pip install "nlohmann-json==3.12.0"
```

Install CMake and a C++17 compiler using the normal package manager for the
host. Pillow and `strace` are qualification-only dependencies used by the plot
regression and network audit; the runnable example does not require them.

Confirm that Python and CMake will consume the same installed wheel:

```bash
python -c 'from importlib.metadata import version; print(version("reforge-core"))'
python -c 'from reforge_core import cmake_prefix; print(cmake_prefix())'
```

## Build and run

From the `cpp/` directory, pass the wheel-owned SDK prefix returned by the
supported `cmake_prefix()` helper, build, and run the example directly:

```bash
REFORGE_SHAPER_PREFIX="$(python -c \
  'from reforge_core import cmake_prefix; print(cmake_prefix())')"
NLOHMANN_JSON_INCLUDE_DIR="$(python -c \
  'import nlohmann_json; print(nlohmann_json.get_include())')"
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DReforgeShaper_ROOT="$REFORGE_SHAPER_PREFIX" \
  -DREFORGE_NLOHMANN_JSON_INCLUDE_DIR="$NLOHMANN_JSON_INCLUDE_DIR"
cmake --build build --parallel
./build/shaper_example_usage \
  --headless \
  --output-dir ./build/figures
```

Headless mode performs all controller calculations and validation without
opening GUI windows. It writes the two runtime figures, the deterministic
documentation close-up, five trajectory CSV files, the plot adapter's CSV, and
`metrics.json`.

Interactive mode opens exactly two windows:

```bash
./build/shaper_example_usage
```

If `--output-dir` is used without `--headless`, the same two runtime figures
are also saved. The third close-up remains headless-only so the interactive
behavior stays parallel with Python.

## Deterministic assets

By default, CMake copies the example-local `assets/` directory and the
repository-owned `src/robot/urdf/test_robot.urdf` beside the executable. A
custom `--assets-dir` must contain all four inputs:

```text
model_bundle.json
shaper_models.native.json
expected_metrics.json
test_robot.urdf
```

The compact native artifact is a deterministic, synthetic two-axis example
fixture. It has no customer provenance and is not a native/PyTorch conversion
qualification record. The manifest's `.pt` filenames are cross-format metadata;
the native loader selects `shaper_models.native.json` and does not require those
files to exist.

The executable loads every expected runtime metric and numeric tolerance from
`assets/expected_metrics.json`, copied beside the executable at build time.
The approved cross-platform metric tolerance is `2e-5` in each metric's printed
unit; array tolerances remain field-specific and substantially tighter.

## Useful failures

Configuration and execution fail nonzero with actionable diagnostics when:

- `reforge-core-shaper` or its `ReforgeShaperConfig.cmake` is missing;
- Python 3 or Matplotlib is missing when plots are rendered;
- the model manifest, native artifact, or `test_robot.urdf` is missing;
- a model artifact is malformed; or
- the baseline manifest is missing, malformed, or differs from the runtime.

For a missing package, verify the two Python commands in the Install section
and confirm that the selected interpreter belongs to the activated environment.
Do not work around discovery by pointing CMake or the dynamic loader at a
Reforge source or build tree.

## Dependency boundary

The example's process-based plotting adapter invokes the user's Python 3 and
Matplotlib installation only at runtime. Python development headers and NumPy
are not build dependencies, and plotting dependencies are not exposed by the
installed `ReforgeShaper::backend` target. Consumer qualification rejects
unresolved libraries, LibTorch, Torch paths, Reforge source/build paths, robot
SDKs, and network dependencies in the executable closure.
