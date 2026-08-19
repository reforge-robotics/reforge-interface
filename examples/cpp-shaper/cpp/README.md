# Hardware-free C++ Covalent Shaper example

This example runs all four controller modes without connecting to a robot. It
is an installed-package consumer: CMake discovers `ReforgeShaper 2.0.9` with
`find_package(ReforgeShaper CONFIG REQUIRED)` and links the public
`ReforgeShaper::backend` target. A Reforge source checkout, Python virtual
environment, `CMAKE_PREFIX_PATH`, and `LD_LIBRARY_PATH` are not required.

The qualified customer platform is Ubuntu 24.04 (noble), amd64. The pinned APT
package is `reforge-core-shaper=2.0.9-907`.

## Install

Add the official Reforge repository and install the exact qualified package:

```bash
curl -fsSL https://reforge-robotics.github.io/reforge-core-cpp/setup.sh \
  | sudo bash
sudo apt-get update
sudo apt-get install -y \
  reforge-core-shaper=2.0.9-907 \
  cmake \
  g++ \
  python3-dev \
  python3-matplotlib \
  python3-numpy
```

`python3-pil` and `strace` are qualification-only packages used by the plot
regression and network audit; the runnable example does not require them.

Confirm the installed package before building:

```bash
dpkg-query -W -f='${Package} ${Version} ${Architecture}\n' \
  reforge-core-shaper
```

The expected output is:

```text
reforge-core-shaper 2.0.9-907 amd64
```

## Build and run

From the `cpp/` directory, the reviewed helper configures a Release build using
only normal system discovery, builds it, and runs it:

```bash
./build_and_run.sh --headless --output-dir ./build/figures
```

`--no-gui` is an alias for `--headless`. `--build-dir PATH` and
`--assets-dir PATH` override the default build and shared asset directories.
`--baseline PATH` overrides the copied canonical expected-metrics manifest.
Run `./build_and_run.sh --help` for the complete interface.

The equivalent explicit commands are:

```bash
unset CMAKE_PREFIX_PATH LD_LIBRARY_PATH
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/shaper_example_usage \
  --headless \
  --output-dir ./build/figures
```

Headless mode performs all controller calculations and validation without
opening GUI windows. It writes the two runtime figures, the deterministic
documentation close-up, five trajectory CSV files, and `metrics.json`.

Interactive mode opens exactly two windows:

```bash
./build_and_run.sh
```

If `--output-dir` is used without `--headless`, the same two runtime figures
are also saved. The third close-up remains headless-only so the interactive
behavior stays parallel with Python.

## Deterministic assets

By default, CMake copies the adjacent shared `../assets/` directory beside the
executable. A custom `--assets-dir` must contain all three inputs:

```text
model/model_bundle.json
model/shaper_models.native.json
modelone.urdf
```

The native artifact and URDF are the qualified Standard Bots bundle from
`reforge-interface` commit `22dc942`; they are checked against the frozen
SHA-256 identities during parity qualification.

The executable loads every expected runtime metric and numeric tolerance from
`validation/expected_metrics.json`, copied beside the executable at build time.
The approved cross-platform metric tolerance is `2e-5` in each metric's printed
unit; array tolerances remain field-specific and substantially tighter.

## Useful failures

Configuration and execution fail nonzero with actionable diagnostics when:

- `reforge-core-shaper` or its `ReforgeShaperConfig.cmake` is missing;
- Python development headers, Matplotlib, or NumPy are missing;
- the model manifest, native artifact, or `modelone.urdf` is missing;
- a model artifact is malformed; or
- the baseline manifest is missing, malformed, or differs from the runtime.

For a missing package, verify `apt-cache policy reforge-core-shaper` and the
`dpkg-query` command above. Do not work around discovery by pointing CMake or
the dynamic loader at a Reforge build tree.

## Dependency boundary

The vendored, MIT-licensed `matplotlib-cpp` adapter is private to this example.
Python, Matplotlib, and NumPy are not exposed by the installed
`ReforgeShaper::backend` target. Consumer qualification rejects unresolved
libraries, LibTorch, Torch paths, Reforge source/build paths, robot SDKs, and
network dependencies in the executable closure.
