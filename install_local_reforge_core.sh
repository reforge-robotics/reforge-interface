#!/usr/bin/env bash
# Exit on the first error, treat unset variables as errors, and fail pipelines
# if any command in the pipeline fails.
set -euo pipefail

# Resolve the directory that contains this script so we can find the repo-local
# virtual environment even if the script is invoked from another directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Default to the sibling reforge-core checkout that lives next to this repo on
# the local machine.
DEFAULT_CORE_SDK_DIR="${HOME}/workspace/reforge-core/src/core_sdk"

# Allow the caller to override the SDK path either with:
# 1. the first positional argument, or
# 2. the REFORGE_CORE_SDK_DIR environment variable.
# If neither is provided, fall back to the default sibling checkout.
CORE_SDK_DIR="${1:-${REFORGE_CORE_SDK_DIR:-$DEFAULT_CORE_SDK_DIR}}"

# Prefer the Python interpreter from this repo's .venv so the editable install
# lands in the same environment the project is using. If that venv does not
# exist, fall back to $PYTHON or plain python3.
if [[ -x "${SCRIPT_DIR}/.venv/bin/python" ]]; then
  PYTHON_BIN="${SCRIPT_DIR}/.venv/bin/python"
else
  PYTHON_BIN="${PYTHON:-python3}"
fi

# Fail early with a clear message if the target directory does not look like the
# root of the local reforge-core SDK package.
if [[ ! -f "${CORE_SDK_DIR}/pyproject.toml" ]]; then
  echo "Expected reforge-core SDK at: ${CORE_SDK_DIR}" >&2
  echo "Pass the path as the first argument or set REFORGE_CORE_SDK_DIR." >&2
  exit 1
fi

# Print the two most important pieces of runtime context so it is obvious
# which Python environment is being modified and which local checkout is used.
echo "Using Python: ${PYTHON_BIN}"
echo "Installing local reforge-core from: ${CORE_SDK_DIR}"

# Preserve any existing CMAKE_PREFIX_PATH value before appending to it.
# The local reforge-core build compiles native extensions and needs Eigen.
# On macOS with Homebrew, Eigen is commonly installed outside default compiler
# search paths, so we provide explicit hints for both CMake and the compiler.
MAYBE_CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"
if command -v brew >/dev/null 2>&1; then
  if BREW_EIGEN_PREFIX="$(brew --prefix eigen 2>/dev/null)"; then
    # Header location used by includes like <Eigen/Dense>.
    EIGEN_INCLUDE_DIR="${BREW_EIGEN_PREFIX}/include/eigen3"

    # Help CMake find the Eigen package config files.
    export CMAKE_PREFIX_PATH="${BREW_EIGEN_PREFIX}${MAYBE_CMAKE_PREFIX_PATH:+:${MAYBE_CMAKE_PREFIX_PATH}}"

    # The sibling SDK's CMake expects these variables; setting them here avoids
    # editing the upstream reforge-core repo just to perform a local install.
    export CMAKE_ARGS="${CMAKE_ARGS:-} -DEigen3_DIR=${BREW_EIGEN_PREFIX}/share/eigen3/cmake -DEIGEN3_INCLUDE_DIR=${EIGEN_INCLUDE_DIR} -DEIGEN3_INCLUDE_DIRS=${EIGEN_INCLUDE_DIR}"

    # Also add Eigen to the compiler include path in case a CMake target does
    # not correctly propagate the include directory.
    export CPLUS_INCLUDE_PATH="${EIGEN_INCLUDE_DIR}${CPLUS_INCLUDE_PATH:+:${CPLUS_INCLUDE_PATH}}"
  fi
fi

# Editable installs for this package use a pyproject build backend. We need the
# wheel/scikit-build-core/pybind11 toolchain available in the active Python
# environment before building, but we only install them if they are missing.
if ! "${PYTHON_BIN}" -c "import pybind11, scikit_build_core, wheel" >/dev/null 2>&1; then
  "${PYTHON_BIN}" -m pip install wheel scikit-build-core pybind11
fi

# Remove any existing reforge-core installation first so there is no ambiguity
# about whether imports resolve to an older wheel or the local editable source.
if "${PYTHON_BIN}" -m pip show reforge-core >/dev/null 2>&1; then
  "${PYTHON_BIN}" -m pip uninstall -y reforge-core
fi

# Install the sibling checkout in editable mode.
#
# We intentionally do NOT pass --no-deps here: pip should read the dependency
# list from the target local repo's pyproject.toml and install any missing
# runtime requirements from there. That keeps the dependency source of truth in
# reforge-core itself rather than duplicating it in this repo or in this script.
#
# --no-build-isolation still reuses the current environment's build tooling
# instead of creating a temporary isolated build env.
"${PYTHON_BIN}" -m pip install --no-build-isolation -e "${CORE_SDK_DIR}"

# Show the installed package metadata so the result is easy to verify.
echo
echo "Installed package metadata:"
"${PYTHON_BIN}" -m pip show reforge-core

# Print the real import path as the final proof that Python now resolves
# reforge_core to the local checkout rather than to a site-packages wheel copy.
echo
echo "Resolved import path:"
"${PYTHON_BIN}" -c 'import inspect; import pathlib; import reforge_core; print(pathlib.Path(inspect.getfile(reforge_core)).resolve())'
