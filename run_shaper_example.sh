#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${1:-${script_dir}/build/standard-bots-shaper}"
model_dir="${2:-${script_dir}/src/robot/models/current/shaper}"
urdf="${3:-${script_dir}/src/robot/urdf/modelone.urdf}"

cmake -S "${script_dir}/src/robot/example_usage/shaper_cpp" -B "${build_dir}"
cmake --build "${build_dir}" --parallel
"${build_dir}/standard_bots_shaper_example" "${model_dir}" "${urdf}"
