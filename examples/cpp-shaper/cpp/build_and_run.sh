#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: build_and_run.sh [options]

Configure and build the installed-package C++ Shaper example, then run it.
The helper never sets CMAKE_PREFIX_PATH or LD_LIBRARY_PATH.

Options:
  --headless             Suppress GUI windows and save three figures.
  --no-gui               Alias for --headless.
  --output-dir PATH      Save figures and numeric artifacts under PATH.
  --assets-dir PATH      Use a different deterministic asset directory.
  --baseline PATH        Use a different expected-metrics JSON manifest.
  --build-dir PATH       Use a different CMake build directory.
  --help                 Show this help.
EOF
}

die() {
    printf 'error: %s\n' "$1" >&2
    exit 2
}

require_value() {
    local option="$1"
    local count="$2"
    [[ "$count" -ge 2 ]] || die "${option} requires a path"
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build_dir="${script_dir}/build"
assets_dir=""
baseline_manifest=""
output_dir=""
headless=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless|--no-gui)
            headless=1
            shift
            ;;
        --output-dir)
            require_value "$1" "$#"
            output_dir="$2"
            shift 2
            ;;
        --assets-dir)
            require_value "$1" "$#"
            assets_dir="$2"
            shift 2
            ;;
        --baseline)
            require_value "$1" "$#"
            baseline_manifest="$2"
            shift 2
            ;;
        --build-dir)
            require_value "$1" "$#"
            build_dir="$2"
            shift 2
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            die "unknown argument: $1"
            ;;
    esac
done

command -v cmake >/dev/null 2>&1 || die "cmake is required but was not found on PATH"

# A qualified APT install is discoverable through normal system paths and
# ldconfig. Removing developer overrides prevents accidental source-tree use.
unset CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH

cmake -S "$script_dir" -B "$build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir" --parallel

run_arguments=()
if [[ "$headless" == "1" ]]; then
    run_arguments+=(--headless)
fi
if [[ -n "$output_dir" ]]; then
    run_arguments+=(--output-dir "$output_dir")
fi
if [[ -n "$assets_dir" ]]; then
    run_arguments+=(--assets-dir "$assets_dir")
fi
if [[ -n "$baseline_manifest" ]]; then
    run_arguments+=(--baseline "$baseline_manifest")
fi

"${build_dir}/shaper_example_usage" "${run_arguments[@]}"
