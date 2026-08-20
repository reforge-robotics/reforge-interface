#!/usr/bin/env bash
set -euo pipefail

repo_root=$(realpath "$(dirname "$0")/../../..")
output_dir=${1:-"${repo_root}/qualification-output"}
mkdir -p "$output_dir"
docker run --rm --network none \
  --platform linux/amd64 \
  --read-only \
  --tmpfs /tmp \
  --security-opt "seccomp=${repo_root}/qualification/standard_bots/docker/phase-c-no-network-seccomp.json" \
  --mount "type=bind,src=$(realpath "$output_dir"),dst=/evidence" \
  reforge-standard-bots-qualification:2.0.9-908 \
  standard_bots_qualification_cli \
    --assets-dir /workspace/src/robot \
    --output-dir /evidence
