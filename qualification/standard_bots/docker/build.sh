#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/extracted-run-39-dist" >&2
  exit 2
fi

artifact_dir=$(realpath "$1")
repo_root=$(realpath "$(dirname "$0")/../../..")
docker buildx build --load --platform linux/amd64 \
  --build-context "reforge_artifacts=${artifact_dir}" \
  --file "${repo_root}/qualification/standard_bots/Dockerfile" \
  --tag reforge-standard-bots-qualification:2.0.9-908 \
  "${repo_root}"
