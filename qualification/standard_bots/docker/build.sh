#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /path/to/extracted-run-39-dist" >&2
  exit 2
fi

artifact_dir=$(realpath "$1")
repo_root=$(realpath "$(dirname "$0")/../../..")
sdk_wheel="${artifact_dir}/standardbots-2.20260731.17-py3-none-any.whl"
sdk_sha256="0f3e4f7c327f157a9f47c07ecd7dbaa0b95b284cbbcfd008ca59c26142d58320"

if [[ ! -f "$sdk_wheel" ]]; then
  echo "Missing pinned Standard Bots SDK wheel: $sdk_wheel" >&2
  exit 1
fi

echo "${sdk_sha256}  ${sdk_wheel}" | sha256sum --check --strict

docker buildx build --load --platform linux/amd64 \
  --build-context "reforge_artifacts=${artifact_dir}" \
  --file "${repo_root}/qualification/standard_bots/Dockerfile" \
  --tag reforge-standard-bots-qualification:2.0.9-908-standardbots-2.20260731.17 \
  "${repo_root}"
