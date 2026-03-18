#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot_ip> [--local_ip <local_ip>] [--sdk_token <token>] [--robot_id <robot_id>] [--freq <hz>] [--identify <api_token> --reforge_robot_id <id>]"
  exit 1
fi

output="$(python3 -m robot.run calibrate "$@" 2>&1)"
status=$?
printf '%s\n' "$output"

if [[ $status -ne 0 ]]; then
  exit "$status"
fi

# `robot.run calibrate` may catch exceptions and still exit 0.
if grep -q "Traceback (most recent call last)" <<<"$output"; then
  exit 1
fi

if grep -q "Failed to connect" <<<"$output"; then
  exit 1
fi

if grep -q "❌" <<<"$output"; then
  exit 1
fi
