#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot_ip> [--local_ip <local_ip>] [--sdk_token <token>] [--robot_id <robot_id>]"
  exit 1
fi

output="$(python3 -m robot.run connect_test "$@" 2>&1)"
status=$?
printf '%s\n' "$output"

if [[ $status -ne 0 ]]; then
  exit "$status"
fi

# `robot.run connect_test` currently prints failure text but may still exit 0.
if grep -q "Failed to connect" <<<"$output"; then
  exit 1
fi

if ! grep -q "Successfully connected" <<<"$output"; then
  echo "Connect test did not report a success marker."
  exit 1
fi
