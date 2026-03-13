#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <robot_ip> [--local_ip <local_ip>] [--sdk_token <token>] [--robot_id <robot_id>] [--freq <hz>] [--identify <api_token> --reforge_robot_id <id>]"
  exit 1
fi

python3 -m robot.run calibrate "$@"
