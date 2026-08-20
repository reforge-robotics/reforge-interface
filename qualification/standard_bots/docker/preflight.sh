#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 5 ]]; then
  echo "Usage: $0 ENDPOINT ROBOT_ID COMMAND_TOPIC JOINT_STATE_TOPIC IMU_TOPIC [extra args]" >&2
  exit 2
fi

docker run --rm --network host --env STANDARD_BOTS_SDK_TOKEN \
  reforge-standard-bots-qualification:2.0.9-908 \
  standard_bots_qualification_cli --preflight \
    --assets-dir /workspace/src/robot \
    --endpoint "$1" --robot-id "$2" \
    --command-topic "$3" --joint-state-topic "$4" \
    --imu-topic "$5" \
    --control-state-probe /workspace/qualification/standard_bots/standard_bots_control_state_probe \
    "${@:6}"
