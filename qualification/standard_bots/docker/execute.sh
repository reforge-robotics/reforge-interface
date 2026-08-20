#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 7 ]]; then
  echo "Usage: $0 TRIAL ENDPOINT ROBOT_ID COMMAND_TOPIC JOINT_STATE_TOPIC IMU_TOPIC OUTPUT_DIR" >&2
  exit 2
fi

mkdir -p "$7"
output_dir=$(realpath "$7")
docker run --rm --interactive --tty --network host \
  --env STANDARD_BOTS_SDK_TOKEN \
  --mount "type=bind,src=${output_dir},dst=/evidence" \
  reforge-standard-bots-qualification:2.0.9-908-standardbots-2.20260731.17 \
  standard_bots_qualification_cli --execute --trial "$1" \
    --assets-dir /workspace/src/robot --output-dir /evidence \
    --endpoint "$2" --robot-id "$3" \
    --command-topic "$4" --joint-state-topic "$5" \
    --imu-topic "$6" \
    --control-state-probe /workspace/qualification/standard_bots/standard_bots_control_state_probe
