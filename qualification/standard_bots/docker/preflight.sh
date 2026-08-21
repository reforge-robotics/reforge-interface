#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 5 ]]; then
  echo "Usage: $0 ENDPOINT ROBOT_ID COMMAND_TOPIC JOINT_STATE_TOPIC IMU_TOPIC [OUTPUT_DIR] [extra args]" >&2
  exit 2
fi

output_dir=${6:-qualification-preflight-output}
mkdir -p "$output_dir"
output_dir=$(realpath "$output_dir")

docker run --rm --network host --env STANDARD_BOTS_SDK_TOKEN \
  --mount "type=bind,src=${output_dir},dst=/evidence" \
  reforge-standard-bots-qualification:2.0.9-908-standardbots-2.20260731.17 \
  standard_bots_qualification_cli --preflight \
    --assets-dir /workspace/src/robot --output-dir /evidence \
    --endpoint "$1" --robot-id "$2" \
    --command-topic "$3" --joint-state-topic "$4" \
    --imu-topic "$5" \
    --control-state-probe /workspace/qualification/standard_bots/standard_bots_control_state_probe \
    "${@:7}"
