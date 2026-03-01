#!/bin/bash
set -euo pipefail

if [[ "${DEBUG:-0}" == "1" ]]; then
  set -x
fi

usage() {
  echo "Usage: $0 <robot_ip> [calibrate flags]"
  echo "Example: $0 10.0.0.4:3000 --sdk_token <token> --robot_id <id> --freq 200"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

robot_ip="$1"
shift

IMAGE_NAME="${IMAGE_NAME:-reforge-interface}"
CONTAINER_NAME="${CONTAINER_NAME:-reforge-calibrate}"
DATA_DIR="${DATA_DIR:-$(pwd)/src/robot/data}"

mkdir -p "$DATA_DIR"

cmd_args=(python3 -m robot.run calibrate "$robot_ip")

if [[ $# -gt 0 ]]; then
  cmd_args+=("$@")
fi

docker run --net=host --rm --name "$CONTAINER_NAME" \
  -v "$DATA_DIR:/control-box-bot/reforge-interface/src/robot/data" \
  "$IMAGE_NAME" \
  "${cmd_args[@]}"
