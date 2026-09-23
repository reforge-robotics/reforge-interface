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
MODELS_DIR="${MODELS_DIR:-$(pwd)/src/robot/models}"
CONTAINER_DATA_DIR="/opt/reforge-interface/src/robot/data"
CONTAINER_MODELS_DIR="/opt/reforge-interface/src/robot/models"

mkdir -p -- "$DATA_DIR" "$MODELS_DIR"
DATA_DIR="$(cd -- "$DATA_DIR" && pwd -P)"
MODELS_DIR="$(cd -- "$MODELS_DIR" && pwd -P)"

cmd_args=(python3 -m robot.run calibrate "$robot_ip")

if [[ $# -gt 0 ]]; then
  cmd_args+=("$@")
fi

docker run --net=host --rm --name "$CONTAINER_NAME" \
  -v "$DATA_DIR:$CONTAINER_DATA_DIR" \
  -v "$MODELS_DIR:$CONTAINER_MODELS_DIR" \
  "$IMAGE_NAME" \
  "${cmd_args[@]}"
