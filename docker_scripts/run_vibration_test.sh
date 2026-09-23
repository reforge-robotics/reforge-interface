#!/bin/bash
set -euo pipefail

if [[ "${DEBUG:-0}" == "1" ]]; then
  set -x
fi

usage() {
  echo "Usage: $0 <robot_ip> <data_folder> [vibration_test flags]"
  echo "Example: $0 cb2002.sb.app src/robot/data/2026-2-25 --sdk_token <token> --robot_id <id> --freq 200"
}

if [[ $# -lt 2 ]]; then
  usage
  exit 1
fi

robot_ip="$1"
host_data_folder="$2"
shift 2

if [[ ! -d "$host_data_folder" ]]; then
  echo "Data folder does not exist: $host_data_folder"
  exit 1
fi
host_data_folder="$(cd -- "$host_data_folder" && pwd -P)"

IMAGE_NAME="${IMAGE_NAME:-reforge-interface}"
CONTAINER_NAME="${CONTAINER_NAME:-reforge-vibration-test}"
MODELS_DIR="${MODELS_DIR:-$(pwd)/src/robot/models}"
CONTAINER_DATA_DIR="${CONTAINER_DATA_DIR:-/opt/reforge-interface/src/robot/data}"
CONTAINER_MODELS_DIR="/opt/reforge-interface/src/robot/models"

mkdir -p -- "$MODELS_DIR"
MODELS_DIR="$(cd -- "$MODELS_DIR" && pwd -P)"

cmd_args=(python3 -m robot.run vibration_test "$robot_ip" "$CONTAINER_DATA_DIR")
if [[ $# -gt 0 ]]; then
  cmd_args+=("$@")
fi

docker run --net=host --rm --name "$CONTAINER_NAME" \
  -v "$host_data_folder:$CONTAINER_DATA_DIR" \
  -v "$MODELS_DIR:$CONTAINER_MODELS_DIR" \
  "$IMAGE_NAME" \
  "${cmd_args[@]}"
