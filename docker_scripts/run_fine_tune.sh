#!/bin/bash
set -euo pipefail

if [[ "${DEBUG:-0}" == "1" ]]; then
  set -x
fi

usage() {
  echo "Usage: $0 <fine_tune_api_token> <robot_id> <data_folder>"
  echo "Example: $0 <token> <robot_id> ./src/robot/data/2026-2-25"
}

if [[ $# -lt 3 ]]; then
  usage
  exit 1
fi

fine_tune_api_token="$1"
robot_id="$2"
host_data_folder="$3"
shift 3

if [[ $# -gt 0 ]]; then
  echo "Unknown argument(s): $*"
  usage
  exit 1
fi

if [[ ! -d "$host_data_folder" ]]; then
  echo "Data folder does not exist: $host_data_folder"
  exit 1
fi

IMAGE_NAME="${IMAGE_NAME:-reforge-interface}"
CONTAINER_NAME="${CONTAINER_NAME:-reforge-fine-tune}"
CONTAINER_DATA_DIR="${CONTAINER_DATA_DIR:-/control-box-bot/reforge-interface/src/robot/data}"

docker run --net=host --rm --name "$CONTAINER_NAME" \
  -v "$host_data_folder:$CONTAINER_DATA_DIR" \
  "$IMAGE_NAME" \
  python3 -m robot.run fine_tune "$fine_tune_api_token" "$robot_id" "$CONTAINER_DATA_DIR"
