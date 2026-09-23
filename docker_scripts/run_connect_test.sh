#!/bin/bash
set -euo pipefail

if [[ "${DEBUG:-0}" == "1" ]]; then
  set -x
fi

usage() {
  echo "Usage: $0 <robot_ip> [--local_ip <local_ip>] [--sdk_token <sdk_token>] [--robot_id <robot_id>]"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

robot_ip="$1"
shift

local_ip=""
sdk_token=""
robot_id=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local_ip)
      [[ $# -ge 2 ]] || { usage; exit 1; }
      local_ip="$2"
      shift 2
      ;;
    --sdk_token)
      [[ $# -ge 2 ]] || { usage; exit 1; }
      sdk_token="$2"
      shift 2
      ;;
    --robot_id)
      [[ $# -ge 2 ]] || { usage; exit 1; }
      robot_id="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

IMAGE_NAME="${IMAGE_NAME:-reforge-interface}"
CONTAINER_NAME="${CONTAINER_NAME:-reforge-connect-test}"
DATA_DIR="${DATA_DIR:-$(pwd)/src/robot/data}"
MODELS_DIR="${MODELS_DIR:-$(pwd)/src/robot/models}"
CONTAINER_DATA_DIR="/opt/reforge-interface/src/robot/data"
CONTAINER_MODELS_DIR="/opt/reforge-interface/src/robot/models"

mkdir -p -- "$DATA_DIR" "$MODELS_DIR"
DATA_DIR="$(cd -- "$DATA_DIR" && pwd -P)"
MODELS_DIR="$(cd -- "$MODELS_DIR" && pwd -P)"

cmd_args=(python3 -m robot.run connect_test "$robot_ip")
if [[ -n "$local_ip" ]]; then
  cmd_args+=(--local_ip "$local_ip")
fi
if [[ -n "$sdk_token" ]]; then
  cmd_args+=(--sdk_token "$sdk_token")
fi
if [[ -n "$robot_id" ]]; then
  cmd_args+=(--robot_id "$robot_id")
fi

docker run --net=host --rm --name "$CONTAINER_NAME" \
  -v "$DATA_DIR:$CONTAINER_DATA_DIR" \
  -v "$MODELS_DIR:$CONTAINER_MODELS_DIR" \
  "$IMAGE_NAME" \
  "${cmd_args[@]}"
