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

mkdir -p "$DATA_DIR"

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
  -v "$DATA_DIR:/control-box-bot/reforge-interface/src/robot/data" \
  "$IMAGE_NAME" \
  "${cmd_args[@]}"
