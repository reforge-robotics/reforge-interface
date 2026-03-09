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

IMAGE_NAME="${IMAGE_NAME:-reforge-interface:latest}"
CONTAINER_NAME="${CONTAINER_NAME:-reforge-calibrate}"
DATA_DIR="${DATA_DIR:-$(pwd)/src/robot/data}"
DEFAULT_CYCLONEDDS_PATH="/etc/standardbots/configuration/cyclonedds.xml"
CYCLONEDDS_URI="${CYCLONEDDS_URI:-}"

normalize_cyclonedds() {
  local input="$1"
  local path=""
  local uri=""
  if [[ "$input" == file://* ]]; then
    path="${input#file://}"
    uri="$input"
  else
    path="$input"
    uri="file://$input"
  fi
  printf '%s\n%s\n' "$path" "$uri"
}

mkdir -p "$DATA_DIR"

cmd_args=(python3 -m robot.run calibrate "$robot_ip")

if [[ $# -gt 0 ]]; then
  cmd_args+=("$@")
fi

docker_args=(
  --net=host
  --rm
  --name "$CONTAINER_NAME"
  -v "$DATA_DIR:/control-box-bot/reforge-interface/src/robot/data"
)

cyclonedds_input="$CYCLONEDDS_URI"
if [[ -z "$cyclonedds_input" && -f "$DEFAULT_CYCLONEDDS_PATH" ]]; then
  cyclonedds_input="$DEFAULT_CYCLONEDDS_PATH"
fi

if [[ -n "$cyclonedds_input" ]]; then
  mapfile -t cyclonedds_cfg < <(normalize_cyclonedds "$cyclonedds_input")
  cyclonedds_path="${cyclonedds_cfg[0]}"
  cyclonedds_uri="${cyclonedds_cfg[1]}"
  if [[ -f "$cyclonedds_path" ]]; then
    cyclonedds_dir="$(dirname "$cyclonedds_path")"
    docker_args+=(
      -v "$cyclonedds_dir:$cyclonedds_dir:ro"
      -e "CYCLONEDDS_URI=$cyclonedds_uri"
    )
  else
    echo "Warning: CYCLONEDDS config not found on host: $cyclonedds_input"
    echo "Calibration will run without explicit CYCLONEDDS_URI."
  fi
else
  echo "Info: CYCLONEDDS_URI is not set and no default config was found."
  echo "Calibration will run with CycloneDDS defaults."
fi

docker run "${docker_args[@]}" \
  "$IMAGE_NAME" \
  "${cmd_args[@]}"
