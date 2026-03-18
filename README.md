# reforge-interface

Robot interface code for integrating external robot SDKs with Reforge calibration and identification workflows.

## Overview

`reforge-interface` provides a robot adapter layer that lets you:

- connect your robot SDK to a consistent interface,
- run calibration data collection,
- run identification / fine-tuning against Reforge Cloud,
- and run vibration tests for controller evaluation.

Core implementation lives in `src/robot/`.

## Repository Layout

```text
reforge-interface/
├── README.md
├── requirements.txt
├── pyproject.toml
└── src/
    └── robot/
        ├── run.py                 # CLI entrypoint
        ├── robot_interface.py     # SDK integration target
        ├── robot_base.py          # abstract base + defaults
        ├── ros_manager.py         # optional ROS trajectory publisher
        ├── urdf/                  # place robot URDF files here
        └── data/                  # calibration outputs
```

## Quick Start

1. Add your robot URDF to `src/robot/urdf/`.
2. Add your robot SDK dependency to `requirements.txt` (or vendor it into the repo).
3. Integrate the SDK in `src/robot/robot_interface.py` by replacing all `# {~.~}` markers.
4. Validate the integration.

```bash
python3 -m py_compile src/robot/robot_interface.py
rg "\{~\.~\}" src/robot/robot_interface.py
```
5. Install dependencies (venv or docker [on the robot control box]).

```bash
python3 -m venv .venv
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash # Your ROS installation
pip install -r requirements.txt
pip install -e .
```
or 
```bash
ssh <user>@<control_box_ip>
git clone https://github.com/reforge-robotics/reforge-interface.git
cd reforge-interface/
git checkout standardbots-sdk
docker build -t reforge-interface:latest .
```
or build without cache
```bash
docker build --no-cache --pull -t reforge-interface:latest .
```

## 5-Minute Happy Path

Use this when you want a fast first calibration run with minimal options.

```bash
# 1) Add your robot URDF (example filename)
cp /path/to/my_robot.urdf src/robot/urdf/my_robot.urdf

# 2) Integrate SDK in src/robot/robot_interface.py
#    - set URDF_PATH="urdf/my_robot.urdf"
#    - replace all # {~.~} markers

# 3) Validate interface wiring
python3 -m py_compile src/robot/robot_interface.py
rg "\{~\.~\}" src/robot/robot_interface.py

# 4) Install
python3 -m venv .venv
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash # Your ROS installation
pip install -r requirements.txt
pip install -e .

# 5) Test connection (no trajectory execution)
python3 -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>

# docker on the control box
chmod +x docker_scripts/run_connect_test.sh
./docker_scripts/run_connect_test.sh <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>

# 6) Run calibration
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200

# docker on the control box
chmod +x docker_scripts/run_calibrate.sh
./docker_scripts/run_calibrate.sh <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200

# 7) Run identification
python3 -m robot.run identify <api_token> <reforge_robot_id> <data_folder>

# docker on the control box
./docker_scripts/run_identify.sh <api_token> <reforge_robot_id> <data_folder>

# 8) Run vibration test
python3 -m robot.run vibration_test <robot_ip> <data_folder> --sdk_token <token> --robot_id <robot_id>

./docker_scripts/run_vibration_test.sh <robot_ip> <data_folder> --sdk_token <token> --robot_id <robot_id>
```

## Robot Integration Flow

1. Set constants in `src/robot/robot_interface.py`:
- `BOT_ID`, `URDF_PATH`, `ROBOT_MAX_FREQ`
- `HOME_SHOULDER_ANGLE`, `HOME_XYZ`, `HOME_QUAT`, `HOME_JOINTS`
- `IS_DEGREES`, `DATA_LOCATION_PREFIX`

2. Add SDK client setup in `RobotInterface.__init__`.
3. Implement required SDK-bound methods:
- `__get_joint_positions`
- `__get_tcp_pose`
- `move_to_joint`
- `move_to_pose`
- `publish_and_record_joint_positions`

4. Keep units and schemas consistent:
- internal joint units in radians,
- pose format `[x, y, z, qx, qy, qz, qw]`,
- data log keys required by calibration pipeline.

See `src/robot/README.md` for the detailed contract.

## CLI Usage

Use module invocation after editable install:

```bash
python3 -m robot.run --help
```

### Connection test

```bash
python3 -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>
```
or 
```bash
./docker_scripts/run_connect_test.sh <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>
```

### Calibration

```bash
python3 -m robot.run calibrate <robot_ip> --robot_id <robot_id> --freq 200
```
or 
```bash
./docker_scripts/run_calibrate.sh <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200
```

Optional cloud actions immediately after calibration:

```bash
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200 --identify <api_token> --reforge_robot_id <reforge_robot_id>
python3 -m robot.run calibrate <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200 --fine_tune <api_token> --reforge_robot_id <reforge_robot_id>
```
or 
```bash
./docker_scripts/run_calibrate.sh <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200 --identify <api_token> --reforge_robot_id <reforge_robot_id>
./docker_scripts/run_calibrate.sh <robot_ip> --sdk_token <token> --robot_id <robot_id> --freq 200 --fine_tune <api_token> --reforge_robot_id <reforge_robot_id>
```

### Identification (manual)

```bash
python3 -m robot.run identify <api_token> <reforge_robot_id> <data_folder>
```
or 
```bash
./docker_scripts/run_identify.sh <api_token> <reforge_robot_id> <data_folder>
```

### Fine-tuning (manual)

```bash
python3 -m robot.run fine_tune <api_token> <reforge_robot_id> <data_folder>
```
or 
```bash
./docker_scripts/run_fine_tune.sh <api_token> <reforge_robot_id> <data_folder>
```

### Vibration test

```bash
python3 -m robot.run vibration_test <robot_ip> <data_folder> --robot_id <robot_id>
```
or 
```bash
./docker_scripts/run_vibration_test.sh <robot_ip> <data_folder> --robot_id <robot_id>
```

## Widget Integration and Deployment

This repo includes a web component widget in `ui-widget/` that can run:

- connect test (`connect_test`)
- calibration (`calibrate`)
- optional identification right after calibration (`--identify ... --reforge_robot_id ...`)

### Pre-implemented files in this repo

- `ui-widget/reforge-robotics-widget.js`
  - UI web component (collapsible panel, Robot ID/IP fields, connect and calibrate controls).
- `ui-widget/reforge-robotics-widget.auto.js`
  - Auto-mount integration script that reads `<script data-*>` configuration and calls API endpoints.
- `ui-widget/app.py`
  - Flask API implementation with background job execution and status polling endpoints.
- `run_connect_test.sh`
  - Wrapper for `python3 -m robot.run connect_test ...` (returns non-zero if connect output indicates failure).
- `run_calibrate.sh`
  - Wrapper for `python3 -m robot.run calibrate ...`.

### Option A: Embed widget from local JS files (host PC / robot)

Serve `ui-widget/` as static files from your host machine or robot control box, then include:

```html
<script
  src="/ui-widget/reforge-robotics-widget.auto.js"
  data-base-url="http://<api-host>:8080"
  data-robot-ip="<robot_ip>"
  data-local-ip="<optional_local_ip>"
  data-sdk-token="<optional_sdk_token>"
  data-robot-id="<optional_robot_id>"
  data-reforge-identify-api-token="<optional_reforge_api_token>"
  data-reforge-robot-id="<optional_reforge_robot_id>"
  data-freq="200"
></script>
```

Use this option when the widget JS should be bundled and served from your own environment.

### Option B: Embed widget directly from `app.reforgerobotics.com`

```html
<script
  src="https://app.reforgerobotics.com/ui-widget/reforge-robotics-widget.auto.js"
  data-base-url="http://<api-host>:8080"
  data-robot-ip="<robot_ip>"
  data-local-ip="<optional_local_ip>"
  data-sdk-token="<optional_sdk_token>"
  data-robot-id="<optional_robot_id>"
  data-reforge-identify-api-token="<optional_reforge_api_token>"
  data-reforge-robot-id="<optional_reforge_robot_id>"
  data-freq="200"
></script>
```

Use this option when you want to consume the hosted widget script directly.

### Attribute reference (`<script data-...>`)

- `data-base-url` (required): Base URL of your API server that implements `/api/reforge/*`.
- `data-robot-ip` (required): Robot IP for `connect_test` and `calibrate`.
- `data-local-ip` (optional): Passed as `--local_ip` when non-empty.
- `data-sdk-token` (optional): Passed as `--sdk_token` when non-empty.
- `data-robot-id` (optional): Passed as `--robot_id` when non-empty.
- `data-freq` (optional): Calibration sampling frequency (`--freq`), default is `200`.
- `data-reforge-identify-api-token` (optional): If set with `data-reforge-robot-id`, runs identify after calibration.
- `data-reforge-robot-id` (optional): Target Reforge robot ID for identify call.
- `data-manage-url` (optional): Override Reforge app manage/register URL.
- `data-mount` (optional): CSS selector to mount widget into (default `body`).
- `data-poll-interval-ms` (optional): Poll interval for status endpoints (default `1500`).
- `data-poll-timeout-ms` (optional): Poll timeout for long-running jobs (default `300000`).

Note: Leaving optional values blank is supported. The backend only forwards non-empty options so `robot.run` defaults are used.

### API endpoints to implement (already pre-implemented in `ui-widget/app.py`)

The auto widget expects these endpoints:

#### 1) `GET /api/reforge/state`

Purpose:
- Initial widget state.

Response:
```json
{
  "robotId": "",
  "vibrationEnabled": true,
  "registered": false,
  "connecting": false,
  "calibrating": false
}
```

#### 2) `POST /api/reforge/vibration`

Purpose:
- Persist vibration toggle from the UI.

Request:
```json
{ "enabled": true }
```

Response:
```json
{ "ok": true }
```

#### 3) `POST /api/reforge/robot-id`

Purpose:
- Persist Robot ID entered in UI.

Request:
```json
{ "id": "<uuid>" }
```

Response:
```json
{ "ok": true }
```

#### 4) `POST /api/reforge/connect/start`

Purpose:
- Start background connect test job.

Request:
```json
{
  "robotIp": "<robot_ip>",
  "localIp": "<optional_local_ip>",
  "sdkToken": "<optional_sdk_token>",
  "robotId": "<optional_robot_id>"
}
```

Response:
```json
{ "ok": true, "jobId": "<uuid>" }
```

#### 5) `GET /api/reforge/connect/status`

Purpose:
- Poll connect test progress and result.

Response (example):
```json
{
  "connecting": false,
  "status": "succeeded",
  "phase": "connect_test",
  "jobId": "<uuid>",
  "error": null,
  "exitCode": 0,
  "stdout": "...",
  "stderr": ""
}
```

#### 6) `POST /api/reforge/calibrate/start`

Purpose:
- Start calibration workflow in background.
- In this repo, implementation runs connect test first, then calibration.

Request:
```json
{
  "robotIp": "<robot_ip>",
  "localIp": "<optional_local_ip>",
  "sdkToken": "<optional_sdk_token>",
  "robotId": "<optional_robot_id>",
  "freq": "200",
  "identifyApiToken": "<optional_reforge_api_token>",
  "reforgeRobotId": "<optional_reforge_robot_id>"
}
```

Response:
```json
{ "ok": true, "jobId": "<uuid>" }
```

#### 7) `GET /api/reforge/calibrate/status`

Purpose:
- Poll calibration workflow status.

Response (example):
```json
{
  "calibrating": true,
  "connecting": false,
  "status": "running",
  "phase": "calibrate",
  "jobId": "<uuid>",
  "error": null,
  "exitCode": null,
  "stdout": "",
  "stderr": ""
}
```

#### 8) `GET /api/reforge/jobs/<job_id>`

Purpose:
- Fetch full metadata for any job id.

### Deploying the API server (`ui-widget/app.py`)

1. Install dependencies and editable package:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .
```

2. Ensure scripts are executable:

```bash
chmod +x run_connect_test.sh run_calibrate.sh
```

3. Start API:

```bash
python3 -m ui-widget.app
```

4. Optional: enable verbose server command logging (includes `Running command: ...` lines):

```bash
REFORGE_WIDGET_VERBOSE=1 python3 -m ui-widget.app
```

By default (`REFORGE_WIDGET_VERBOSE` unset/false), command-level debug prints are suppressed.

5. Set `data-base-url` in your embed snippet to this API host and port.

6. Production deployment recommendations:

- Run Flask behind a production WSGI server (for example gunicorn) and reverse proxy.
- Serve over HTTPS.
- Restrict CORS to your allowed web origins.
- Inject secrets (`sdk_token`, API tokens) from server-side config when possible.
- Keep `run_connect_test.sh` and `run_calibrate.sh` co-located with repo so `app.py` can execute them.

## Validation Checklist

- `python3 -m py_compile src/robot/robot_interface.py` passes.
- `rg "\{~\.~\}" src/robot/robot_interface.py` returns no matches.
- `URDF_PATH` points to a real file under `src/robot/urdf/`.
- Joint count returned by SDK is compatible with URDF model count.
- Calibration starts without unit, schema, or connection errors.

## Troubleshooting

- SDK import fails:
  - confirm package is installable and import path matches SDK docs.
- Robot does not move:
  - verify control mode, motor enable/unbrake, and any SDK session requirements.
- Motion appears wrong:
  - verify `IS_DEGREES` and conversion boundaries.
- Missing telemetry fields:
  - preserve required output keys with empty values instead of removing keys.

## Additional Documentation

- Robot integration details: `src/robot/README.md`
- Skill-based integration guide: `docs/robot-sdk-skill-user-guide.md`
