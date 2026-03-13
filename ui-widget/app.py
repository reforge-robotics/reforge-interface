from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from flask import Flask, jsonify, request
from flask_cors import CORS
from pathlib import Path
import logging
import shlex
import subprocess
import threading
import uuid


app = Flask(__name__)
# Dev-friendly CORS for local widget pages and remote embeds.
CORS(app)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUN_CONNECT_SCRIPT = PROJECT_ROOT / "run_connect_test.sh"
RUN_CALIBRATE_SCRIPT = PROJECT_ROOT / "run_calibrate.sh"
DEFAULT_FREQ = "200"


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _build_connect_command(params: dict) -> list[str]:
    robot_ip = (params.get("robotIp") or "").strip()
    if not robot_ip:
        raise ValueError("Missing required field: robotIp")

    cmd = [str(RUN_CONNECT_SCRIPT), robot_ip]
    local_ip = (params.get("localIp") or "").strip()
    sdk_token = (params.get("sdkToken") or "").strip()
    robot_id = (params.get("robotId") or "").strip()

    if local_ip:
        cmd.extend(["--local_ip", local_ip])
    if sdk_token:
        cmd.extend(["--sdk_token", sdk_token])
    if robot_id:
        cmd.extend(["--robot_id", robot_id])
    return cmd


def _build_calibrate_command(params: dict) -> list[str]:
    robot_ip = (params.get("robotIp") or "").strip()
    if not robot_ip:
        raise ValueError("Missing required field: robotIp")

    cmd = [str(RUN_CALIBRATE_SCRIPT), robot_ip]
    local_ip = (params.get("localIp") or "").strip()
    sdk_token = (params.get("sdkToken") or "").strip()
    robot_id = (params.get("robotId") or "").strip()
    freq = str(params.get("freq") or DEFAULT_FREQ).strip()
    identify_api_token = (params.get("identifyApiToken") or "").strip()
    reforge_robot_id = (params.get("reforgeRobotId") or "").strip()

    if local_ip:
        cmd.extend(["--local_ip", local_ip])
    if sdk_token:
        cmd.extend(["--sdk_token", sdk_token])
    if robot_id:
        cmd.extend(["--robot_id", robot_id])
    if freq:
        cmd.extend(["--freq", freq])
    if identify_api_token and reforge_robot_id:
        cmd.extend(["--identify", identify_api_token, "--reforge_robot_id", reforge_robot_id])
    return cmd


def _run_process(cmd: list[str]) -> tuple[int, str, str]:
    app.logger.info("Running command: %s", shlex.join(cmd))
    completed = subprocess.run(
        cmd,
        cwd=PROJECT_ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    return completed.returncode, completed.stdout, completed.stderr


@dataclass
class Job:
    id: str
    kind: str
    command: str
    status: str = "queued"
    started_at: str = field(default_factory=_utc_now_iso)
    ended_at: str | None = None
    exit_code: int | None = None
    stdout: str = ""
    stderr: str = ""
    phase: str | None = None
    error: str | None = None


state_lock = threading.Lock()
jobs: dict[str, Job] = {}
latest_job_ids = {
    "connect": None,
    "calibrate": None,
}

# Minimal in-memory robot state used by the widget.
state = {
    "robotId": "",
    "vibrationEnabled": True,
    "calibrating": False,
    "connecting": False,
}


def _serialize_job(job: Job) -> dict:
    return {
        "id": job.id,
        "kind": job.kind,
        "command": job.command,
        "status": job.status,
        "startedAt": job.started_at,
        "endedAt": job.ended_at,
        "exitCode": job.exit_code,
        "stdout": job.stdout,
        "stderr": job.stderr,
        "phase": job.phase,
        "error": job.error,
    }


def _latest_job(kind: str) -> Job | None:
    job_id = latest_job_ids.get(kind)
    if not job_id:
        return None
    return jobs.get(job_id)


def _job_in_progress(kind: str) -> bool:
    latest = _latest_job(kind)
    return bool(latest and latest.status in {"queued", "running"})


def _start_connect_job(params: dict) -> Job:
    connect_cmd = _build_connect_command(params)
    connect_job = Job(
        id=str(uuid.uuid4()),
        kind="connect",
        command=shlex.join(connect_cmd),
        status="queued",
    )
    jobs[connect_job.id] = connect_job
    latest_job_ids["connect"] = connect_job.id
    app.logger.info("Queued connect job=%s", connect_job.id)

    def worker() -> None:
        with state_lock:
            state["connecting"] = True
            connect_job.status = "running"
            connect_job.phase = "connect_test"
            app.logger.info("Connect job=%s status=running phase=connect_test", connect_job.id)

        code, out, err = _run_process(connect_cmd)
        with state_lock:
            state["connecting"] = False
            connect_job.exit_code = code
            connect_job.stdout = out
            connect_job.stderr = err
            connect_job.ended_at = _utc_now_iso()
            connect_job.status = "succeeded" if code == 0 else "failed"
            app.logger.info(
                "Connect job=%s status=%s exit_code=%s",
                connect_job.id,
                connect_job.status,
                code,
            )
            if code != 0 and err:
                app.logger.error("Connect job=%s stderr:\n%s", connect_job.id, err)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return connect_job


def _start_calibrate_job(params: dict) -> Job:
    connect_cmd = _build_connect_command(params)
    calibrate_cmd = _build_calibrate_command(params)

    calibrate_job = Job(
        id=str(uuid.uuid4()),
        kind="calibrate",
        command=f"{shlex.join(connect_cmd)} && {shlex.join(calibrate_cmd)}",
        status="queued",
    )
    jobs[calibrate_job.id] = calibrate_job
    latest_job_ids["calibrate"] = calibrate_job.id
    app.logger.info("Queued calibrate job=%s", calibrate_job.id)

    def worker() -> None:
        with state_lock:
            state["connecting"] = True
            state["calibrating"] = True
            calibrate_job.status = "running"
            calibrate_job.phase = "connect_test"
            app.logger.info(
                "Calibrate job=%s status=running phase=connect_test",
                calibrate_job.id,
            )

        connect_code, connect_out, connect_err = _run_process(connect_cmd)
        if connect_code != 0:
            with state_lock:
                state["connecting"] = False
                state["calibrating"] = False
                calibrate_job.exit_code = connect_code
                calibrate_job.stdout = connect_out
                calibrate_job.stderr = connect_err
                calibrate_job.ended_at = _utc_now_iso()
                calibrate_job.phase = "connect_test"
                calibrate_job.error = "Connect test failed."
                calibrate_job.status = "failed"
                app.logger.error(
                    "Calibrate job=%s failed during connect_test exit_code=%s",
                    calibrate_job.id,
                    connect_code,
                )
                if connect_err:
                    app.logger.error("Calibrate job=%s connect stderr:\n%s", calibrate_job.id, connect_err)
            return

        with state_lock:
            state["connecting"] = False
            calibrate_job.phase = "calibrate"
            app.logger.info("Calibrate job=%s phase=calibrate", calibrate_job.id)

        calibrate_code, calibrate_out, calibrate_err = _run_process(calibrate_cmd)
        with state_lock:
            state["calibrating"] = False
            calibrate_job.exit_code = calibrate_code
            calibrate_job.stdout = f"=== Connect Test ===\n{connect_out}\n=== Calibration ===\n{calibrate_out}"
            calibrate_job.stderr = f"=== Connect Test ===\n{connect_err}\n=== Calibration ===\n{calibrate_err}"
            calibrate_job.ended_at = _utc_now_iso()
            calibrate_job.phase = "calibrate"
            calibrate_job.status = "succeeded" if calibrate_code == 0 else "failed"
            if calibrate_code != 0:
                calibrate_job.error = "Calibration failed."
            app.logger.info(
                "Calibrate job=%s status=%s exit_code=%s",
                calibrate_job.id,
                calibrate_job.status,
                calibrate_code,
            )
            if calibrate_code != 0 and calibrate_err:
                app.logger.error("Calibrate job=%s calibration stderr:\n%s", calibrate_job.id, calibrate_err)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    return calibrate_job


# Widget initialization: fetch current robot state.
@app.get("/api/reforge/state")
def get_state():
    return jsonify(
        {
            "robotId": state["robotId"],
            "vibrationEnabled": state["vibrationEnabled"],
            "registered": bool(state["robotId"]),
            "connecting": bool(state["connecting"]),
            "calibrating": bool(state["calibrating"]),
        }
    )


# Toggle vibration control.
@app.post("/api/reforge/vibration")
def set_vibration():
    body = request.get_json(silent=True) or {}
    state["vibrationEnabled"] = bool(body.get("enabled"))
    return jsonify({"ok": True})


# Update robot ID.
@app.post("/api/reforge/robot-id")
def set_robot_id():
    body = request.get_json(silent=True) or {}
    state["robotId"] = body.get("id", "") if isinstance(body.get("id"), str) else ""
    return jsonify({"ok": True})


@app.post("/api/reforge/connect/start")
def connect_start():
    body = request.get_json(silent=True) or {}
    if _job_in_progress("connect"):
        return jsonify({"ok": False, "error": "Connect test is already running."}), 409
    if _job_in_progress("calibrate"):
        return jsonify({"ok": False, "error": "Calibration job is already running."}), 409

    try:
        job = _start_connect_job(body)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    return jsonify({"ok": True, "jobId": job.id})


@app.get("/api/reforge/connect/status")
def connect_status():
    latest = _latest_job("connect")
    if latest is None:
        return jsonify({"connecting": bool(state["connecting"]), "status": "idle"})
    return jsonify(
        {
            "connecting": bool(state["connecting"]),
            "status": latest.status,
            "phase": latest.phase,
            "jobId": latest.id,
            "error": latest.error,
            "exitCode": latest.exit_code,
            "stdout": latest.stdout,
            "stderr": latest.stderr,
        }
    )


# Start calibration by running connect test first, then calibration.
@app.post("/api/reforge/calibrate/start")
def calibrate_start():
    body = request.get_json(silent=True) or {}
    if _job_in_progress("calibrate"):
        return jsonify({"ok": False, "error": "Calibration job is already running."}), 409

    try:
        job = _start_calibrate_job(body)
    except ValueError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 400

    return jsonify({"ok": True, "jobId": job.id})


# Poll calibration status.
@app.get("/api/reforge/calibrate/status")
def calibrate_status():
    latest = _latest_job("calibrate")
    if latest is None:
        return jsonify({"calibrating": bool(state["calibrating"]), "status": "idle"})
    return jsonify(
        {
            "calibrating": bool(state["calibrating"]),
            "connecting": bool(state["connecting"]),
            "status": latest.status,
            "phase": latest.phase,
            "jobId": latest.id,
            "error": latest.error,
            "exitCode": latest.exit_code,
            "stdout": latest.stdout,
            "stderr": latest.stderr,
        }
    )


@app.get("/api/reforge/jobs/<job_id>")
def get_job(job_id: str):
    job = jobs.get(job_id)
    if job is None:
        return jsonify({"ok": False, "error": "Job not found."}), 404
    return jsonify({"ok": True, "job": _serialize_job(job)})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
