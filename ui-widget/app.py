# app.py
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading

app = Flask(__name__)
# Allow the widget hosted at app.reforgerobotics.com to call this API.
CORS(app, origins=["https://app.reforgerobotics.com"])

# Minimal in-memory robot state used by the widget.
state = {
    "robotId": "3f2504e0-4f89-11d3-9a0c-0305e82c3301",
    "vibrationEnabled": True,
    "calibrating": False,
}

# Widget initialization: fetch current robot state.
@app.get("/api/reforge/state")
def get_state():
    return jsonify({
        "robotId": state["robotId"],
        "vibrationEnabled": state["vibrationEnabled"],
        "registered": bool(state["robotId"])
    })

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

# Start calibration (returns immediately).
@app.post("/api/reforge/calibrate/start")
def calibrate_start():
    state["calibrating"] = True

    # Simulate calibration finishing after 5s.
    def finish():
        state["calibrating"] = False

    threading.Timer(5.0, finish).start()
    return jsonify({"ok": True})

# Poll calibration status.
@app.get("/api/reforge/calibrate/status")
def calibrate_status():
    return jsonify({"calibrating": state["calibrating"]})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)