# New Robot Checklist

As of today, a new robot integration in `src/robot/robot_interface.py` must provide all of the following:

1. **A URDF file**
   - Add the robot URDF under `src/robot/urdf/`.
   - Set `URDF_PATH` to that file.

2. **An importable SDK**
   - Add the SDK dependency in `requirements.txt` or make the SDK importable in the repo.
   - Replace the SDK import placeholder in `robot_interface.py`.

3. **Robot-specific constants**
   - `BOT_ID`
   - `URDF_PATH`
   - `ROBOT_MAX_FREQ`
   - `HOME_SHOULDER_ANGLE`
   - `HOME_XYZ`
   - `HOME_QUAT`
   - `HOME_JOINTS`
   - `HOME_POSE_OVERRIDE` if needed
   - `IS_DEGREES`
   - `DATA_LOCATION_PREFIX`

4. **SDK setup in `RobotInterface.__init__`**
   - Instantiate the SDK client.
   - Enable control / ROS mode if the robot requires it.
   - Unbrake / enable the robot if the robot requires it.

5. **These required methods**
   - `__get_joint_positions(self) -> List`
   - `__get_tcp_pose(self) -> List`
   - `move_to_joint(self, target_joint: Tuple[float, ...]) -> None`
   - `move_to_pose(self, target_quat: List[float], target_xyz: List[float]) -> None`
   - `publish_and_record_joint_positions(...) -> Deque[Dict]`

6. **The required units / shapes**
   - Internal joint values must be in **radians**.
   - TCP pose must be `[x, y, z, qx, qy, qz, qw]`.
   - If the SDK uses degrees, convert only at the SDK boundary with `IS_DEGREES=True`.

7. **The required telemetry output contract**
   - `publish_and_record_joint_positions(...)` must return rows with these keys:
     - `cmd_time`
     - `input_positions`
     - `output_positions`
     - `velocities`
     - `efforts`
     - `imu_time`
     - `linear_acceleration`
     - `angular_velocity`
     - `orientation`
   - If a field is unavailable, keep the key and use an empty value.

8. **Validation before use**
   - `python -m py_compile src/robot/robot_interface.py`
   - `rg "\{~\.~\}" src/robot/robot_interface.py`
   - `python -m robot.run connect_test <robot_ip> --local_ip <local_ip> --sdk_token <token> --robot_id <robot_id>`

Primary sources:
- `src/robot/robot_interface.py`
- `src/robot/README.md`
- `docs/robot-sdk-skill-user-guide.md`
