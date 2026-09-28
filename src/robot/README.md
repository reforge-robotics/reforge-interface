# Robot Integration Template

Use this package as a starting point for a robot integration. Keep the package
import name `robot`, then adapt `robot_interface.py` to the vendor SDK.

Before opening a change:

1. Add the robot URDF and meshes under the package's `urdf/` directory.
2. Declare the vendor SDK in `requirements.txt`.
3. Set the robot constants and implement the required SDK methods.
4. Adjust the sample KineCal configuration for the robot's end-effector and
   probe links.
5. From the repository root, install the package with:

   ```bash
   pip install -r src/robot/requirements.txt
   pip install -e src/robot
   ```

6. Run `python -m py_compile src/robot/robot_interface.py` and
   `python -m robot.run --help` before testing with hardware.

## Bimanual calibration

Set `BASE_URDF_PATH` in `robot_interface.py` to the unsplit URDF under `urdf/`.
Set `URDF_PATH` to the selected arm's output, and choose the arm with `USE_LEFT`.
Then run:

```bash
python -m robot.run calibrate ROBOT_IP --bimanual
```

Only `calibrate ... --bimanual` generates per-arm URDFs. Without the flag,
calibration loads `BASE_URDF_PATH` directly and does not split it. When a split
output is missing, the flagged command generates both arms before calibration.
Configure `LEFT_FIRST_JOINT` and `RIGHT_FIRST_JOINT` in `robot_interface.py`
for unattended splitting; otherwise the splitter prompts for them. Existing
arm outputs are reused. The flagged calibration runner loads the arm selected
by `USE_LEFT`.

The Axol configuration selects the right arm by default (`USE_LEFT = False`).
Its generated arm URDFs are ignored by Git and are rebuilt from `axol.urdf`
only when `--bimanual` is specified.

## Simulator pipeline check

Run the simulator pipeline with an explicit split request:

```bash
python -m robot.run calibrate sim --bimanual
python -m robot.run joint_tracker_performance_validation sim --mode single-speed \
  --urdf-path ./src/robot/urdf/axol-right.urdf
```

The flagged calibration uses `left_s1_0` and `right_s1_0` as the first joints
when generating missing arm models. Its default scan uses one pose, one radius,
the selected arm's first joint, 13 sine cycles, frequencies from 1 through 4 Hz,
and a 0.1-second dwell. Pass scan options explicitly for a wider run.

Unflagged `calibrate sim` loads the 14-joint `axol.urdf` without generating
arm models. The current calibration planner assumes one TCP chain with a
world-z base joint, so this full Axol model fails base-joint validation; use
`--bimanual` for a complete simulator calibration.

The validation command writes a tracking JSON and HTML report under `src/robot/performance_validation/joint_tracker/`. The current
Joint Tracker model directory must include `identification_parameters.csv`
alongside its model JSON files.
