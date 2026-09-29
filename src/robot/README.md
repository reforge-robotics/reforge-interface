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
output is missing, the flagged command prompts for each arm's first actuator
joint before generating the models. For Axol, enter `left_s1_0` and
`right_s1_0`. Run the command in an interactive terminal when splitting is
needed. Existing outputs are reused, and calibration loads the arm selected by
`USE_LEFT`.

The Axol configuration selects the right arm by default (`USE_LEFT = False`).
Both split URDFs are currently tracked by Git, so the flagged command reuses
them while they exist, despite the `.gitignore` entries. Generated mesh paths
are relative to each output URDF (for example, `meshes/Base.stl` when the
outputs are in `src/robot/urdf/`). Keep the outputs with their `meshes/` folder.

## Simulator pipeline check

For a short simulator scan, pass the scan options explicitly:

```bash
python -m robot.run calibrate sim --bimanual \
  --nv 1 --nr 1 --axes 1 --first_axis 0 \
  --sine_cycles 13 --maxfreq 5 --freqspace 1 --dwell 0.1
python -m robot.run joint_tracker_performance_validation sim --mode single-speed \
  --urdf-path ./src/robot/urdf/axol-right.urdf
```

Omitting scan options uses the SDK calibration defaults, currently eight angle
samples and four radii. `run.py` does not add simulator-specific scan defaults.
If an arm URDF is missing, the flagged command prompts for the first joints
before the scan begins.

Unflagged `calibrate sim` loads the 14-joint `axol.urdf` without generating
arm models. The current calibration planner assumes one TCP chain with a
world-z base joint, so this full Axol model fails base-joint validation; use
`--bimanual` for a complete simulator calibration.

The validation command writes a tracking JSON and HTML report under
`src/robot/performance_validation/joint_tracker/`. The current Joint Tracker
model directory must include `identification_parameters.csv` alongside its
model JSON files.
