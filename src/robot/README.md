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

When either arm output is missing, the command prompts for each arm's first
actuator joint (and an end joint if the chain branches), then generates both
arm URDFs before calibration starts. Existing arm outputs are reused. The
calibration runner loads the arm selected by `USE_LEFT`.

The included URDF and configuration files are examples only. They do not
claim compatibility with a particular robot or vendor SDK.
