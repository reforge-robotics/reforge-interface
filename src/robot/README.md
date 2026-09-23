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

The included URDF and configuration files are examples only. They do not
claim compatibility with a particular robot or vendor SDK.
