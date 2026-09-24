# Reforge robot interface

This repository contains one generated, public robot adapter package and its
reviewed configuration, robot description, mesh, license, and example assets.
The included adapter is a software integration bundle; this documentation does
not claim that a particular robot, controller, or hardware setup is supported.

## Target status

Every Trossen URDF STL and PNG reference is bundled. The texture is pinned from `TrossenRobotics/trossen_arm_description@21d8b360` under the bundled BSD notice; asset completeness does not claim live-hardware qualification.

## Install

Run these commands from the repository root:

```bash
python -m pip install -r src/robot/requirements.txt
python -m pip install -e src/robot
```

Run commands from the repository root. Calibration and identification data
are written below the repository-relative `src/robot/data/` directory, and
generated model artifacts are written below `src/robot/models/`. Container
helpers mount both directories at their matching paths inside the image.

## Offline help

After installation, inspect the available command routes without connecting to
hardware or running a trajectory:

```bash
python -m robot.run --help
```

The package's validation workflow performs source, package-data, and offline
smoke checks. Hardware connection, calibration, and controller compatibility
remain deployment-specific qualification work.

Optional native/C++ examples may require a separately installed native SDK and
its documented build prerequisites; they are not required for the Python
package's offline checks.
