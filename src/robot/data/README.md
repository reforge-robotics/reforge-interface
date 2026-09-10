# Calibration Data

This directory is intentionally empty in version control. Place raw sweep logs and processed `.pkl` files here when running calibration routines. The data are git-ignored so robot-specific measurements remain local to each machine.

Kinecal data-collection runs write timestamped artifact folders under:

```text
src/robot/data/kinecal/datacol/
```

Each current run folder contains `metadata.json`, including both sockets' joint
rows, and may include `run.log` and `terminal.txt`. Historical run folders may
instead contain the joint rows in `hole_0.csv` and `hole_1.csv`.
