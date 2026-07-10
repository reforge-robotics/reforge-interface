# Calibration Data

This directory is intentionally empty in version control. Place raw sweep logs and processed `.pkl` files here when running calibration routines. The data are git-ignored so robot-specific measurements remain local to each machine.

Kinecal data-collection runs write timestamped artifact folders under:

```text
src/robot/data/kinecal/datacol/
```

Each run folder may include `run.log`, `terminal.txt`, `metadata.json`,
`hole_0.csv`, and `hole_1.csv`.
