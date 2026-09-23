# Runtime data and model locations

Run commands from the repository root. Calibration and identification outputs
belong in the repository-relative, writable directory:

```text
src/robot/data/
```

Generated model artifacts are persisted in:

```text
src/robot/models/
```

Container helpers mount both directories at these same paths inside the
container.
