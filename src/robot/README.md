# Public robot package

This directory is an installable `robot` package containing one selected
adapter and its public resources. Install it from this directory with:

The Yaskawa NEX7 adapter and Reforge-owned ACU bridge client are an unqualified customer-review candidate. The bundled URDF is an inert test model, not an approved NEX7 description; production publication and hardware motion remain disabled.

```bash
python -m pip install -r requirements.txt
python -m pip install -e .
```

Run commands from the repository root. The adapter writes calibration and
identification data below `src/robot/data/` and generated model artifacts below
`src/robot/models/`. Copy configuration files to another writable location
when a run needs local changes; do not edit installed package resources.

For offline command discovery, use:

```bash
python -m robot.run --help
```

This package and its example assets do not by themselves claim hardware or
vendor compatibility. Hardware connection and motion qualification require the
selected adapter's separately reviewed prerequisites.
