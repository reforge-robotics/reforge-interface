# Public robot package

This directory is an installable `robot` package containing one selected
adapter and its public resources. Install it from this directory with:

The Yaskawa NEX7 adapter, Reforge-owned ACU bridge client, and Yaskawa-supplied NEX07C00 model are an unqualified customer-review candidate. Public redistribution approval for the model was reported by the user on 2026-09-28. Production publication and hardware motion remain disabled pending controller/model and trajectory qualification.

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
