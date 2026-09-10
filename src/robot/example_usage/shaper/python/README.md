# Python Covalent Shaper Example

Run the hardware-free Python example from the repository root:

```bash
source .venv/bin/activate
python src/robot/example_usage/shaper/python/shaper_example_usage.py
```

The example demonstrates the four frozen controller modes and opens the two
interactive figures described in the Covalent Shaper documentation. It does
not connect to or command a robot.

The colocated `axis0_model.pt` preserves the Phase 1 legacy Python-backend
baseline. The shared `../assets/` directory contains the frozen three-axis
native bundle and matching URDF that the Python and C++ examples will consume
together when cross-language parity is implemented.
