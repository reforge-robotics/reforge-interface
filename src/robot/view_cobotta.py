"""Serve an interactive Viser view of the COBOTTA PRO 900 URDF."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf


URDF_PATH = Path(__file__).with_name("urdf") / "cobotta_pro_900.urdf"


def main() -> None:
    """Start the local COBOTTA viewer and block until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8080, help="Viser port")
    args = parser.parse_args()

    server = viser.ViserServer(port=args.port)
    urdf = yourdfpy.URDF.load(URDF_PATH, mesh_dir=str(URDF_PATH.parent))
    robot = ViserUrdf(server, urdf)
    sliders = []

    for joint_name in robot.get_actuated_joint_names():
        lower, upper = robot.get_actuated_joint_limits()[joint_name]
        slider = server.gui.add_slider(
            joint_name,
            min=-np.pi if lower is None else lower,
            max=np.pi if upper is None else upper,
            step=0.01,
            initial_value=0.0,
        )
        sliders.append(slider)

        @slider.on_update
        def _update_robot(_: viser.GuiEvent) -> None:
            """Apply the current slider values to the displayed robot."""
            robot.update_cfg(np.array([handle.value for handle in sliders]))

    print(f"COBOTTA viewer: http://localhost:{args.port}")
    server.sleep_forever()


if __name__ == "__main__":
    main()
