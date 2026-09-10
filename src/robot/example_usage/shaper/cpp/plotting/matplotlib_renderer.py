"""Render the C++ Shaper example's CSV plot data with Matplotlib."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
import os
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

if TYPE_CHECKING:
    from matplotlib.axes import Axes


PlotData = dict[str, dict[str, dict[str, list[tuple[float, float]]]]]

LABELS = {
    "Example 1": "Example 1: Shape a full known trajectory",
    "Example 2": "Example 2: Shape the final part of a known trajectory",
    "Example 3": "Example 3: Shape a known trajectory in fixed windows",
    "Example 4": "Example 4: Shape a sample-by-sample stream",
}
STYLE = {
    "Desired command": ("black", "-", 1.8),
    "Slower desired command": ("tab:green", "-.", 1.5),
    "Baseline response": ("tab:gray", "-", 1.5),
    "Slower response": ("tab:green", "-.", 1.5),
    "Example 1": ("tab:blue", "-", 1.8),
    "Example 2": ("tab:orange", "-.", 2.0),
    "Example 3": ("tab:cyan", ":", 2.3),
    "Example 4": ("tab:purple", "--", 2.2),
}


def _load_plot_data(path: Path) -> PlotData:
    """Load the command and response series from the C++ plot CSV.

    Args:
        path: CSV file emitted by the C++ example.

    Returns:
        Plot data grouped by category, series, and field.
    """

    data: PlotData = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    with path.open(newline="", encoding="utf-8") as csv_file:
        for row in csv.DictReader(csv_file):
            data[row["category"]][row["series"]][row["field"]].append(
                (float(row["time_s"]), float(row["value"]))
            )
    return data


def _draw(
    axis: Axes,
    data: PlotData,
    category: str,
    series: str,
    field: str,
    label: str | None = None,
) -> None:
    """Draw one named series on a Matplotlib axis.

    Args:
        axis: Destination Matplotlib axis.
        data: Loaded command and response series.
        category: CSV category containing the series.
        series: Series name and style key.
        field: Numeric field to draw.
        label: Optional legend label override.
    """

    values = data[category][series][field]
    color, linestyle, linewidth = STYLE[series]
    axis.plot(
        [value[0] for value in values],
        [value[1] for value in values],
        label=label or LABELS.get(series, series),
        color=color,
        linestyle=linestyle,
        linewidth=linewidth,
    )


def _parse_flag(value: str) -> bool:
    """Parse the C++ renderer's numeric boolean argument.

    Args:
        value: Either ``0`` or ``1``.

    Returns:
        Whether the flag is enabled.

    Raises:
        argparse.ArgumentTypeError: If value is not ``0`` or ``1``.
    """

    if value not in {"0", "1"}:
        raise argparse.ArgumentTypeError("flag must be 0 or 1")
    return value == "1"


def main(argv: Sequence[str] | None = None) -> None:
    """Render interactive or headless figures from a C++ plot-data CSV.

    Args:
        argv: Optional command-line arguments, excluding the program name.
    """

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("headless", type=_parse_flag)
    parser.add_argument("save", type=_parse_flag)
    parser.add_argument("residual_start_s", type=float)
    args = parser.parse_args(argv)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(args.out_dir / ".matplotlib-cache"))
    import matplotlib

    if args.headless:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = _load_plot_data(args.csv_path)
    fig1, axes1 = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
    fields = (
        ("position", "Position [rad]"),
        ("velocity", "Velocity [rad/s]"),
        ("acceleration", "Acceleration [rad/s^2]"),
    )
    for axis, (field, ylabel) in zip(axes1, fields):
        for series in (
            "Desired command",
            "Example 1",
            "Example 2",
            "Example 3",
            "Example 4",
        ):
            _draw(axis, data, "command", series, field)
        axis.axvline(
            args.residual_start_s,
            color="tab:purple",
            linestyle=":",
            label="Residual Shaper starts" if field == "position" else "_nolegend_",
        )
        axis.set_ylabel(ylabel)
        axis.grid(True)
    axes1[0].legend(loc="best")
    axes1[-1].set_xlabel("Time [s]")
    fig1.tight_layout()

    fig2, axes2 = plt.subplots(2, 1, figsize=(10, 7), sharex=False)
    _draw(
        axes2[0],
        data,
        "command",
        "Desired command",
        "position",
        "Baseline command: 0.2 s move",
    )
    _draw(
        axes2[0],
        data,
        "command",
        "Slower desired command",
        "position",
        "Comparison command: 0.6 s move",
    )
    axes2[0].set_ylabel("Command [rad]")
    axes2[0].legend(loc="best")
    axes2[0].grid(True)
    response_labels = {
        "Baseline response": "Baseline response: unshaped 0.2 s move",
        "Slower response": "Comparison response: unshaped 0.6 s move",
    }
    for series in (
        "Baseline response",
        "Slower response",
        "Example 1",
        "Example 2",
        "Example 3",
        "Example 4",
    ):
        _draw(
            axes2[1],
            data,
            "response",
            series,
            "response",
            response_labels.get(series),
        )
    axes2[1].axvline(
        args.residual_start_s,
        color="tab:purple",
        linestyle=":",
        label="Residual Shaper starts",
    )
    axes2[1].axvline(
        0.2, color="tab:red", linestyle=":", label="End of point-to-point move"
    )
    axes2[1].axvline(
        0.6,
        color="tab:green",
        linestyle=":",
        label="End of slower point-to-point move",
    )
    axes2[1].set_xlabel("Time [s]")
    axes2[1].set_ylabel("Response [rad]")
    axes2[1].legend(loc="best")
    axes2[1].grid(True)
    fig2.tight_layout()

    if args.save:
        fig1.savefig(args.out_dir / "figure_1_robot_commands.png", dpi=160)
        fig2.savefig(args.out_dir / "figure_2_robot_response.png", dpi=160)
    if args.headless:
        fig3, axis3 = plt.subplots(1, 1, figsize=(16, 9))
        for series in (
            "Baseline response",
            "Slower response",
            "Example 1",
            "Example 2",
            "Example 3",
            "Example 4",
        ):
            _draw(
                axis3,
                data,
                "response",
                series,
                "response",
                response_labels.get(series),
            )
        axis3.set_xlim(0.84, 1.0)
        axis3.set_ylim(0.3475, 0.3525)
        axis3.set_xlabel("Time [s]")
        axis3.set_ylabel("Response [rad]")
        axis3.set_title("Residual vibration close-up")
        axis3.legend(loc="best")
        axis3.grid(True)
        fig3.tight_layout()
        fig3.savefig(args.out_dir / "figure_3_residual_closeup.png", dpi=100)
        plt.close("all")
    else:
        plt.show()


if __name__ == "__main__":
    main()
