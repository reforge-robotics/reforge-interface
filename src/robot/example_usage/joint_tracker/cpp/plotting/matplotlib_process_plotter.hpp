#pragma once

// This small example-local adapter keeps plotting out of the SDK target. It
// shells out to the user's Python Matplotlib installation so the example stays
// header-only and does not add compile-time plotting dependencies to the SDK.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace reforge_example_plotting {

inline std::string Quote(const std::string& value) {
    std::string quoted = "'";
    for (const char character : value) {
        if (character == '\'') quoted += "'\\''";
        else quoted += character;
    }
    return quoted + "'";
}

inline void RenderJointTrackerPlots(
    const std::filesystem::path& csv_path,
    const std::filesystem::path& output_directory,
    double motion_end_time_s,
    bool show) {
    const auto script_path = output_directory / "render_joint_tracker_plots.py";
    std::ofstream script(script_path);
    if (!script) throw std::runtime_error("cannot create Matplotlib helper script");
    script
        << "import csv, os, sys\n"
        << "from collections import defaultdict\n"
        << "from pathlib import Path\n"
        << "os.environ.setdefault('MPLCONFIGDIR', '/tmp/reforge-matplotlib-cache')\n"
        << "import matplotlib.pyplot as plt\n"
        << "csv_path = Path(sys.argv[1])\n"
        << "out_dir = Path(sys.argv[2])\n"
        << "motion_end_time_s = float(sys.argv[3])\n"
        << "rows = list(csv.DictReader(csv_path.open(newline='')))\n"
        << "data = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))\n"
        << "for row in rows:\n"
        << "    data[row['profile']][row['series']][int(row['joint'])].append((float(row['time_s']), float(row['value'])))\n"
        << "style = {\n"
        << "    'Desired command': ('black', '-'),\n"
        << "    'Example 1 command': ('tab:blue', '-'),\n"
        << "    'Example 2A command': ('tab:purple', '--'),\n"
        << "    'Desired trajectory': ('black', ':'),\n"
        << "    'Response to desired command': ('tab:gray', '-'),\n"
        << "    'Response to JTC Example 1 command': ('tab:blue', '-'),\n"
        << "    'Response to JTC Example 2A command': ('tab:purple', '--'),\n"
        << "}\n"
        << "profile_rows = [('position', 'Position [rad]'), ('velocity', 'Velocity [rad/s]'), ('acceleration', 'Acceleration [rad/s^2]')]\n"
        << "fig, axes = plt.subplots(3, 6, sharex=True, figsize=(20, 8))\n"
        << "for row_index, (profile, ylabel) in enumerate(profile_rows):\n"
        << "    for joint in range(1, 7):\n"
        << "        axis = axes[row_index][joint - 1]\n"
        << "        for series in ('Desired command', 'Example 1 command', 'Example 2A command'):\n"
        << "            values = data[profile][series][joint]\n"
        << "            if values:\n"
        << "                color, linestyle = style[series]\n"
        << "                axis.plot([v[0] for v in values], [v[1] for v in values], label=series, color=color, linestyle=linestyle, linewidth=1.8)\n"
        << "        axis.grid(True, alpha=0.3)\n"
        << "        if joint == 1:\n"
        << "            axis.set_ylabel(ylabel)\n"
        << "        if row_index == 0:\n"
        << "            axis.set_title(f'Joint {joint}')\n"
        << "        if row_index == 2:\n"
        << "            axis.set_xlabel('Time [s]')\n"
        << "axes[0][5].legend(loc='best')\n"
        << "fig.tight_layout()\n"
        << "fig.savefig(out_dir / 'robot_commands.png', dpi=120)\n"
        << "fig2, axes2 = plt.subplots(6, 1, sharex=True, figsize=(10, 12))\n"
        << "for joint in range(1, 7):\n"
        << "    axis = axes2[joint - 1]\n"
        << "    for series in ('Desired trajectory', 'Response to desired command', 'Response to JTC Example 1 command', 'Response to JTC Example 2A command'):\n"
        << "        values = data['response'][series][joint]\n"
        << "        if values:\n"
        << "            color, linestyle = style[series]\n"
        << "            axis.plot([v[0] for v in values], [v[1] for v in values], label=series, color=color, linestyle=linestyle, linewidth=1.8)\n"
        << "    axis.axvline(motion_end_time_s, color='tab:red', linestyle=':', label='End of commanded motion' if joint == 1 else None)\n"
        << "    axis.set_ylabel(f'Joint {joint} [rad]')\n"
        << "    axis.grid(True, alpha=0.3)\n"
        << "axes2[0].legend(loc='best')\n"
        << "axes2[-1].set_xlabel('Time [s]')\n"
        << "fig2.tight_layout()\n"
        << "fig2.savefig(out_dir / 'robot_response.png', dpi=120)\n"
        << "if sys.argv[4] == '1':\n"
        << "    plt.show()\n";
    script.close();
    std::filesystem::create_directories(output_directory);
    const std::string command = "python3 " + Quote(script_path.string()) + " " +
        Quote(csv_path.string()) + " " + Quote(output_directory.string()) +
        " " + Quote(std::to_string(motion_end_time_s)) + " " +
        (show ? "1" : "0");
    if (std::system(command.c_str()) != 0) {
        throw std::runtime_error("Matplotlib failed; install python3-matplotlib");
    }
}

}  // namespace reforge_example_plotting
