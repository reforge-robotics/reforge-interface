#pragma once

// This example-local adapter keeps plotting out of the SDK and executable link
// boundary. It invokes the user's Python Matplotlib installation at runtime,
// following the Joint Tracker example's process-based plotting pattern.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>

#include "reforge_core/native/common/array_types.hpp"

namespace reforge::examples::shaper {

/** Own sample-aligned joint commands and their derivatives. */
struct Trajectory final {
    reforge::native::Vector time_s;
    reforge::native::Matrix positions_rad;
    reforge::native::Matrix velocities_rad_s;
    reforge::native::Matrix accelerations_rad_s2;
};

/** Describe one dominant flexible mode for the hardware-free plant. */
struct ModalPlant final {
    double natural_frequency_rad_s = 0.0;
    double damping_ratio = 0.0;
};

/** Own one simulated command and modal-plant response. */
struct SimulatedResponse final {
    reforge::native::Vector time_s;
    reforge::native::Vector command_rad;
    reforge::native::Vector response_rad;
};

/** Own all numerically prepared series required by the example figures. */
struct PlotData final {
    Trajectory desired;
    Trajectory slower_desired;
    Trajectory always_on;
    Trajectory residual_tail;
    Trajectory windowed;
    Trajectory streamed;
    SimulatedResponse desired_response;
    SimulatedResponse slower_response;
    SimulatedResponse always_on_response;
    SimulatedResponse residual_tail_response;
    SimulatedResponse windowed_response;
    SimulatedResponse streamed_response;
    std::size_t shaped_axis = 0;
    double residual_shaping_start_time_s = 0.0;
};

/** Select interactive display or deterministic headless image generation. */
struct PlotOptions final {
    bool headless = false;
    std::optional<std::filesystem::path> output_directory;
    std::filesystem::path renderer_script;
};

namespace plotting_detail {

/** Quote one shell argument without allowing shell interpolation.
 *
 * Args:
 *     value: Literal argument value.
 *
 * Returns:
 *     POSIX-shell-safe single-quoted argument.
 */
[[nodiscard]] inline std::string QuoteShellArgument(const std::string& value) {
    std::string quoted = "'";
    for (const char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted += character;
        }
    }
    return quoted + "'";
}

/** Write one trajectory field for the selected axis to the plot CSV.
 *
 * Args:
 *     output: Open CSV output stream.
 *     series: Stable legend label.
 *     field: Position, velocity, or acceleration field name.
 *     time_s: Sample times [s].
 *     values: Sample-major values for every joint.
 *     shaped_axis: Joint column rendered by this example.
 */
inline void WriteTrajectoryField(
    std::ofstream& output,
    const std::string& series,
    const std::string& field,
    const reforge::native::Vector& time_s,
    const reforge::native::Matrix& values,
    std::size_t shaped_axis) {
    const auto axis = static_cast<Eigen::Index>(shaped_axis);
    if (values.rows() != time_s.size() || axis >= values.cols()) {
        throw std::invalid_argument("plot trajectory dimensions are inconsistent");
    }
    for (Eigen::Index sample = 0; sample < time_s.size(); ++sample) {
        output << "command," << series << ',' << field << ','
               << time_s(sample) << ',' << values(sample, axis) << '\n';
    }
}

/** Write position, velocity, and acceleration for one command series.
 *
 * Args:
 *     output: Open CSV output stream.
 *     series: Stable legend label.
 *     trajectory: Plot-ready command trajectory.
 *     shaped_axis: Joint column rendered by this example.
 */
inline void WriteTrajectory(
    std::ofstream& output,
    const std::string& series,
    const Trajectory& trajectory,
    std::size_t shaped_axis) {
    WriteTrajectoryField(
        output,
        series,
        "position",
        trajectory.time_s,
        trajectory.positions_rad,
        shaped_axis);
    WriteTrajectoryField(
        output,
        series,
        "velocity",
        trajectory.time_s,
        trajectory.velocities_rad_s,
        shaped_axis);
    WriteTrajectoryField(
        output,
        series,
        "acceleration",
        trajectory.time_s,
        trajectory.accelerations_rad_s2,
        shaped_axis);
}

/** Write one simulated plant response to the plot CSV.
 *
 * Args:
 *     output: Open CSV output stream.
 *     series: Stable legend label.
 *     response: Sample-aligned plant response.
 */
inline void WriteResponse(
    std::ofstream& output,
    const std::string& series,
    const SimulatedResponse& response) {
    if (response.time_s.size() != response.response_rad.size()) {
        throw std::invalid_argument("plot response dimensions are inconsistent");
    }
    for (Eigen::Index sample = 0; sample < response.time_s.size(); ++sample) {
        output << "response," << series << ",response,"
               << response.time_s(sample) << ',' << response.response_rad(sample)
               << '\n';
    }
}

/** Serialize every command and response series consumed by Matplotlib.
 *
 * Args:
 *     path: Destination CSV path.
 *     data: Numerically prepared command profiles and responses.
 */
inline void WritePlotCsv(
    const std::filesystem::path& path,
    const PlotData& data) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot create Matplotlib plot-data CSV");
    }
    output << std::setprecision(17);
    output << "category,series,field,time_s,value\n";
    WriteTrajectory(output, "Desired command", data.desired, data.shaped_axis);
    WriteTrajectory(
        output, "Slower desired command", data.slower_desired, data.shaped_axis);
    WriteTrajectory(output, "Example 1", data.always_on, data.shaped_axis);
    WriteTrajectory(output, "Example 2", data.residual_tail, data.shaped_axis);
    WriteTrajectory(output, "Example 3", data.windowed, data.shaped_axis);
    WriteTrajectory(output, "Example 4", data.streamed, data.shaped_axis);
    WriteResponse(output, "Baseline response", data.desired_response);
    WriteResponse(output, "Slower response", data.slower_response);
    WriteResponse(output, "Example 1", data.always_on_response);
    WriteResponse(output, "Example 2", data.residual_tail_response);
    WriteResponse(output, "Example 3", data.windowed_response);
    WriteResponse(output, "Example 4", data.streamed_response);
}

}  // namespace plotting_detail

/** Render the two runtime figures and optional documentation close-up.
 *
 * Interactive mode opens exactly two figure windows. Headless mode suppresses
 * GUI windows and writes exactly three PNG files to `output_directory`.
 * Matplotlib is invoked as a child process through the checked-in renderer
 * script and is not a compile dependency.
 *
 * Args:
 *     data: Numerically prepared command profiles and simulated responses.
 *     options: Display mode and image destination.
 */
inline void RenderPlots(const PlotData& data, const PlotOptions& options) {
    if (options.headless && !options.output_directory.has_value()) {
        throw std::invalid_argument(
            "headless plotting requires an output directory");
    }
    const std::filesystem::path working_directory =
        options.output_directory.value_or(
            std::filesystem::temp_directory_path() / "reforge_shaper_plots");
    if (!std::filesystem::is_regular_file(options.renderer_script)) {
        throw std::runtime_error(
            "Matplotlib renderer script is missing: " +
            options.renderer_script.string());
    }
    std::filesystem::create_directories(working_directory);
    const auto csv_path = working_directory / "shaper_plot_data.csv";
    plotting_detail::WritePlotCsv(csv_path, data);

    const std::string command =
        "python3 " +
        plotting_detail::QuoteShellArgument(options.renderer_script.string()) +
        " " + plotting_detail::QuoteShellArgument(csv_path.string()) + " " +
        plotting_detail::QuoteShellArgument(working_directory.string()) + " " +
        (options.headless ? "1" : "0") + " " +
        (options.output_directory.has_value() ? "1" : "0") + " " +
        plotting_detail::QuoteShellArgument(
            std::to_string(data.residual_shaping_start_time_s));
    if (std::system(command.c_str()) != 0) {
        throw std::runtime_error(
            "Matplotlib rendering failed; install python3 and "
            "python3-matplotlib");
    }
}

}  // namespace reforge::examples::shaper
