#include "example_plotting.hpp"

#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <matplotlibcpp.h>

namespace reforge::examples::shaper {
namespace {

namespace plt = matplotlibcpp;

constexpr int kPlotDpi = 160;
constexpr int kCloseupPlotDpi = 100;
constexpr std::size_t kCommandFigureWidthPx = 1600;
constexpr std::size_t kCommandFigureHeightPx = 1280;
constexpr std::size_t kResponseFigureWidthPx = 1600;
constexpr std::size_t kResponseFigureHeightPx = 1120;
constexpr std::size_t kCloseupFigureWidthPx = 1600;
constexpr std::size_t kCloseupFigureHeightPx = 900;
constexpr double kBaselineMoveDurationS = 0.2;
constexpr double kSlowerMoveDurationS = 0.6;
constexpr double kCloseupStartTimeS = 0.84;
constexpr double kCloseupStopTimeS = 1.0;

const std::string kExample1Label =
    "Example 1: Shape a full known trajectory";
const std::string kExample2Label =
    "Example 2: Shape the final part of a known trajectory";
const std::string kExample3Label =
    "Example 3: Shape a known trajectory in fixed windows";
const std::string kExample4Label =
    "Example 4: Shape a sample-by-sample stream";

/** Convert an Eigen vector expression to matplotlib-cpp storage.
 *
 * Args:
 *     values: Contiguous numeric values.
 *
 * Returns:
 *     Standard vector with the same ordered values.
 */
template <typename Derived>
[[nodiscard]] std::vector<double> ToStdVector(
    const Eigen::MatrixBase<Derived>& values) {
    return std::vector<double>(values.derived().data(),
                               values.derived().data() + values.size());
}

/** Draw one labeled line with stable Matplotlib styling.
 *
 * Args:
 *     label: Legend label.
 *     time_s: X-axis sample times [s].
 *     values: Y-axis samples.
 *     color: Matplotlib color name.
 *     linestyle: Matplotlib line style.
 *     linewidth: Line width encoded for the header-only adapter.
 */
void PlotLine(
    const std::string& label,
    const reforge::native::Vector& time_s,
    const reforge::native::Vector& values,
    const std::string& color,
    const std::string& linestyle = "-",
    const std::string& linewidth = "1.5") {
    if (!plt::plot(
            ToStdVector(time_s),
            ToStdVector(values),
            {{"label", label},
             {"color", color},
             {"linestyle", linestyle},
             {"linewidth", linewidth}})) {
        throw std::runtime_error("Matplotlib failed to draw: " + label);
    }
}

/** Draw one command-profile field for all five controller paths.
 *
 * Args:
 *     data: Plot-ready trajectories.
 *     field_index: Position, velocity, or acceleration field index.
 */
void PlotCommandField(const PlotData& data, std::size_t field_index) {
    const auto axis = static_cast<Eigen::Index>(data.shaped_axis);
    const auto field = [axis, field_index](const Trajectory& trajectory) {
        if (field_index == 0) {
            return reforge::native::Vector(trajectory.positions_rad.col(axis));
        }
        if (field_index == 1) {
            return reforge::native::Vector(
                trajectory.velocities_rad_s.col(axis));
        }
        return reforge::native::Vector(
            trajectory.accelerations_rad_s2.col(axis));
    };
    PlotLine("Desired command", data.desired.time_s, field(data.desired), "black", "-", "1.8");
    PlotLine(kExample1Label, data.always_on.time_s, field(data.always_on), "tab:blue", "-", "1.8");
    PlotLine(kExample2Label, data.residual_tail.time_s, field(data.residual_tail), "tab:orange", "-.", "2.0");
    PlotLine(kExample3Label, data.windowed.time_s, field(data.windowed), "tab:cyan", ":", "2.3");
    PlotLine(kExample4Label, data.streamed.time_s, field(data.streamed), "tab:purple", "--", "2.2");
    plt::axvline(
        data.residual_shaping_start_time_s,
        0.0,
        1.0,
        {{"color", "tab:purple"},
         {"linestyle", ":"},
         {"label", field_index == 0 ? "Residual Shaper starts" : "_nolegend_"}});
    plt::grid(true);
}

/** Render the command-profile figure.
 *
 * Args:
 *     data: Plot-ready trajectories.
 */
void RenderCommandFigure(const PlotData& data) {
    plt::figure_size(kCommandFigureWidthPx * 100 / kPlotDpi,
                     kCommandFigureHeightPx * 100 / kPlotDpi);
    const std::array<std::string, 3> labels = {
        "Position [rad]", "Velocity [rad/s]", "Acceleration [rad/s^2]"};
    for (std::size_t field_index = 0; field_index < labels.size(); ++field_index) {
        plt::subplot2grid(3, 1, static_cast<long>(field_index), 0);
        PlotCommandField(data, field_index);
        plt::ylabel(labels[field_index]);
        if (field_index == 0) {
            plt::legend({{"loc", "best"}});
        }
    }
    plt::xlabel("Time [s]");
    plt::tight_layout();
}

/** Draw all simulated controller responses on the current axes.
 *
 * Args:
 *     data: Plot-ready simulated responses.
 *     include_annotations: Whether to draw timing annotations.
 */
void PlotResponseSeries(const PlotData& data, bool include_annotations) {
    PlotLine("Baseline response: unshaped 0.2 s move", data.desired_response.time_s, data.desired_response.response_rad, "tab:gray");
    PlotLine("Comparison response: unshaped 0.6 s move", data.slower_response.time_s, data.slower_response.response_rad, "tab:green", "-.");
    PlotLine(kExample1Label, data.always_on_response.time_s, data.always_on_response.response_rad, "tab:blue", "-", "1.8");
    PlotLine(kExample2Label, data.residual_tail_response.time_s, data.residual_tail_response.response_rad, "tab:orange");
    PlotLine(kExample3Label, data.windowed_response.time_s, data.windowed_response.response_rad, "tab:cyan", ":", "2.3");
    PlotLine(kExample4Label, data.streamed_response.time_s, data.streamed_response.response_rad, "tab:purple", "--", "2.2");
    if (include_annotations) {
        plt::axvline(data.residual_shaping_start_time_s, 0.0, 1.0, {{"color", "tab:purple"}, {"linestyle", ":"}, {"label", "Residual Shaper starts"}});
        plt::axvline(kBaselineMoveDurationS, 0.0, 1.0, {{"color", "tab:red"}, {"linestyle", ":"}, {"label", "End of point-to-point move"}});
        plt::axvline(kSlowerMoveDurationS, 0.0, 1.0, {{"color", "tab:green"}, {"linestyle", ":"}, {"label", "End of slower point-to-point move"}});
    }
    plt::grid(true);
}

/** Render the baseline-comparable command and response figure.
 *
 * Args:
 *     data: Plot-ready commands and responses.
 */
void RenderResponseFigure(const PlotData& data) {
    plt::figure_size(kResponseFigureWidthPx * 100 / kPlotDpi,
                     kResponseFigureHeightPx * 100 / kPlotDpi);
    plt::subplot2grid(2, 1, 0, 0);
    const auto axis = static_cast<Eigen::Index>(data.shaped_axis);
    PlotLine("Baseline command: 0.2 s move", data.desired.time_s, data.desired.positions_rad.col(axis), "black");
    PlotLine("Comparison command: 0.6 s move", data.slower_desired.time_s, data.slower_desired.positions_rad.col(axis), "tab:green", "-.");
    plt::ylabel("Command [rad]");
    plt::legend({{"loc", "best"}});
    plt::grid(true);
    plt::subplot2grid(2, 1, 1, 0);
    PlotResponseSeries(data, true);
    plt::xlabel("Time [s]");
    plt::ylabel("Response [rad]");
    plt::legend({{"loc", "best"}});
    plt::tight_layout();
}

/** Render the deterministic documentation-only response close-up.
 *
 * Args:
 *     data: Plot-ready simulated responses.
 */
void RenderResidualCloseup(const PlotData& data) {
    plt::figure_size(kCloseupFigureWidthPx, kCloseupFigureHeightPx);
    PlotResponseSeries(data, false);
    plt::xlim(kCloseupStartTimeS, kCloseupStopTimeS);
    plt::ylim(0.3475, 0.3525);
    plt::xlabel("Time [s]");
    plt::ylabel("Response [rad]");
    plt::title("Residual vibration close-up");
    plt::legend({{"loc", "best"}});
    plt::tight_layout();
}

}  // namespace

void RenderPlots(const PlotData& data, const PlotOptions& options) {
    if (options.headless) {
        plt::backend("Agg");
    }
    if (options.headless && !options.output_directory.has_value()) {
        throw std::invalid_argument(
            "headless plotting requires an output directory");
    }
    if (options.output_directory.has_value()) {
        std::filesystem::create_directories(*options.output_directory);
    }

    RenderCommandFigure(data);
    if (options.output_directory.has_value()) {
        plt::save((*options.output_directory / "figure_1_robot_commands.png").string(), kPlotDpi);
    }
    if (options.headless) {
        plt::close();
    }

    RenderResponseFigure(data);
    if (options.output_directory.has_value()) {
        plt::save((*options.output_directory / "figure_2_robot_response.png").string(), kPlotDpi);
    }
    if (options.headless) {
        plt::close();
        RenderResidualCloseup(data);
        plt::save(
            (*options.output_directory / "figure_3_residual_closeup.png")
                .string(),
            kCloseupPlotDpi);
        plt::close();
        return;
    }

    // One blocking show displays the two runtime figures without creating the
    // documentation-only close-up as a third interactive window.
    plt::show(true);
}

}  // namespace reforge::examples::shaper
