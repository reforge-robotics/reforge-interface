#include "example_plot_data.hpp"

#include <stdexcept>

namespace reforge::examples::shaper {
namespace {

/** Recompute displayed derivatives from position samples like the Python plot.
 *
 * Args:
 *     trajectory: Source trajectory with a uniform time grid.
 *
 * Returns:
 *     Copy whose velocity and acceleration are plot-specific gradients.
 */
[[nodiscard]] Trajectory WithPlotDerivatives(const Trajectory& trajectory) {
    const double sample_time_s = trajectory.time_s(1) - trajectory.time_s(0);
    auto derivatives = EstimateDerivatives(
        trajectory.positions_rad, sample_time_s);
    return Trajectory{
        trajectory.time_s,
        trajectory.positions_rad,
        std::move(derivatives.first),
        std::move(derivatives.second),
    };
}

}  // namespace

PlotData PreparePlotData(
    const Trajectory& desired,
    const Trajectory& slower_desired,
    const Trajectory& always_on,
    const Trajectory& residual_tail,
    const Trajectory& windowed,
    const Trajectory& streamed,
    const ModalPlant& plant,
    std::size_t shaped_axis,
    double residual_shaping_start_time_s) {
    const auto axis = static_cast<Eigen::Index>(shaped_axis);
    for (const Trajectory* trajectory : {
             &desired,
             &slower_desired,
             &always_on,
             &residual_tail,
             &windowed,
             &streamed}) {
        if (axis < 0 || axis >= trajectory->positions_rad.cols()) {
            throw std::invalid_argument(
                "shaped axis must exist in every plotted trajectory");
        }
    }
    const auto simulate = [&plant, axis](const Trajectory& trajectory) {
        return SimulateModalPositionResponse(
            trajectory.time_s,
            trajectory.positions_rad.col(axis),
            plant);
    };
    return PlotData{
        WithPlotDerivatives(desired),
        WithPlotDerivatives(slower_desired),
        WithPlotDerivatives(always_on),
        WithPlotDerivatives(residual_tail),
        WithPlotDerivatives(windowed),
        WithPlotDerivatives(streamed),
        simulate(desired),
        simulate(slower_desired),
        simulate(always_on),
        simulate(residual_tail),
        simulate(windowed),
        simulate(streamed),
        shaped_axis,
        residual_shaping_start_time_s,
    };
}

}  // namespace reforge::examples::shaper
