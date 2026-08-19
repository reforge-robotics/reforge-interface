#pragma once

#include "example_utility.hpp"

namespace reforge::examples::shaper {

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

/** Prepare deterministic profile and plant-response arrays for plotting.
 *
 * Args:
 *     desired: Unshaped baseline command.
 *     slower_desired: Slower controller-off comparison command.
 *     always_on: Example 1 shaped command.
 *     residual_tail: Example 2 shaped command.
 *     windowed: Example 3 shaped command.
 *     streamed: Example 4 shaped command.
 *     plant: Dominant hardware-free modal plant.
 *     shaped_axis: Joint column plotted and simulated.
 *     residual_shaping_start_time_s: First residual-tail command change [s].
 *
 * Returns:
 *     Plot-ready numeric data independent of any GUI or Python runtime.
 */
[[nodiscard]] PlotData PreparePlotData(
    const Trajectory& desired,
    const Trajectory& slower_desired,
    const Trajectory& always_on,
    const Trajectory& residual_tail,
    const Trajectory& windowed,
    const Trajectory& streamed,
    const ModalPlant& plant,
    std::size_t shaped_axis,
    double residual_shaping_start_time_s);

}  // namespace reforge::examples::shaper
