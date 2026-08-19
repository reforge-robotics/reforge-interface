#pragma once

#include <filesystem>
#include <optional>

#include "example_plot_data.hpp"

namespace reforge::examples::shaper {

/** Select interactive display or deterministic headless image generation. */
struct PlotOptions final {
    bool headless = false;
    std::optional<std::filesystem::path> output_directory;
};

/** Render the two runtime figures and optional documentation close-up.
 *
 * Interactive mode opens exactly two figure windows. Headless mode suppresses
 * GUI windows and writes exactly three PNG files to `output_directory`.
 *
 * Args:
 *     data: Numerically prepared command profiles and simulated responses.
 *     options: Display mode and image destination.
 */
void RenderPlots(const PlotData& data, const PlotOptions& options);

}  // namespace reforge::examples::shaper
