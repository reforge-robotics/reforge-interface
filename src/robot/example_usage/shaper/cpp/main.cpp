#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "plotting/matplotlib_process_plotter.hpp"
#include <unsupported/Eigen/MatrixFunctions>
#include "reforge_core/control/runtime/modal_inference.hpp"
#include "reforge_core/control/shaper/backend/backend_configuration.hpp"
#include "reforge_core/control/shaper/backend/backend_requests.hpp"
#include "reforge_core/control/shaper/backend/native_shaper.hpp"
#include "reforge_core/control/shaper/config.hpp"
#include "reforge_core/control/shaper/windowing.hpp"

#ifndef REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION
#define REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION "unknown"
#endif

namespace reforge::examples::shaper {

/** Own one shaped window and its stable source range. */
struct ShapedWindow final {
    Trajectory trajectory;
    std::size_t start_index = 0;
    std::size_t stop_index = 0;
    bool is_tail = false;
};

namespace {

using reforge::native::Matrix;
using reforge::native::Vector;

/** Differentiate a sample-major matrix with second-order edge estimates.
 *
 * Args:
 *     samples: Sample-major values.
 *     sample_time_s: Uniform sample period [s].
 *
 * Returns:
 *     First derivative on the input timing grid.
 */
[[nodiscard]] Matrix Gradient(
    const Matrix& samples,
    double sample_time_s) {
    if (samples.rows() < 2) {
        throw std::invalid_argument(
            "at least two samples are required to estimate derivatives");
    }
    Matrix derivative(samples.rows(), samples.cols());
    if (samples.rows() == 2) {
        const auto slope =
            (samples.row(1) - samples.row(0)) / sample_time_s;
        derivative.row(0) = slope;
        derivative.row(1) = slope;
        return derivative;
    }

    derivative.row(0) =
        (-3.0 * samples.row(0) + 4.0 * samples.row(1) - samples.row(2)) /
        (2.0 * sample_time_s);
    for (Eigen::Index row = 1; row + 1 < samples.rows(); ++row) {
        derivative.row(row) =
            (samples.row(row + 1) - samples.row(row - 1)) /
            (2.0 * sample_time_s);
    }
    const Eigen::Index last = samples.rows() - 1;
    derivative.row(last) =
        (3.0 * samples.row(last) - 4.0 * samples.row(last - 1) +
         samples.row(last - 2)) /
        (2.0 * sample_time_s);
    return derivative;
}

/** Return whether every element of an Eigen expression is finite.
 *
 * Args:
 *     values: Numeric Eigen expression.
 *
 * Returns:
 *     True only when every element is finite.
 */
template <typename Derived>
[[nodiscard]] bool AllFinite(const Eigen::MatrixBase<Derived>& values) {
    return values.array().isFinite().all();
}

}  // namespace

/** Estimate velocity and acceleration with NumPy-compatible gradients.
 *
 * Args:
 *     positions_rad: Sample-major joint positions [rad].
 *     sample_time_s: Controller sample period [s].
 *
 * Returns:
 *     Sample-major velocity [rad/s] and acceleration [rad/s^2].
 */
[[nodiscard]] std::pair<reforge::native::Matrix, reforge::native::Matrix>
EstimateDerivatives(
    const reforge::native::Matrix& positions_rad,
    double sample_time_s);

Trajectory GeneratePointToPointTrajectory(
    const Vector& start_position_rad,
    const Vector& goal_position_rad,
    double sample_time_s,
    double move_duration_s,
    double dwell_duration_s) {
    if (start_position_rad.size() != goal_position_rad.size()) {
        throw std::invalid_argument(
            "start and goal position vectors must have equal sizes");
    }
    if (!(sample_time_s > 0.0) || !(move_duration_s > 0.0) ||
        dwell_duration_s < 0.0) {
        throw std::invalid_argument(
            "sample and move durations must be positive and dwell nonnegative");
    }

    const auto move_samples = static_cast<std::size_t>(
                                  std::llround(move_duration_s / sample_time_s)) +
                              1;
    const auto dwell_samples = static_cast<std::size_t>(
        std::llround(dwell_duration_s / sample_time_s));
    const auto total_samples = move_samples + dwell_samples;
    Matrix positions_rad(
        static_cast<Eigen::Index>(total_samples), start_position_rad.size());
    const Vector move_delta_rad = goal_position_rad - start_position_rad;

    for (std::size_t sample = 0; sample < move_samples; ++sample) {
        const double phase = static_cast<double>(sample) /
                             static_cast<double>(move_samples - 1);
        const double blend = 10.0 * std::pow(phase, 3) -
                             15.0 * std::pow(phase, 4) +
                             6.0 * std::pow(phase, 5);
        positions_rad.row(static_cast<Eigen::Index>(sample)) =
            (start_position_rad + blend * move_delta_rad).transpose();
    }
    for (std::size_t sample = move_samples; sample < total_samples; ++sample) {
        positions_rad.row(static_cast<Eigen::Index>(sample)) =
            goal_position_rad.transpose();
    }

    Vector time_s(static_cast<Eigen::Index>(total_samples));
    for (std::size_t sample = 0; sample < total_samples; ++sample) {
        time_s(static_cast<Eigen::Index>(sample)) =
            static_cast<double>(sample) * sample_time_s;
    }
    auto derivatives = EstimateDerivatives(positions_rad, sample_time_s);
    return Trajectory{
        std::move(time_s),
        std::move(positions_rad),
        std::move(derivatives.first),
        std::move(derivatives.second),
    };
}

Trajectory GeneratePointToPointSample(
    const Vector& start_position_rad,
    const Vector& goal_position_rad,
    double current_time_s,
    double move_duration_s) {
    if (start_position_rad.size() != goal_position_rad.size()) {
        throw std::invalid_argument(
            "start and goal position vectors must have equal sizes");
    }
    if (!(move_duration_s > 0.0)) {
        throw std::invalid_argument("move duration must be positive");
    }

    const double phase = std::clamp(
        current_time_s / move_duration_s, 0.0, 1.0);
    const double blend = 10.0 * std::pow(phase, 3) -
                         15.0 * std::pow(phase, 4) +
                         6.0 * std::pow(phase, 5);
    double blend_dot = 30.0 * std::pow(phase, 2) -
                       60.0 * std::pow(phase, 3) +
                       30.0 * std::pow(phase, 4);
    double blend_ddot = 60.0 * phase - 180.0 * std::pow(phase, 2) +
                        120.0 * std::pow(phase, 3);
    if (phase == 0.0 || phase == 1.0) {
        blend_dot = 0.0;
        blend_ddot = 0.0;
    }

    const Vector move_delta_rad = goal_position_rad - start_position_rad;
    Trajectory result;
    result.time_s = Vector::Constant(1, current_time_s);
    result.positions_rad.resize(1, start_position_rad.size());
    result.velocities_rad_s.resize(1, start_position_rad.size());
    result.accelerations_rad_s2.resize(1, start_position_rad.size());
    result.positions_rad.row(0) =
        (start_position_rad + blend * move_delta_rad).transpose();
    result.velocities_rad_s.row(0) =
        ((blend_dot / move_duration_s) * move_delta_rad).transpose();
    result.accelerations_rad_s2.row(0) =
        ((blend_ddot / (move_duration_s * move_duration_s)) * move_delta_rad)
            .transpose();
    return result;
}

std::pair<Matrix, Matrix> EstimateDerivatives(
    const Matrix& positions_rad,
    double sample_time_s) {
    if (!(sample_time_s > 0.0)) {
        throw std::invalid_argument("sample time must be positive");
    }
    Matrix velocities_rad_s = Gradient(positions_rad, sample_time_s);
    Matrix accelerations_rad_s2 = Gradient(velocities_rad_s, sample_time_s);
    return {std::move(velocities_rad_s), std::move(accelerations_rad_s2)};
}

void ValidateTrajectory(
    const Trajectory& trajectory,
    std::size_t expected_joint_count,
    double sample_time_s,
    const std::string& label) {
    const Eigen::Index sample_count = trajectory.positions_rad.rows();
    const Eigen::Index joint_count =
        static_cast<Eigen::Index>(expected_joint_count);
    if (sample_count < 2 || trajectory.positions_rad.cols() != joint_count ||
        trajectory.velocities_rad_s.rows() != sample_count ||
        trajectory.velocities_rad_s.cols() != joint_count ||
        trajectory.accelerations_rad_s2.rows() != sample_count ||
        trajectory.accelerations_rad_s2.cols() != joint_count ||
        trajectory.time_s.size() != sample_count) {
        throw std::runtime_error(label + " has inconsistent array dimensions");
    }
    if (!AllFinite(trajectory.time_s) ||
        !AllFinite(trajectory.positions_rad) ||
        !AllFinite(trajectory.velocities_rad_s) ||
        !AllFinite(trajectory.accelerations_rad_s2)) {
        throw std::runtime_error(label + " contains a non-finite value");
    }
    for (Eigen::Index sample = 1; sample < sample_count; ++sample) {
        const double interval_s =
            trajectory.time_s(sample) - trajectory.time_s(sample - 1);
        if (std::abs(interval_s - sample_time_s) > 1.0e-12) {
            throw std::runtime_error(label + " is not on the configured timing grid");
        }
    }
}

Trajectory ConcatenateWindowedOutputs(
    const std::vector<ShapedWindow>& windows,
    std::size_t expected_sample_count) {
    if (windows.empty()) {
        throw std::invalid_argument("at least one shaped window is required");
    }
    const Eigen::Index joint_count = windows.front().trajectory.positions_rad.cols();
    Trajectory output;
    output.time_s.resize(static_cast<Eigen::Index>(expected_sample_count));
    output.positions_rad.resize(
        static_cast<Eigen::Index>(expected_sample_count), joint_count);
    output.velocities_rad_s.resize(
        static_cast<Eigen::Index>(expected_sample_count), joint_count);
    output.accelerations_rad_s2.resize(
        static_cast<Eigen::Index>(expected_sample_count), joint_count);

    std::size_t cursor = 0;
    for (const ShapedWindow& window : windows) {
        const auto window_samples = static_cast<std::size_t>(
            window.trajectory.positions_rad.rows());
        if (window.is_tail || window.start_index != cursor ||
            window.stop_index != cursor + window_samples ||
            window.stop_index > expected_sample_count) {
            throw std::runtime_error(
                "window ranges do not reconstruct the source sample order");
        }
        const Eigen::Index destination_row = static_cast<Eigen::Index>(cursor);
        const Eigen::Index row_count = static_cast<Eigen::Index>(window_samples);
        output.time_s.segment(destination_row, row_count) =
            window.trajectory.time_s;
        output.positions_rad.middleRows(destination_row, row_count) =
            window.trajectory.positions_rad;
        output.velocities_rad_s.middleRows(destination_row, row_count) =
            window.trajectory.velocities_rad_s;
        output.accelerations_rad_s2.middleRows(destination_row, row_count) =
            window.trajectory.accelerations_rad_s2;
        cursor = window.stop_index;
    }
    if (cursor != expected_sample_count) {
        throw std::runtime_error(
            "window ranges leave a gap in the source sample order");
    }
    return output;
}

SimulatedResponse SimulateModalPositionResponse(
    const Vector& time_s,
    const Vector& command_rad,
    const ModalPlant& plant) {
    if (time_s.size() != command_rad.size() || time_s.size() < 2) {
        throw std::invalid_argument(
            "simulation time and command arrays must have equal nontrivial size");
    }
    if (!(plant.natural_frequency_rad_s > 0.0) ||
        !(plant.damping_ratio > 0.0) || plant.damping_ratio >= 1.0) {
        throw std::invalid_argument("modal plant parameters are not physical");
    }
    const double sample_time_s = time_s(1) - time_s(0);
    if (!(sample_time_s > 0.0)) {
        throw std::invalid_argument("simulation time must be strictly increasing");
    }
    for (Eigen::Index sample = 2; sample < time_s.size(); ++sample) {
        if (std::abs((time_s(sample) - time_s(sample - 1)) - sample_time_s) >
            1.0e-12) {
            throw std::invalid_argument("simulation time must be uniformly spaced");
        }
    }

    const double wn = plant.natural_frequency_rad_s;
    const double zeta = plant.damping_ratio;
    Eigen::Matrix4d augmented = Eigen::Matrix4d::Zero();
    augmented(0, 1) = sample_time_s;
    augmented(1, 0) = -wn * wn * sample_time_s;
    augmented(1, 1) = -2.0 * zeta * wn * sample_time_s;
    augmented(1, 2) = wn * wn * sample_time_s;
    // The last state stores the complete step-to-step input delta, so this
    // normalized first-order-hold transition is intentionally not scaled by dt.
    augmented(2, 3) = 1.0;
    const Eigen::Matrix4d transition = augmented.exp();
    const Eigen::Matrix2d state_transition = transition.block<2, 2>(0, 0);
    const Eigen::Vector2d previous_input =
        transition.block<2, 1>(0, 2) - transition.block<2, 1>(0, 3);
    const Eigen::Vector2d current_input = transition.block<2, 1>(0, 3);

    Vector response_rad = Vector::Zero(time_s.size());
    Eigen::Vector2d state = Eigen::Vector2d::Zero();
    for (Eigen::Index sample = 1; sample < time_s.size(); ++sample) {
        state = state_transition * state +
                previous_input * command_rad(sample - 1) +
                current_input * command_rad(sample);
        response_rad(sample) = state(0);
    }
    return SimulatedResponse{time_s, command_rad, std::move(response_rad)};
}

double ResidualVibrationRad(
    const SimulatedResponse& response,
    double final_value_rad,
    double start_time_s) {
    double residual_rad = 0.0;
    for (Eigen::Index sample = 0; sample < response.time_s.size(); ++sample) {
        if (response.time_s(sample) >= start_time_s) {
            residual_rad = std::max(
                residual_rad,
                std::abs(response.response_rad(sample) - final_value_rad));
        }
    }
    return residual_rad;
}

double FirstChangeTimeS(
    const Trajectory& candidate,
    const Trajectory& reference,
    std::size_t axis_index,
    double tolerance_rad,
    double fallback_time_s) {
    if (axis_index >= static_cast<std::size_t>(candidate.positions_rad.cols()) ||
        axis_index >= static_cast<std::size_t>(reference.positions_rad.cols())) {
        throw std::invalid_argument(
            "candidate and reference trajectories must contain the selected axis");
    }
    const Eigen::Index axis = static_cast<Eigen::Index>(axis_index);
    for (Eigen::Index sample = 0; sample < candidate.time_s.size(); ++sample) {
        const double candidate_time_s = candidate.time_s(sample);
        if (candidate_time_s < reference.time_s(0) ||
            candidate_time_s > reference.time_s(reference.time_s.size() - 1)) {
            throw std::invalid_argument(
                "reference trajectory does not cover every candidate time");
        }
        const double* reference_begin = reference.time_s.data();
        const double* reference_end = reference_begin + reference.time_s.size();
        const double* upper =
            std::lower_bound(reference_begin, reference_end, candidate_time_s);
        double reference_position_rad = 0.0;
        if (upper == reference_begin) {
            reference_position_rad = reference.positions_rad(0, axis);
        } else if (upper == reference_end) {
            reference_position_rad =
                reference.positions_rad(reference.time_s.size() - 1, axis);
        } else {
            const Eigen::Index upper_index = upper - reference_begin;
            if (std::abs(*upper - candidate_time_s) <= 1.0e-12) {
                reference_position_rad =
                    reference.positions_rad(upper_index, axis);
            } else {
                const Eigen::Index lower_index = upper_index - 1;
                const double fraction =
                    (candidate_time_s - reference.time_s(lower_index)) /
                    (reference.time_s(upper_index) -
                     reference.time_s(lower_index));
                reference_position_rad =
                    reference.positions_rad(lower_index, axis) +
                    fraction *
                        (reference.positions_rad(upper_index, axis) -
                         reference.positions_rad(lower_index, axis));
            }
        }
        if (std::abs(
                candidate.positions_rad(sample, axis) -
                reference_position_rad) > tolerance_rad) {
            return candidate_time_s;
        }
    }
    return fallback_time_s;
}

void WriteTrajectoryCsv(
    const std::filesystem::path& output_path,
    const Trajectory& trajectory,
    const std::vector<std::string>& joint_names) {
    if (joint_names.size() !=
        static_cast<std::size_t>(trajectory.positions_rad.cols())) {
        throw std::invalid_argument(
            "joint names must match trajectory column count");
    }
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error(
            "could not open trajectory artifact: " + output_path.string());
    }
    output << "time_s";
    for (const char* field : {"position_rad", "velocity_rad_s", "acceleration_rad_s2"}) {
        for (const std::string& joint_name : joint_names) {
            output << ',' << field << '.' << joint_name;
        }
    }
    output << '\n' << std::setprecision(17);
    for (Eigen::Index sample = 0; sample < trajectory.time_s.size(); ++sample) {
        output << trajectory.time_s(sample);
        for (const Matrix* values : {
                 &trajectory.positions_rad,
                 &trajectory.velocities_rad_s,
                 &trajectory.accelerations_rad_s2}) {
            for (Eigen::Index joint = 0; joint < values->cols(); ++joint) {
                output << ',' << (*values)(sample, joint);
            }
        }
        output << '\n';
    }
    if (!output) {
        throw std::runtime_error(
            "failed while writing trajectory artifact: " + output_path.string());
    }
}

}  // namespace reforge::examples::shaper


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


namespace {

using Clock = std::chrono::steady_clock;
using reforge::control::runtime::FittedAxisModes;
using reforge::control::runtime::RobotState;
using reforge::control::runtime::SharedImpulsePolicy;
using reforge::control::shaper::ResidualPathScope;
using reforge::control::shaper::ResidualShapingStrategy;
using reforge::control::shaper::ResidualSwitchLimits;
using reforge::control::shaper::ShaperWindowRequest;
using reforge::control::shaper::backend::BackendConfiguration;
using reforge::control::shaper::backend::NativeShaper;
using reforge::control::shaper::backend::ProcessSampleRequest;
using reforge::control::shaper::backend::ProcessTrajectoryRequest;
using reforge::examples::shaper::ConcatenateWindowedOutputs;
using reforge::examples::shaper::FirstChangeTimeS;
using reforge::examples::shaper::GeneratePointToPointSample;
using reforge::examples::shaper::GeneratePointToPointTrajectory;
using reforge::examples::shaper::ModalPlant;
using reforge::examples::shaper::PlotOptions;
using reforge::examples::shaper::PreparePlotData;
using reforge::examples::shaper::RenderPlots;
using reforge::examples::shaper::ResidualVibrationRad;
using reforge::examples::shaper::ShapedWindow;
using reforge::examples::shaper::SimulateModalPositionResponse;
using reforge::examples::shaper::Trajectory;
using reforge::examples::shaper::ValidateTrajectory;
using reforge::examples::shaper::WriteTrajectoryCsv;
using reforge::native::Matrix;
using reforge::native::ShapedTrajectory;
using reforge::native::Vector;

constexpr double kSampleTimeS = 0.004;
constexpr std::size_t kNumAxes = 2;
constexpr std::size_t kNumJoints = 6;
constexpr std::size_t kShapedAxis = 0;
constexpr double kGoalPositionRad = 0.35;
constexpr double kMoveDurationS = 0.2;
constexpr double kDwellDurationS = 0.8;
constexpr double kSlowerMoveDurationS = 0.6;
constexpr double kResidualWindowDurationS = 0.12;
constexpr double kResidualTransitionMarginS = 0.04;
constexpr double kWindowDurationS = 0.20;
constexpr double kShaperEnabledWeight = 1.0;
constexpr double kSwitchMaxVelocityRadS = 3.0;
constexpr double kSwitchMaxAccelerationRadS2 = 45.0;
constexpr double kSwitchMaxSearchS = 0.75;
constexpr std::size_t kSwitchMaxQpAttempts = 10;
const std::vector<std::string> kJointOrder = {
    "joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};

/** Own command-line paths selected for the example run. */
struct Options final {
    std::filesystem::path assets_directory;
    std::filesystem::path baseline_manifest;
    std::optional<std::filesystem::path> output_directory;
    std::filesystem::path renderer_script;
    bool headless = false;
};

/** Own frozen numeric expectations loaded from the JSON baseline manifest. */
struct ExpectedMetrics final {
    double sample_time_s = 0.0;
    double metric_tolerance = 0.0;
    double time_tolerance_s = 0.0;
    double position_tolerance_rad = 0.0;
    double velocity_tolerance_rad_s = 0.0;
    double acceleration_tolerance_rad_s2 = 0.0;
    double dominant_natural_frequency_rad_s = 0.0;
    double dominant_damping_ratio = 0.0;
    double residual_start_time_s = 0.0;
    double unshaped_residual_rad = 0.0;
    double slower_unshaped_residual_rad = 0.0;
    double deceleration_start_time_s = 0.0;
    double residual_change_start_time_s = 0.0;
    double always_on_residual_rad = 0.0;
    double residual_tail_residual_rad = 0.0;
    double windowed_residual_rad = 0.0;
    double streamed_residual_rad = 0.0;
};

/** Own metrics printed and optionally serialized by the example. */
struct ExampleMetrics final {
    ModalPlant plant;
    double residual_start_time_s = 0.0;
    double unshaped_residual_rad = 0.0;
    double slower_unshaped_residual_rad = 0.0;
    double deceleration_start_time_s = 0.0;
    double residual_change_start_time_s = 0.0;
    double always_on_residual_rad = 0.0;
    double residual_tail_residual_rad = 0.0;
    double windowed_residual_rad = 0.0;
    double streamed_residual_rad = 0.0;
};

/** Parse artifact paths without accepting unrelated runtime behaviors.
 *
 * Args:
 *     argument_count: Number of process arguments.
 *     arguments: Process argument values.
 *
 * Returns:
 *     Validated asset and optional output paths.
 */
[[nodiscard]] Options ParseOptions(int argument_count, char** arguments) {
    Options options;
    const std::filesystem::path executable_path =
        std::filesystem::absolute(arguments[0]);
    options.assets_directory = executable_path.parent_path() / "assets";
    options.baseline_manifest =
        options.assets_directory / "expected_metrics.json";
    options.renderer_script =
        executable_path.parent_path() / "matplotlib_renderer.py";
    bool baseline_manifest_overridden = false;
    for (int index = 1; index < argument_count; ++index) {
        const std::string_view argument(arguments[index]);
        if (argument == "--assets-dir" && index + 1 < argument_count) {
            options.assets_directory = arguments[++index];
        } else if (argument == "--baseline" && index + 1 < argument_count) {
            options.baseline_manifest = arguments[++index];
            baseline_manifest_overridden = true;
        } else if (argument == "--output-dir" && index + 1 < argument_count) {
            options.output_directory = arguments[++index];
        } else if (argument == "--headless" || argument == "--no-gui") {
            options.headless = true;
        } else if (argument == "--help") {
            std::cout
                << "Usage: shaper_example_usage [--assets-dir PATH] "
                   "[--baseline PATH] [--output-dir PATH] "
                   "[--headless|--no-gui]\n";
            std::exit(0);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete argument: " + std::string(argument));
        }
    }
    if (!baseline_manifest_overridden) {
        options.baseline_manifest =
            options.assets_directory / "expected_metrics.json";
    }
    if (options.headless && !options.output_directory.has_value()) {
        options.output_directory = executable_path.parent_path() / "figures";
    }
    return options;
}

/** Read one required numeric member from a strict baseline JSON document.
 *
 * This reader accepts the reviewed manifest's numeric schema and rejects
 * missing, nonnumeric, or non-finite members while keeping the example's
 * validation errors actionable.
 *
 * Args:
 *     document: Complete baseline manifest text.
 *     member_name: Required JSON member name.
 *
 * Returns:
 *     Finite numeric member value.
 */
[[nodiscard]] double ReadRequiredNumber(
    const nlohmann::json& document,
    const std::string& member_name) {
    if (!document.is_object() || !document.contains(member_name)) {
        throw std::invalid_argument(
            "baseline manifest is missing numeric member: " + member_name);
    }
    const auto& member = document.at(member_name);
    if (!member.is_number()) {
        throw std::invalid_argument(
            "baseline manifest member is not numeric: " + member_name);
    }
    const double value = member.get<double>();
    if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "baseline manifest member is not finite: " + member_name);
    }
    return value;
}

/** Load every C++ runtime expectation from the canonical JSON manifest.
 *
 * Args:
 *     manifest_path: Baseline manifest path.
 *
 * Returns:
 *     Complete frozen metric and tolerance contract.
 */
[[nodiscard]] ExpectedMetrics LoadExpectedMetrics(
    const std::filesystem::path& manifest_path) {
    std::ifstream input(manifest_path);
    if (!input) {
        throw std::invalid_argument(
            "baseline manifest is missing or unreadable: " +
            manifest_path.string());
    }
    const std::string document_text{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    nlohmann::json document;
    try {
        document = nlohmann::json::parse(document_text);
    } catch (const nlohmann::json::parse_error& error) {
        throw std::invalid_argument(
            "baseline manifest is not valid JSON: " +
            std::string(error.what()));
    }
    if (!document.is_object() || !document.contains("metrics") ||
        !document.at("metrics").is_object()) {
        throw std::invalid_argument(
            "baseline manifest is missing object: metrics");
    }
    const auto& metrics = document.at("metrics");
    const ExpectedMetrics expected{
        ReadRequiredNumber(document, "sample_time_s"),
        ReadRequiredNumber(document, "metric_tolerance"),
        ReadRequiredNumber(document, "time_tolerance_s"),
        ReadRequiredNumber(document, "position_tolerance_rad"),
        ReadRequiredNumber(document, "velocity_tolerance_rad_s"),
        ReadRequiredNumber(document, "acceleration_tolerance_rad_s2"),
        ReadRequiredNumber(document, "dominant_natural_frequency_rad_s"),
        ReadRequiredNumber(document, "dominant_damping_ratio"),
        ReadRequiredNumber(metrics, "residual_start_time_s"),
        ReadRequiredNumber(metrics, "unshaped_residual_rad"),
        ReadRequiredNumber(metrics, "slower_unshaped_residual_rad"),
        ReadRequiredNumber(metrics, "deceleration_start_time_s"),
        ReadRequiredNumber(metrics, "residual_change_start_time_s"),
        ReadRequiredNumber(metrics, "always_on_residual_rad"),
        ReadRequiredNumber(metrics, "residual_tail_residual_rad"),
        ReadRequiredNumber(metrics, "windowed_residual_rad"),
        ReadRequiredNumber(metrics, "streamed_residual_rad")};
    for (const auto& tolerance : std::vector<std::pair<std::string, double>>{
             {"metric_tolerance", expected.metric_tolerance},
             {"time_tolerance_s", expected.time_tolerance_s},
             {"position_tolerance_rad", expected.position_tolerance_rad},
             {"velocity_tolerance_rad_s", expected.velocity_tolerance_rad_s},
             {"acceleration_tolerance_rad_s2",
              expected.acceleration_tolerance_rad_s2}}) {
        if (tolerance.second <= 0.0) {
            throw std::invalid_argument(
                "baseline manifest tolerance must be positive: " +
                tolerance.first);
        }
    }
    return expected;
}

/** Construct the explicit synthetic-fixture native backend configuration.
 *
 * Args:
 *     assets_directory: Directory containing the shared model and URDF.
 *     shared_impulse_policy: Modal impulse-sharing policy for this controller.
 *     shared_impulse_shapes_all_joints: Whether a shared impulse applies to all
 *         six commanded joints.
 *
 * Returns:
 *     Complete native backend construction settings.
 */
[[nodiscard]] BackendConfiguration MakeBackendConfiguration(
    const std::filesystem::path& assets_directory,
    SharedImpulsePolicy shared_impulse_policy =
        SharedImpulsePolicy::kCombineAllModes,
    bool shared_impulse_shapes_all_joints = true) {
    const std::filesystem::path model_directory = assets_directory;
    const std::filesystem::path urdf_filepath =
        assets_directory / "test_robot.urdf";
    if (!std::filesystem::is_regular_file(
            assets_directory / "model_bundle.json") ||
        !std::filesystem::is_regular_file(
            model_directory / "shaper_models.native.json") ||
        !std::filesystem::is_regular_file(urdf_filepath)) {
        throw std::invalid_argument(
            "assets directory must contain the synthetic native model "
            "fixture and repository test_robot.urdf: " +
            assets_directory.string());
    }

    BackendConfiguration configuration;
    configuration.sample_time_s = kSampleTimeS;
    configuration.model_directory = model_directory;
    configuration.urdf_filepath = urdf_filepath;
    configuration.num_axes = kNumAxes;
    configuration.side_length_m = 1.0;
    configuration.base_height_m = 0.0;
    configuration.num_joints = kNumJoints;
    configuration.probability_threshold = 0.5;
    configuration.shared_impulse_policy = shared_impulse_policy;
    configuration.shared_impulse_shapes_all_joints =
        shared_impulse_shapes_all_joints;
    configuration.residual_path_scope = ResidualPathScope::kShapedAxes;
    configuration.aligned_tail_require_zero_acceleration_region = false;
    configuration.train_base_angles_rad = Vector::Zero(kNumAxes);
    configuration.feature_frame_translation_m = std::nullopt;
    return configuration;
}

/** Convert an SDK-owned shaped trajectory to the cohesive example type.
 *
 * Args:
 *     shaped: Native SDK trajectory output.
 *
 * Returns:
 *     Example trajectory with explicit units in its field names.
 */
[[nodiscard]] Trajectory ToTrajectory(ShapedTrajectory shaped) {
    return Trajectory{
        std::move(shaped.time),
        std::move(shaped.positions),
        std::move(shaped.velocities),
        std::move(shaped.accelerations),
    };
}

/** Build the qualified bounded-search residual-switch profile.
 *
 * Returns:
 *     Explicit six-joint example switch limits.
 */
[[nodiscard]] ResidualSwitchLimits MakeSpeedFirstSwitchLimits() {
    ResidualSwitchLimits limits;
    limits.max_velocity_rad_s = Vector::Constant(
        static_cast<Eigen::Index>(kNumJoints), kSwitchMaxVelocityRadS);
    limits.max_acceleration_rad_s2 = Vector::Constant(
        static_cast<Eigen::Index>(kNumJoints), kSwitchMaxAccelerationRadS2);
    limits.max_search_s = kSwitchMaxSearchS;
    limits.window_target_margin_s = 0.0;
    limits.max_qp_attempts = kSwitchMaxQpAttempts;
    limits.tracking_weight = 1.0;
    limits.acceleration_weight = 1.0;
    limits.jerk_weight = 0.01;
    return limits;
}

/** Return elapsed wall time since a steady-clock start [s].
 *
 * Args:
 *     start: Earlier steady-clock time point.
 *
 * Returns:
 *     Nonnegative elapsed duration [s].
 */
[[nodiscard]] double ElapsedS(const Clock::time_point& start) {
    return std::chrono::duration<double>(Clock::now() - start).count();
}

/** Return the least-damped fitted mode for one model-backed axis.
 *
 * Args:
 *     shaper: Initialized synthetic-fixture native controller.
 *     representative_command_rad: Joint state used for modal inference [rad].
 *     axis_index: Model-backed axis to inspect.
 *
 * Returns:
 *     Dominant plant parameters used in the hardware-free simulation.
 */
[[nodiscard]] ModalPlant InferDominantPlant(
    NativeShaper& shaper,
    const Vector& representative_command_rad,
    std::size_t axis_index) {
    RobotState state;
    state.joint_angles_rad = representative_command_rad;
    state.tcp_position_m =
        shaper.ComputeForwardKinematics(representative_command_rad);
    const std::vector<FittedAxisModes> fitted_axes =
        shaper.InferFittedModes(state);
    if (axis_index >= fitted_axes.size() ||
        fitted_axes[axis_index].modes.empty()) {
        throw std::runtime_error(
            "the synthetic fixture did not return a dominant fitted mode");
    }
    const auto dominant = std::min_element(
        fitted_axes[axis_index].modes.begin(),
        fitted_axes[axis_index].modes.end(),
        [](const auto& left, const auto& right) {
            return left.damping_ratio < right.damping_ratio;
        });
    return ModalPlant{
        dominant->natural_frequency_rad_per_s,
        dominant->damping_ratio,
    };
}

/** Return the time of peak positive velocity before final deceleration.
 *
 * Args:
 *     trajectory: Desired command trajectory.
 *     axis_index: Joint column to inspect.
 *
 * Returns:
 *     Deceleration start time [s].
 */
[[nodiscard]] double DecelerationStartTimeS(
    const Trajectory& trajectory,
    std::size_t axis_index) {
    const Eigen::Index axis = static_cast<Eigen::Index>(axis_index);
    Eigen::Index peak_index = 0;
    trajectory.velocities_rad_s.col(axis).maxCoeff(&peak_index);
    return trajectory.time_s(peak_index);
}

/** Compute all controller-off and controller-on simulation metrics.
 *
 * Args:
 *     plant: Synthetic fixture's dominant modal plant.
 *     desired: Unshaped command.
 *     slower_desired: Slower unshaped comparison command.
 *     always_on: Full-trajectory shaped output.
 *     residual_tail: Residual-tail shaped output.
 *     windowed: Reconstructed fixed-window output.
 *     streamed: Sample-by-sample shaped output.
 *     position_change_tolerance_rad: Minimum position delta counted as a
 *         residual-tail command change [rad].
 *
 * Returns:
 *     Complete set of printed parity metrics.
 */
[[nodiscard]] ExampleMetrics ComputeMetrics(
    const ModalPlant& plant,
    const Trajectory& desired,
    const Trajectory& slower_desired,
    const Trajectory& always_on,
    const Trajectory& residual_tail,
    const Trajectory& windowed,
    const Trajectory& streamed,
    double position_change_tolerance_rad) {
    const double residual_start_time_s =
        kMoveDurationS + kDwellDurationS - kResidualWindowDurationS;
    const double slower_residual_start_time_s =
        kSlowerMoveDurationS + kDwellDurationS - kResidualWindowDurationS;
    const double deceleration_start_time_s =
        DecelerationStartTimeS(desired, kShapedAxis);

    const auto simulate = [&plant](const Trajectory& trajectory) {
        return SimulateModalPositionResponse(
            trajectory.time_s,
            trajectory.positions_rad.col(
                static_cast<Eigen::Index>(kShapedAxis)),
            plant);
    };
    return ExampleMetrics{
        plant,
        residual_start_time_s,
        ResidualVibrationRad(
            simulate(desired), kGoalPositionRad, residual_start_time_s),
        ResidualVibrationRad(
            simulate(slower_desired),
            kGoalPositionRad,
            slower_residual_start_time_s),
        deceleration_start_time_s,
        FirstChangeTimeS(
            residual_tail,
            desired,
            kShapedAxis,
            position_change_tolerance_rad,
            deceleration_start_time_s),
        ResidualVibrationRad(
            simulate(always_on), kGoalPositionRad, residual_start_time_s),
        ResidualVibrationRad(
            simulate(residual_tail), kGoalPositionRad, residual_start_time_s),
        ResidualVibrationRad(
            simulate(windowed), kGoalPositionRad, residual_start_time_s),
        ResidualVibrationRad(
            simulate(streamed), kGoalPositionRad, residual_start_time_s),
    };
}

/** Return percent residual reduction relative to the controller-off command.
 *
 * Args:
 *     controller_off_rad: Unshaped residual vibration [rad].
 *     controller_on_rad: Shaped residual vibration [rad].
 *
 * Returns:
 *     Signed percent reduction.
 */
[[nodiscard]] double EffectivenessPercent(
    double controller_off_rad,
    double controller_on_rad) {
    return 100.0 * (controller_off_rad - controller_on_rad) /
           controller_off_rad;
}

/** Require one measured value to match the Python-native expectation.
 *
 * Args:
 *     label: Metric name used in failure diagnostics.
 *     actual: C++ measured value.
 *     expected: Frozen Python-native value.
 *     tolerance: Absolute acceptance tolerance.
 */
void RequireNear(
    const std::string& label,
    double actual,
    double expected,
    double tolerance) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(
            label + " failed Python parity: actual=" +
            std::to_string(actual) + ", expected=" +
            std::to_string(expected));
    }
}

/** Validate structural and numeric parity before reporting success.
 *
 * Args:
 *     desired: Frozen deterministic source command.
 *     always_on: Full-trajectory shaped output.
 *     residual_tail: Residual-tail shaped output.
 *     windowed: Reconstructed fixed-window output.
 *     streamed: Sample-by-sample shaped output.
 *     metrics: Hardware-free response metrics.
 *     expected: Canonical JSON baseline expectations.
 */
void ValidateResults(
    const Trajectory& desired,
    const Trajectory& always_on,
    const Trajectory& residual_tail,
    const Trajectory& windowed,
    const Trajectory& streamed,
    const ExampleMetrics& metrics,
    const ExpectedMetrics& expected) {
    const auto expected_samples =
        static_cast<std::size_t>(desired.positions_rad.rows());
    for (const auto& item : std::vector<std::pair<std::string, const Trajectory*>>{
             {"desired", &desired},
             {"Example 1", &always_on},
             {"Example 2", &residual_tail},
             {"Example 3", &windowed},
             {"Example 4", &streamed}}) {
        ValidateTrajectory(*item.second, kNumJoints, kSampleTimeS, item.first);
    }
    for (const auto& item : std::vector<std::pair<std::string, const Trajectory*>>{
             {"Example 1", &always_on},
             {"Example 3", &windowed},
             {"Example 4", &streamed}}) {
        if (static_cast<std::size_t>(item.second->positions_rad.rows()) !=
            expected_samples) {
            throw std::runtime_error(
                item.first + " did not emit exactly one result per source cycle");
        }
    }
    if (!always_on.time_s.isApprox(
            windowed.time_s, expected.time_tolerance_s) ||
        !always_on.positions_rad.isApprox(
            windowed.positions_rad, expected.position_tolerance_rad) ||
        !always_on.velocities_rad_s.isApprox(
            windowed.velocities_rad_s, expected.velocity_tolerance_rad_s) ||
        !always_on.accelerations_rad_s2.isApprox(
            windowed.accelerations_rad_s2,
            expected.acceleration_tolerance_rad_s2)) {
        throw std::runtime_error(
            "fixed-window output does not reconstruct full-trajectory shaping");
    }

    RequireNear(
        "dominant natural frequency",
        metrics.plant.natural_frequency_rad_s,
        expected.dominant_natural_frequency_rad_s,
        expected.metric_tolerance);
    RequireNear(
        "dominant damping ratio",
        metrics.plant.damping_ratio,
        expected.dominant_damping_ratio,
        expected.metric_tolerance);
    RequireNear(
        "unshaped residual",
        metrics.unshaped_residual_rad,
        expected.unshaped_residual_rad,
        expected.metric_tolerance);
    RequireNear(
        "slower unshaped residual",
        metrics.slower_unshaped_residual_rad,
        expected.slower_unshaped_residual_rad,
        expected.metric_tolerance);
    RequireNear(
        "always-on residual",
        metrics.always_on_residual_rad,
        expected.always_on_residual_rad,
        expected.metric_tolerance);
    RequireNear(
        "residual-tail residual",
        metrics.residual_tail_residual_rad,
        expected.residual_tail_residual_rad,
        expected.metric_tolerance);
    RequireNear(
        "windowed residual",
        metrics.windowed_residual_rad,
        expected.windowed_residual_rad,
        expected.metric_tolerance);
    RequireNear(
        "streamed residual",
        metrics.streamed_residual_rad,
        expected.streamed_residual_rad,
        expected.metric_tolerance);
    RequireNear(
        "residual start time",
        metrics.residual_start_time_s,
        expected.residual_start_time_s,
        expected.time_tolerance_s);
    RequireNear(
        "deceleration start time",
        metrics.deceleration_start_time_s,
        expected.deceleration_start_time_s,
        expected.time_tolerance_s);
    RequireNear(
        "residual change start time",
        metrics.residual_change_start_time_s,
        expected.residual_change_start_time_s,
        expected.time_tolerance_s);
    RequireNear(
        "sample time",
        kSampleTimeS,
        expected.sample_time_s,
        expected.time_tolerance_s);
    if (metrics.always_on_residual_rad >= metrics.unshaped_residual_rad ||
        metrics.residual_tail_residual_rad >= metrics.unshaped_residual_rad ||
        metrics.windowed_residual_rad >= metrics.unshaped_residual_rad ||
        metrics.streamed_residual_rad >= metrics.unshaped_residual_rad) {
        throw std::runtime_error(
            "a controller-on path did not improve the controller-off residual");
    }
}

/** Print metrics and execution timing for customer-visible evidence.
 *
 * Args:
 *     metrics: Validated hardware-free response metrics.
 *     execution_times_s: Example labels and elapsed execution times [s].
 */
void PrintResults(
    const ExampleMetrics& metrics,
    const std::vector<std::pair<std::string, double>>& execution_times_s) {
    std::cout << std::fixed;
    std::cout << "Covalent Shaper C++ example complete.\n";
    std::cout << "Package: reforge-core-shaper "
              << REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION << '\n';
    std::cout << "Model: synthetic native example fixture (2 axes)\n";
    std::cout << "URDF: repository test_robot.urdf\n";
    std::cout << "Joint order: joint0, joint1, joint2, joint3, joint4, joint5\n";
    std::cout << "Units: time [s], position [rad], velocity [rad/s], "
                 "acceleration [rad/s^2]\n";
    for (const auto& item : execution_times_s) {
        std::cout << item.first << " execution time: " << std::setprecision(6)
                  << item.second << " s\n";
    }
    std::cout << "Dominant simulation mode: wn=" << std::setprecision(3)
              << metrics.plant.natural_frequency_rad_s << " rad/s, zeta="
              << std::setprecision(4) << metrics.plant.damping_ratio << '\n';
    std::cout << "Residual metric starts at t=" << std::setprecision(3)
              << metrics.residual_start_time_s << " s\n";
    std::cout << "Controller-off residual vibration: " << std::setprecision(6)
              << metrics.unshaped_residual_rad << " rad\n";
    std::cout << "Slower controller-off residual vibration: "
              << metrics.slower_unshaped_residual_rad << " rad after a "
              << std::setprecision(1) << kSlowerMoveDurationS << " s move\n";
    std::cout << "Command deceleration starts at t=" << std::setprecision(3)
              << metrics.deceleration_start_time_s << " s\n";
    std::cout << "Residual Shaper starts changing the command at t="
              << metrics.residual_change_start_time_s << " s\n";
    for (const auto& item :
         std::vector<std::pair<std::string, double>>{
             {"Example 1 always-on", metrics.always_on_residual_rad},
             {"Example 2 residual-tail", metrics.residual_tail_residual_rad},
             {"Example 3 fixed-window", metrics.windowed_residual_rad},
             {"Example 4 sample-stream", metrics.streamed_residual_rad}}) {
        std::cout << item.first << " controller-on residual vibration: "
                  << std::setprecision(6) << item.second << " rad ("
                  << std::setprecision(2)
                  << EffectivenessPercent(
                         metrics.unshaped_residual_rad, item.second)
                  << "% reduction)\n";
    }
    std::cout << "Validation: PASS\n";
}

/** Write deterministic metrics JSON for automated parity checks.
 *
 * Args:
 *     output_path: Destination JSON path.
 *     metrics: Validated hardware-free response metrics.
 */
void WriteMetricsJson(
    const std::filesystem::path& output_path,
    const ExampleMetrics& metrics) {
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error(
            "could not open metrics artifact: " + output_path.string());
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"model_identity\": \"synthetic-example-fixture-v1\",\n"
           << "  \"num_axes\": " << kNumAxes << ",\n"
           << "  \"num_joints\": " << kNumJoints << ",\n"
           << "  \"sample_time_s\": " << kSampleTimeS << ",\n"
           << "  \"dominant_natural_frequency_rad_s\": "
           << metrics.plant.natural_frequency_rad_s << ",\n"
           << "  \"dominant_damping_ratio\": "
           << metrics.plant.damping_ratio << ",\n"
           << "  \"residual_start_time_s\": "
           << metrics.residual_start_time_s << ",\n"
           << "  \"unshaped_residual_rad\": "
           << metrics.unshaped_residual_rad << ",\n"
           << "  \"slower_unshaped_residual_rad\": "
           << metrics.slower_unshaped_residual_rad << ",\n"
           << "  \"deceleration_start_time_s\": "
           << metrics.deceleration_start_time_s << ",\n"
           << "  \"residual_change_start_time_s\": "
           << metrics.residual_change_start_time_s << ",\n"
           << "  \"always_on_residual_rad\": "
           << metrics.always_on_residual_rad << ",\n"
           << "  \"residual_tail_residual_rad\": "
           << metrics.residual_tail_residual_rad << ",\n"
           << "  \"windowed_residual_rad\": "
           << metrics.windowed_residual_rad << ",\n"
           << "  \"streamed_residual_rad\": "
           << metrics.streamed_residual_rad << "\n"
           << "}\n";
    if (!output) {
        throw std::runtime_error(
            "failed while writing metrics artifact: " + output_path.string());
    }
}

}  // namespace

/** Run all four hardware-free Covalent Shaper examples.
 *
 * Args:
 *     argument_count: Number of command-line arguments.
 *     arguments: Optional asset and output directory arguments.
 *
 * Returns:
 *     Zero only when all examples and validations pass.
 */
int main(int argument_count, char** arguments) {
    try {
        const Options options = ParseOptions(argument_count, arguments);
        const ExpectedMetrics expected =
            LoadExpectedMetrics(options.baseline_manifest);
        const Vector start_position_rad = Vector::Zero(kNumJoints);
        Vector goal_position_rad = Vector::Zero(kNumJoints);
        goal_position_rad(static_cast<Eigen::Index>(kShapedAxis)) =
            kGoalPositionRad;
        const Trajectory desired = GeneratePointToPointTrajectory(
            start_position_rad,
            goal_position_rad,
            kSampleTimeS,
            kMoveDurationS,
            kDwellDurationS);
        const Trajectory slower_desired = GeneratePointToPointTrajectory(
            start_position_rad,
            goal_position_rad,
            kSampleTimeS,
            kSlowerMoveDurationS,
            kDwellDurationS);
        std::vector<std::pair<std::string, double>> execution_times_s;

        // =====================================================================
        // Start Example 1: Shape a complete trajectory with Shaper enabled from
        // the start.
        // =====================================================================
        NativeShaper always_on_shaper(
            MakeBackendConfiguration(options.assets_directory));
        ProcessTrajectoryRequest always_on_request;
        always_on_request.input.position_rad = desired.positions_rad;
        always_on_request.input.velocity_rad_per_s = desired.velocities_rad_s;
        always_on_request.input.acceleration_rad_per_s2 =
            desired.accelerations_rad_s2;
        always_on_request.input.time_s = desired.time_s;
        always_on_request.vibration_shaping_weight = kShaperEnabledWeight;
        always_on_request.residual_shaping_strategy = std::nullopt;
        always_on_request.finalize_tail = false;
        const auto example_1_start = Clock::now();
        Trajectory always_on = ToTrajectory(
            always_on_shaper.ProcessTrajectory(always_on_request));
        execution_times_s.emplace_back(
            "Example 1", ElapsedS(example_1_start));
        // In a robot application, send the shaped trajectory through the robot
        // SDK. This hardware-free example intentionally performs no robot I/O.
        // =====================================================================
        // End Example 1.
        // =====================================================================

        // =====================================================================
        // Start Example 2: Shape only the residual tail of a planned trajectory.
        // =====================================================================
        const ResidualSwitchLimits speed_first_switch_limits =
            MakeSpeedFirstSwitchLimits();
        NativeShaper residual_offline_shaper(
            MakeBackendConfiguration(options.assets_directory));
        ProcessTrajectoryRequest residual_request;
        residual_request.input.position_rad = desired.positions_rad;
        residual_request.input.velocity_rad_per_s = desired.velocities_rad_s;
        residual_request.input.acceleration_rad_per_s2 =
            desired.accelerations_rad_s2;
        residual_request.input.time_s = desired.time_s;
        residual_request.vibration_shaping_weight = kShaperEnabledWeight;
        residual_request.residual_shaping_strategy =
            ResidualShapingStrategy::kAlignedTail;
        residual_request.residual_switch_limits = speed_first_switch_limits;
        // This margin preserves a positive aligned-tail transition for the
        // deterministic synthetic fixture.
        residual_request.residual_transition_margin_s =
            kResidualTransitionMarginS;
        residual_request.finalize_tail = false;
        const auto example_2_start = Clock::now();
        Trajectory residual_tail = ToTrajectory(
            residual_offline_shaper.ProcessTrajectory(residual_request));
        execution_times_s.emplace_back(
            "Example 2", ElapsedS(example_2_start));
        // In a robot application, send the shaped trajectory through the robot
        // SDK. This hardware-free example intentionally performs no robot I/O.
        // =====================================================================
        // End Example 2.
        // =====================================================================

        // =====================================================================
        // Start Example 3: Shape a complete trajectory in fixed-size windows.
        // =====================================================================
        NativeShaper windowed_shaper(
            MakeBackendConfiguration(options.assets_directory));
        ShaperWindowRequest window_request;
        window_request.command_rad = desired.positions_rad;
        window_request.command_velocity_rad_s = desired.velocities_rad_s;
        window_request.command_acceleration_rad_s2 =
            desired.accelerations_rad_s2;
        window_request.time_s = desired.time_s;
        window_request.vibration_shaping_weight = kShaperEnabledWeight;
        window_request.residual_shaping_strategy = std::nullopt;
        window_request.window_s = kWindowDurationS;
        window_request.prefill_windows = 1;
        window_request.finalize_tail = false;
        const auto example_3_start = Clock::now();
        auto windowed_buffer =
            windowed_shaper.CreateWindowedBuffer(window_request);
        static_cast<void>(windowed_buffer->FillAvailable());
        std::vector<ShapedWindow> shaped_windows;
        while (windowed_buffer->HasNext()) {
            auto shaped_window = windowed_buffer->PopWindow();
            if (!shaped_window.has_value()) {
                static_cast<void>(windowed_buffer->FillAvailable(1));
                continue;
            }
            ShapedTrajectory shaped_trajectory{
                std::move(shaped_window->positions),
                std::move(shaped_window->velocities),
                std::move(shaped_window->accelerations),
                std::move(shaped_window->time_s),
            };
            shaped_windows.push_back(ShapedWindow{
                ToTrajectory(std::move(shaped_trajectory)),
                shaped_window->start_index,
                shaped_window->stop_index,
                shaped_window->is_tail,
            });
        }
        windowed_buffer->Close();
        Trajectory windowed = ConcatenateWindowedOutputs(
            shaped_windows,
            static_cast<std::size_t>(desired.positions_rad.rows()));
        execution_times_s.emplace_back(
            "Example 3", ElapsedS(example_3_start));
        // Each popped window would be sent through the robot SDK in timestamp
        // order. This hardware-free example only reconstructs the output.
        // =====================================================================
        // End Example 3.
        // =====================================================================

        // =====================================================================
        // Start Example 4: Shape one command sample at a time in an online loop.
        // =====================================================================
        NativeShaper streaming_shaper(MakeBackendConfiguration(
            options.assets_directory,
            SharedImpulsePolicy::kPerAxis,
            false));
        Trajectory streamed;
        const Eigen::Index stream_samples = desired.time_s.size();
        streamed.time_s = desired.time_s;
        streamed.positions_rad.resize(stream_samples, kNumJoints);
        streamed.velocities_rad_s.resize(stream_samples, kNumJoints);
        streamed.accelerations_rad_s2.resize(stream_samples, kNumJoints);
        const auto example_4_start = Clock::now();
        for (Eigen::Index sample = 0; sample < stream_samples; ++sample) {
            const Trajectory command = GeneratePointToPointSample(
                start_position_rad,
                goal_position_rad,
                streamed.time_s(sample),
                kMoveDurationS);
            ProcessSampleRequest request;
            request.position_rad = command.positions_rad.row(0).transpose();
            request.velocity_rad_per_s =
                command.velocities_rad_s.row(0).transpose();
            request.acceleration_rad_per_s2 =
                command.accelerations_rad_s2.row(0).transpose();
            request.vibration_shaping_weight = kShaperEnabledWeight;
            auto shaped_sample = streaming_shaper.ProcessSample(request);
            streamed.positions_rad.row(sample) =
                shaped_sample.position_rad.transpose();
            streamed.velocities_rad_s.row(sample) =
                shaped_sample.velocity_rad_s.transpose();
            streamed.accelerations_rad_s2.row(sample) =
                shaped_sample.acceleration_rad_s2.transpose();
            // A real control loop would send this single shaped sample through
            // the robot SDK. This example only records it for validation.
        }
        execution_times_s.emplace_back(
            "Example 4", ElapsedS(example_4_start));
        // =====================================================================
        // End Example 4.
        // =====================================================================

        const ModalPlant plant = InferDominantPlant(
            always_on_shaper, start_position_rad, kShapedAxis);
        const ExampleMetrics metrics = ComputeMetrics(
            plant,
            desired,
            slower_desired,
            always_on,
            residual_tail,
            windowed,
            streamed,
            expected.position_tolerance_rad);
        if (options.output_directory.has_value()) {
            // Preserve artifacts even when a frozen parity check fails so a
            // customer can diagnose the exact mismatching field.
            WriteMetricsJson(
                *options.output_directory / "metrics.json", metrics);
            WriteTrajectoryCsv(
                *options.output_directory / "desired.csv",
                desired,
                kJointOrder);
            WriteTrajectoryCsv(
                *options.output_directory / "example_1_always_on.csv",
                always_on,
                kJointOrder);
            WriteTrajectoryCsv(
                *options.output_directory / "example_2_residual_tail.csv",
                residual_tail,
                kJointOrder);
            WriteTrajectoryCsv(
                *options.output_directory / "example_3_fixed_windows.csv",
                windowed,
                kJointOrder);
            WriteTrajectoryCsv(
                *options.output_directory / "example_4_sample_stream.csv",
                streamed,
                kJointOrder);
        }
        ValidateResults(
            desired,
            always_on,
            residual_tail,
            windowed,
            streamed,
            metrics,
            expected);
        const auto plot_data = PreparePlotData(
            desired,
            slower_desired,
            always_on,
            residual_tail,
            windowed,
            streamed,
            plant,
            kShapedAxis,
            metrics.residual_change_start_time_s);
        RenderPlots(
            plot_data,
            PlotOptions{
                options.headless,
                options.output_directory,
                options.renderer_script,
            });
        PrintResults(metrics, execution_times_s);

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Covalent Shaper C++ example failed: " << error.what()
                  << '\n';
        return 1;
    }
}
