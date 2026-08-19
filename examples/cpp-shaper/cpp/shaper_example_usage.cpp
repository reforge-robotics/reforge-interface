#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "example_utility.hpp"
#include "example_plot_data.hpp"
#include "example_plotting.hpp"
#include "reforge_core/control/runtime/modal_inference.hpp"
#include "reforge_core/control/shaper/backend/backend_configuration.hpp"
#include "reforge_core/control/shaper/backend/backend_requests.hpp"
#include "reforge_core/control/shaper/backend/native_shaper.hpp"
#include "reforge_core/control/shaper/config.hpp"
#include "reforge_core/control/shaper/windowing.hpp"

#ifndef REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION
#define REFORGE_SHAPER_EXAMPLE_PACKAGE_VERSION "unknown"
#endif

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
constexpr std::size_t kNumAxes = 3;
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
        executable_path.parent_path() / "validation" / "expected_metrics.json";
    for (int index = 1; index < argument_count; ++index) {
        const std::string_view argument(arguments[index]);
        if (argument == "--assets-dir" && index + 1 < argument_count) {
            options.assets_directory = arguments[++index];
        } else if (argument == "--baseline" && index + 1 < argument_count) {
            options.baseline_manifest = arguments[++index];
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
    if (options.headless && !options.output_directory.has_value()) {
        options.output_directory = executable_path.parent_path() / "figures";
    }
    return options;
}

/** Read one required numeric member from a strict baseline JSON document.
 *
 * The example owns no general JSON API. This reader accepts the reviewed
 * manifest's numeric schema and rejects missing, duplicate, nonnumeric, or
 * non-finite members without adding a consumer-visible JSON dependency.
 *
 * Args:
 *     document: Complete baseline manifest text.
 *     member_name: Required JSON member name, unique across the manifest.
 *
 * Returns:
 *     Finite numeric member value.
 */
[[nodiscard]] double ReadRequiredNumber(
    const std::string& document,
    const std::string& member_name) {
    const std::regex member_pattern(
        "\\\"" + member_name +
        "\\\"[[:space:]]*:[[:space:]]*"
        "([-+]?[0-9]+(\\.[0-9]*)?([eE][-+]?[0-9]+)?)");
    const std::sregex_iterator begin(
        document.begin(), document.end(), member_pattern);
    const std::sregex_iterator end;
    if (begin == end) {
        throw std::invalid_argument(
            "baseline manifest is missing numeric member: " + member_name);
    }
    const auto next = std::next(begin);
    if (next != end) {
        throw std::invalid_argument(
            "baseline manifest repeats numeric member: " + member_name);
    }
    const double value = std::stod((*begin)[1].str());
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
    const std::string document{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    const ExpectedMetrics expected{
        ReadRequiredNumber(document, "sample_time_s"),
        ReadRequiredNumber(document, "metric_tolerance"),
        ReadRequiredNumber(document, "time_tolerance_s"),
        ReadRequiredNumber(document, "position_tolerance_rad"),
        ReadRequiredNumber(document, "velocity_tolerance_rad_s"),
        ReadRequiredNumber(document, "acceleration_tolerance_rad_s2"),
        ReadRequiredNumber(document, "dominant_natural_frequency_rad_s"),
        ReadRequiredNumber(document, "dominant_damping_ratio"),
        ReadRequiredNumber(document, "residual_start_time_s"),
        ReadRequiredNumber(document, "unshaped_residual_rad"),
        ReadRequiredNumber(document, "slower_unshaped_residual_rad"),
        ReadRequiredNumber(document, "deceleration_start_time_s"),
        ReadRequiredNumber(document, "residual_change_start_time_s"),
        ReadRequiredNumber(document, "always_on_residual_rad"),
        ReadRequiredNumber(document, "residual_tail_residual_rad"),
        ReadRequiredNumber(document, "windowed_residual_rad"),
        ReadRequiredNumber(document, "streamed_residual_rad")};
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

/** Construct the explicit Standard Bots native backend configuration.
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
    const std::filesystem::path model_directory = assets_directory / "model";
    const std::filesystem::path urdf_filepath =
        assets_directory / "modelone.urdf";
    if (!std::filesystem::is_regular_file(
            model_directory / "model_bundle.json") ||
        !std::filesystem::is_regular_file(
            model_directory / "shaper_models.native.json") ||
        !std::filesystem::is_regular_file(urdf_filepath)) {
        throw std::invalid_argument(
            "assets directory must contain the Standard Bots native model "
            "bundle and matching modelone.urdf: " +
            assets_directory.string());
    }

    BackendConfiguration configuration;
    configuration.sample_time_s = kSampleTimeS;
    configuration.model_directory = model_directory;
    configuration.urdf_filepath = urdf_filepath;
    configuration.num_axes = kNumAxes;
    configuration.side_length_m = 0.0007;
    configuration.base_height_m = 0.364;
    configuration.num_joints = kNumJoints;
    configuration.probability_threshold = 0.5;
    configuration.shared_impulse_policy = shared_impulse_policy;
    configuration.shared_impulse_shapes_all_joints =
        shared_impulse_shapes_all_joints;
    configuration.residual_path_scope = ResidualPathScope::kShapedAxes;
    configuration.aligned_tail_require_zero_acceleration_region = false;
    configuration.train_base_angles_rad = Vector::Zero(1);
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
 *     Explicit six-joint Standard Bots switch limits.
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
 *     shaper: Initialized Standard Bots native controller.
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
            "the Standard Bots bundle did not return a dominant fitted mode");
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
 *     plant: Standard Bots dominant modal plant.
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

/** Require one measured value to match the Standard Bots Python expectation.
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
            label + " failed Standard Bots parity: actual=" +
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
    std::cout << "Model: Standard Bots native bundle (reforge-interface "
                 "22dc942, 3 axes)\n";
    std::cout << "URDF: matching modelone.urdf\n";
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
           << "  \"model_identity\": \"standard-bots-22dc942\",\n"
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

/** Run all four hardware-free Standard Bots Covalent Shaper examples.
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
        // source-marker: example-1-full-known-trajectory
        // source-marker: configure-controller
        NativeShaper always_on_shaper(
            MakeBackendConfiguration(options.assets_directory));
        // source-marker: prepare-input
        ProcessTrajectoryRequest always_on_request;
        always_on_request.input.position_rad = desired.positions_rad;
        always_on_request.input.velocity_rad_per_s = desired.velocities_rad_s;
        always_on_request.input.acceleration_rad_per_s2 =
            desired.accelerations_rad_s2;
        always_on_request.input.time_s = desired.time_s;
        always_on_request.vibration_shaping_weight = kShaperEnabledWeight;
        always_on_request.residual_shaping_strategy = std::nullopt;
        always_on_request.finalize_tail = false;
        // source-marker: shape-command
        const auto example_1_start = Clock::now();
        Trajectory always_on = ToTrajectory(
            always_on_shaper.ProcessTrajectory(always_on_request));
        execution_times_s.emplace_back(
            "Example 1", ElapsedS(example_1_start));
        // source-marker: robot-output-placeholder
        // In a robot application, send the shaped trajectory through the robot
        // SDK. This hardware-free example intentionally performs no robot I/O.
        // =====================================================================
        // End Example 1.
        // =====================================================================

        // =====================================================================
        // Start Example 2: Shape only the residual tail of a planned trajectory.
        // =====================================================================
        // source-marker: example-2-residual-tail
        // source-marker: configure-controller
        const ResidualSwitchLimits speed_first_switch_limits =
            MakeSpeedFirstSwitchLimits();
        NativeShaper residual_offline_shaper(
            MakeBackendConfiguration(options.assets_directory));
        // source-marker: prepare-input
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
        // The Standard Bots bundle qualifies a real modal-aware candidate at
        // this margin. Zero truncates output before the frozen metric window;
        // margins of 0.06 s or more fall back with no positive alignment.
        residual_request.residual_transition_margin_s =
            kResidualTransitionMarginS;
        residual_request.finalize_tail = false;
        // source-marker: shape-command
        const auto example_2_start = Clock::now();
        Trajectory residual_tail = ToTrajectory(
            residual_offline_shaper.ProcessTrajectory(residual_request));
        execution_times_s.emplace_back(
            "Example 2", ElapsedS(example_2_start));
        // source-marker: robot-output-placeholder
        // In a robot application, send the shaped trajectory through the robot
        // SDK. This hardware-free example intentionally performs no robot I/O.
        // =====================================================================
        // End Example 2.
        // =====================================================================

        // =====================================================================
        // Start Example 3: Shape a complete trajectory in fixed-size windows.
        // =====================================================================
        // source-marker: example-3-fixed-windows
        // source-marker: configure-controller
        NativeShaper windowed_shaper(
            MakeBackendConfiguration(options.assets_directory));
        // source-marker: prepare-input
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
        // source-marker: shape-command
        const auto example_3_start = Clock::now();
        auto windowed_buffer =
            windowed_shaper.CreateWindowedBuffer(window_request);
        static_cast<void>(windowed_buffer->FillAvailable());
        std::vector<ShapedWindow> shaped_windows;
        // source-marker: drain-output
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
        // source-marker: robot-output-placeholder
        // Each popped window would be sent through the robot SDK in timestamp
        // order. This hardware-free example only reconstructs the output.
        // =====================================================================
        // End Example 3.
        // =====================================================================

        // =====================================================================
        // Start Example 4: Shape one command sample at a time in an online loop.
        // =====================================================================
        // source-marker: example-4-sample-stream
        // source-marker: configure-controller
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
        // source-marker: prepare-input
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
            // source-marker: shape-command
            auto shaped_sample = streaming_shaper.ProcessSample(request);
            streamed.positions_rad.row(sample) =
                shaped_sample.position_rad.transpose();
            streamed.velocities_rad_s.row(sample) =
                shaped_sample.velocity_rad_s.transpose();
            streamed.accelerations_rad_s2.row(sample) =
                shaped_sample.acceleration_rad_s2.transpose();
            // source-marker: robot-output-placeholder
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
            PlotOptions{options.headless, options.output_directory});
        PrintResults(metrics, execution_times_s);

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Covalent Shaper C++ example failed: " << error.what()
                  << '\n';
        return 1;
    }
}
