#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "plotting/matplotlib_process_plotter.hpp"
#include "reforge_core/joint_tracker/joint_tracker.hpp"

namespace {

using reforge::joint_tracker::JointSample;
using reforge::joint_tracker::JointTracker;
using reforge::joint_tracker::JointTrackerConfig;
using reforge::joint_tracker::JointTrackerOptimizerOptions;
using reforge::joint_tracker::JointTrackerStream;
using reforge::joint_tracker::JointTrackerWindow;
using reforge::joint_tracker::JointTrajectory;

constexpr double kSampleTimeS = 0.01;
constexpr std::size_t kNumAxes = 6;
constexpr std::size_t kNumJoints = 6;
constexpr std::array<int, kNumAxes> kModeledAxes = {0, 1, 2, 3, 4, 5};
constexpr double kDwellDurationS = 0.75;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kCobotJointPositionLimitRad = 350.0 * kPi / 180.0;
constexpr double kCobotMaxJointSpeedRadS = 180.0 * kPi / 180.0;
constexpr double kCobotMaxJointAccelerationRadS2 = 720.0 * kPi / 180.0;
constexpr double kQuinticMaxVelocityFactor = 1.875;
constexpr double kQuinticMaxAccelerationFactor = 10.0 * 1.7320508075688772 / 3.0;
constexpr double kMinTbiSegmentDurationS = 0.55;
constexpr std::size_t kInitialPlannerChunkSamples = 1;
constexpr std::size_t kAppendPlannerChunkSamples = 1;
constexpr std::size_t kLowWatermarkSamples = 10;
constexpr std::array<std::size_t, 3> kDelayAfterSourceSamples = {56, 111, 170};
constexpr std::array<std::size_t, 3> kDelayWindows = {3, 3, 4};
constexpr std::size_t kOnlinePlannerUpdateSamples = 1;
constexpr int kResponseIntegrationSubsteps = 20;
// n_apply_scale tunes how many optimized samples Joint Tracker makes available
// per window. Smaller positive values make shorter windows; this example uses
// one-sample windows.
constexpr double kSingleSampleNApplyScale = 1.0e-9;

struct Options final {
    bool no_plots = false;
    std::filesystem::path metrics_json;
    std::filesystem::path save_plots;
};

struct Matrix3 final {
    std::array<std::array<double, 3>, 3> value{};
};

struct Vector3 final {
    std::array<double, 3> value{};
};

struct TransferFunctionModel final {
    std::vector<double> numerator;
    std::vector<double> denominator;
};

struct StateSpaceModel final {
    Matrix3 a;
    Vector3 b;
    Vector3 c;
    double d = 0.0;
};

struct TrajectoryArrays final {
    std::vector<double> time_s;
    std::vector<std::vector<double>> positions_rad;
    std::vector<std::vector<double>> velocities_rad_s;
    std::vector<std::vector<double>> accelerations_rad_s2;
    double motion_end_time_s = 0.0;
};

struct WindowStreamRecord final {
    std::size_t start_index = 0;
    std::size_t stop_index = 0;
    std::size_t sample_count = 0;
    double pop_wait_time_s = 0.0;
    std::size_t n_apply = 0;
    std::size_t solve_index = 0;
    std::size_t solve_start_index = 0;
    std::size_t solve_stop_index = 0;
    std::size_t terminal_padding_samples = 0;
    std::size_t hold_samples_added = 0;
};

struct WindowMetricsSummary final {
    std::size_t min_window_samples = 0;
    std::size_t max_window_samples = 0;
    std::size_t min_n_apply_samples = 0;
    std::size_t max_n_apply_samples = 0;
};

struct StreamResult final {
    std::vector<double> desired_time_s;
    std::vector<std::vector<double>> desired_positions_rad;
    std::vector<double> optimized_time_s;
    std::vector<std::vector<double>> optimized_positions_rad;
    std::vector<WindowStreamRecord> window_records;
    std::size_t appended_source_samples = 0;
    std::size_t terminal_padding_samples = 0;
    std::size_t hold_samples_added = 0;
    std::vector<std::size_t> planner_update_sample_counts;
};

struct PlannerSimulationState final {
    const std::vector<std::vector<double>>* source_positions_rad = nullptr;
    std::size_t append_stop_index = 0;
    std::size_t append_chunk_samples = 0;
    std::size_t low_watermark_samples = 0;
    std::array<std::size_t, 3> delay_thresholds{};
    std::array<std::size_t, 3> delay_windows{};
    std::size_t next_delay_index = 0;
    std::size_t remaining_delay_windows = 0;
    bool has_finished_appending = false;
    std::vector<std::size_t> append_batch_sizes;
};

struct PlannerDecision final {
    std::optional<std::pair<std::size_t, std::size_t>> append_range;
    bool should_finish = false;
};

struct TrackingSummary final {
    std::vector<double> unoptimized_rms_rad;
    std::vector<double> example_1_rms_rad;
    std::vector<double> streamed_rms_rad;
};

struct ExampleResults final {
    TrajectoryArrays desired;
    std::vector<double> example_1_time_s;
    std::vector<std::vector<double>> example_1_positions_rad;
    StreamResult example_2a_stream;
    StreamResult example_2b_stream;
    std::vector<double> delayed_reference_time_s;
    std::vector<std::vector<double>> delayed_reference_positions_rad;
    TrackingSummary tracking_summary;
    double example_2a_max_difference_rad = 0.0;
    double example_2b_max_difference_rad = 0.0;
};

[[nodiscard]] Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--no-plots") {
            options.no_plots = true;
        } else if (argument == "--metrics-json" || argument == "--save-plots") {
            if (++index >= argc) {
                throw std::invalid_argument(argument + " requires a path.");
            }
            if (argument == "--metrics-json") {
                options.metrics_json = argv[index];
            } else {
                options.save_plots = argv[index];
            }
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: joint_tracker_example_usage [--no-plots] "
                   "[--metrics-json PATH] [--save-plots DIRECTORY]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("Unknown argument: " + argument);
        }
    }
    return options;
}

[[nodiscard]] std::vector<double> Zeros(std::size_t count) {
    return std::vector<double>(count, 0.0);
}

[[nodiscard]] std::vector<double> Add(
    const std::vector<double>& first,
    const std::vector<double>& second) {
    if (first.size() != second.size()) {
        throw std::invalid_argument("Vector sizes must match.");
    }
    std::vector<double> result(first.size(), 0.0);
    for (std::size_t index = 0; index < first.size(); ++index) {
        result[index] = first[index] + second[index];
    }
    return result;
}

[[nodiscard]] std::vector<double> Subtract(
    const std::vector<double>& first,
    const std::vector<double>& second) {
    if (first.size() != second.size()) {
        throw std::invalid_argument("Vector sizes must match.");
    }
    std::vector<double> result(first.size(), 0.0);
    for (std::size_t index = 0; index < first.size(); ++index) {
        result[index] = first[index] - second[index];
    }
    return result;
}

[[nodiscard]] std::vector<double> Scale(
    const std::vector<double>& values,
    double scale) {
    std::vector<double> result(values.size(), 0.0);
    for (std::size_t index = 0; index < values.size(); ++index) {
        result[index] = values[index] * scale;
    }
    return result;
}

[[nodiscard]] TrajectoryArrays GenerateTbiTrajectory() {
    const std::array<std::array<double, kNumAxes>, 5> modeled_waypoints_rad = {{
        {{0.00, 0.00, 0.00, 0.00, 0.00, 0.00}},
        {{0.45, -0.25, 0.15, -0.18, 0.12, -0.10}},
        {{-0.20, 0.32, -0.12, 0.20, -0.16, 0.14}},
        {{0.55, 0.10, 0.18, -0.25, 0.20, -0.18}},
        {{0.10, -0.35, 0.00, 0.12, -0.10, 0.08}},
    }};

    std::vector<std::vector<double>> waypoints_rad;
    waypoints_rad.reserve(modeled_waypoints_rad.size());
    for (const auto& modeled_waypoint_rad : modeled_waypoints_rad) {
        std::vector<double> waypoint_rad(kNumJoints, 0.0);
        for (std::size_t axis_index = 0; axis_index < kNumAxes; ++axis_index) {
            if (std::abs(modeled_waypoint_rad[axis_index]) >
                kCobotJointPositionLimitRad) {
                throw std::runtime_error(
                    "TBI waypoints exceed the joint position limit.");
            }
            waypoint_rad[axis_index] = modeled_waypoint_rad[axis_index];
        }
        waypoints_rad.push_back(std::move(waypoint_rad));
    }

    TrajectoryArrays trajectory;
    double elapsed_time_s = 0.0;
    bool has_previous_segment = false;

    // Each segment uses a quintic blend so position, velocity, and acceleration
    // start and end smoothly at every waypoint.
    for (std::size_t segment_index = 0;
         segment_index + 1 < waypoints_rad.size();
         ++segment_index) {
        const std::vector<double>& start_position_rad =
            waypoints_rad[segment_index];
        const std::vector<double>& end_position_rad =
            waypoints_rad[segment_index + 1];
        const std::vector<double> delta_rad =
            Subtract(end_position_rad, start_position_rad);
        double max_abs_delta_rad = 0.0;
        for (const double value : delta_rad) {
            max_abs_delta_rad = std::max(max_abs_delta_rad, std::abs(value));
        }
        if (max_abs_delta_rad == 0.0) {
            continue;
        }

        const double speed_limited_duration_s =
            kQuinticMaxVelocityFactor * max_abs_delta_rad /
            kCobotMaxJointSpeedRadS;
        const double acceleration_limited_duration_s = std::sqrt(
            kQuinticMaxAccelerationFactor * max_abs_delta_rad /
            kCobotMaxJointAccelerationRadS2);
        const double required_duration_s = std::max(
            {kMinTbiSegmentDurationS,
             speed_limited_duration_s,
             acceleration_limited_duration_s});
        const std::size_t segment_sample_count = static_cast<std::size_t>(
            std::ceil(required_duration_s / kSampleTimeS));
        const double segment_duration_s =
            static_cast<double>(segment_sample_count) * kSampleTimeS;

        for (std::size_t sample_index = 0;
             sample_index <= segment_sample_count;
             ++sample_index) {
            if (has_previous_segment && sample_index == 0) {
                continue;
            }
            const double local_time_s =
                static_cast<double>(sample_index) * kSampleTimeS;
            const double phase = local_time_s / segment_duration_s;
            const double phase2 = phase * phase;
            const double phase3 = phase2 * phase;
            const double phase4 = phase3 * phase;
            const double phase5 = phase4 * phase;
            const double blend =
                10.0 * phase3 - 15.0 * phase4 + 6.0 * phase5;
            const double blend_dot =
                30.0 * phase2 - 60.0 * phase3 + 30.0 * phase4;
            const double blend_ddot =
                60.0 * phase - 180.0 * phase2 + 120.0 * phase3;

            std::vector<double> position_rad(kNumJoints, 0.0);
            std::vector<double> velocity_rad_s(kNumJoints, 0.0);
            std::vector<double> acceleration_rad_s2(kNumJoints, 0.0);
            for (std::size_t joint_index = 0; joint_index < kNumJoints;
                 ++joint_index) {
                position_rad[joint_index] =
                    start_position_rad[joint_index] +
                    blend * delta_rad[joint_index];
                velocity_rad_s[joint_index] =
                    blend_dot / segment_duration_s * delta_rad[joint_index];
                acceleration_rad_s2[joint_index] =
                    blend_ddot / (segment_duration_s * segment_duration_s) *
                    delta_rad[joint_index];
            }

            trajectory.time_s.push_back(elapsed_time_s + local_time_s);
            trajectory.positions_rad.push_back(std::move(position_rad));
            trajectory.velocities_rad_s.push_back(std::move(velocity_rad_s));
            trajectory.accelerations_rad_s2.push_back(
                std::move(acceleration_rad_s2));
        }

        elapsed_time_s += segment_duration_s;
        has_previous_segment = true;
    }

    trajectory.motion_end_time_s = elapsed_time_s;
    const std::size_t dwell_sample_count = static_cast<std::size_t>(
        std::llround(kDwellDurationS / kSampleTimeS));
    if (dwell_sample_count > 0) {
        if (trajectory.positions_rad.empty()) {
            throw std::runtime_error("TBI trajectory has no moving samples.");
        }
        const std::vector<double> final_position_rad =
            trajectory.positions_rad.back();
        for (std::size_t sample_index = 1; sample_index <= dwell_sample_count;
             ++sample_index) {
            trajectory.time_s.push_back(
                trajectory.motion_end_time_s +
                static_cast<double>(sample_index) * kSampleTimeS);
            trajectory.positions_rad.push_back(final_position_rad);
            trajectory.velocities_rad_s.push_back(Zeros(kNumJoints));
            trajectory.accelerations_rad_s2.push_back(Zeros(kNumJoints));
        }
    }
    return trajectory;
}

[[nodiscard]] std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>>
EstimateDerivatives(const std::vector<std::vector<double>>& trajectory_rad) {
    const std::size_t sample_count = trajectory_rad.size();
    if (sample_count == 0) {
        return {{}, {}};
    }
    std::vector<std::vector<double>> velocity_rad_s(
        sample_count, Zeros(kNumJoints));
    std::vector<std::vector<double>> acceleration_rad_s2(
        sample_count, Zeros(kNumJoints));

    for (std::size_t joint_index = 0; joint_index < kNumJoints; ++joint_index) {
        for (std::size_t sample_index = 0; sample_index < sample_count;
             ++sample_index) {
            if (sample_count == 1) {
                velocity_rad_s[sample_index][joint_index] = 0.0;
            } else if (sample_index == 0) {
                velocity_rad_s[sample_index][joint_index] =
                    (trajectory_rad[1][joint_index] -
                     trajectory_rad[0][joint_index]) /
                    kSampleTimeS;
            } else if (sample_index + 1 == sample_count) {
                velocity_rad_s[sample_index][joint_index] =
                    (trajectory_rad[sample_index][joint_index] -
                     trajectory_rad[sample_index - 1][joint_index]) /
                    kSampleTimeS;
            } else {
                velocity_rad_s[sample_index][joint_index] =
                    (trajectory_rad[sample_index + 1][joint_index] -
                     trajectory_rad[sample_index - 1][joint_index]) /
                    (2.0 * kSampleTimeS);
            }
        }
        for (std::size_t sample_index = 0; sample_index < sample_count;
             ++sample_index) {
            if (sample_count == 1) {
                acceleration_rad_s2[sample_index][joint_index] = 0.0;
            } else if (sample_index == 0) {
                acceleration_rad_s2[sample_index][joint_index] =
                    (velocity_rad_s[1][joint_index] -
                     velocity_rad_s[0][joint_index]) /
                    kSampleTimeS;
            } else if (sample_index + 1 == sample_count) {
                acceleration_rad_s2[sample_index][joint_index] =
                    (velocity_rad_s[sample_index][joint_index] -
                     velocity_rad_s[sample_index - 1][joint_index]) /
                    kSampleTimeS;
            } else {
                acceleration_rad_s2[sample_index][joint_index] =
                    (velocity_rad_s[sample_index + 1][joint_index] -
                     velocity_rad_s[sample_index - 1][joint_index]) /
                    (2.0 * kSampleTimeS);
            }
        }
    }
    return {velocity_rad_s, acceleration_rad_s2};
}

[[nodiscard]] JointTrajectory ToJointTrajectory(
    const std::vector<double>& time_s,
    const std::vector<std::vector<double>>& positions_rad,
    const std::vector<std::vector<double>>* velocities_rad_s = nullptr,
    const std::vector<std::vector<double>>* accelerations_rad_s2 = nullptr) {
    if (time_s.size() != positions_rad.size()) {
        throw std::invalid_argument("Time and position sample counts must match.");
    }
    std::vector<JointSample> samples;
    samples.reserve(time_s.size());
    for (std::size_t sample_index = 0; sample_index < time_s.size();
         ++sample_index) {
        JointSample sample;
        sample.time_s = time_s[sample_index];
        sample.positions_rad = positions_rad[sample_index];
        if (velocities_rad_s != nullptr) {
            sample.velocities_rad_s = velocities_rad_s->at(sample_index);
        }
        if (accelerations_rad_s2 != nullptr) {
            sample.accelerations_rad_s2 = accelerations_rad_s2->at(sample_index);
        }
        samples.push_back(std::move(sample));
    }
    return JointTrajectory(std::move(samples));
}

[[nodiscard]] std::vector<double> UniformTimeVector(std::size_t sample_count) {
    std::vector<double> time_s;
    time_s.reserve(sample_count);
    for (std::size_t sample_index = 0; sample_index < sample_count;
         ++sample_index) {
        time_s.push_back(static_cast<double>(sample_index) * kSampleTimeS);
    }
    return time_s;
}

[[nodiscard]] std::vector<double> TimeFromTrajectory(
    const JointTrajectory& trajectory) {
    std::vector<double> time_s;
    time_s.reserve(trajectory.sample_count());
    for (const JointSample& sample : trajectory.samples()) {
        time_s.push_back(sample.time_s);
    }
    return time_s;
}

[[nodiscard]] std::vector<std::vector<double>> PositionsFromTrajectory(
    const JointTrajectory& trajectory) {
    std::vector<std::vector<double>> positions_rad;
    positions_rad.reserve(trajectory.sample_count());
    for (const JointSample& sample : trajectory.samples()) {
        positions_rad.push_back(sample.positions_rad);
    }
    return positions_rad;
}

[[nodiscard]] JointTracker MakeTracker(
    const JointTrackerOptimizerOptions& optimizer_options = {}) {
    // Build each example tracker from the same validated configuration while
    // allowing streaming examples to override window sizing.
    JointTrackerConfig config;
    config.num_joints = kNumJoints;
    config.modeled_axis_indices = {0, 1, 2, 3, 4, 5};
    config.sample_time_s = kSampleTimeS;
    config.optimizer_options = optimizer_options;
    JointTracker tracker(config);
    tracker.LoadModels(REFORGE_JOINT_TRACKER_EXAMPLE_MODELS_DIR);
    return tracker;
}

[[nodiscard]] WindowMetricsSummary SummarizeWindows(
    const std::vector<WindowStreamRecord>& window_records) {
    // Aggregate controller metadata emitted for one example stream.
    if (window_records.empty()) {
        throw std::invalid_argument("Cannot summarize an empty window stream.");
    }
    WindowMetricsSummary summary;
    summary.min_window_samples = std::numeric_limits<std::size_t>::max();
    summary.min_n_apply_samples = std::numeric_limits<std::size_t>::max();
    for (const WindowStreamRecord& record : window_records) {
        summary.min_window_samples =
            std::min(summary.min_window_samples, record.sample_count);
        summary.max_window_samples =
            std::max(summary.max_window_samples, record.sample_count);
        summary.min_n_apply_samples =
            std::min(summary.min_n_apply_samples, record.n_apply);
        summary.max_n_apply_samples =
            std::max(summary.max_n_apply_samples, record.n_apply);
    }
    return summary;
}

void AppendSamplesToStream(
    JointTrackerStream* stream,
    const std::vector<std::vector<double>>& source_positions_rad,
    std::size_t start_index,
    std::size_t stop_index,
    std::vector<std::vector<double>>* committed_positions_rad = nullptr) {
    if (stream == nullptr) {
        throw std::invalid_argument("stream must not be null.");
    }
    if (stop_index < start_index || stop_index > source_positions_rad.size()) {
        throw std::invalid_argument("Invalid append range.");
    }
    for (std::size_t sample_index = start_index; sample_index < stop_index;
         ++sample_index) {
        JointSample sample;
        sample.time_s = static_cast<double>(sample_index) * kSampleTimeS;
        sample.positions_rad = source_positions_rad[sample_index];
        stream->AppendReferenceSample(std::move(sample));
        if (committed_positions_rad != nullptr) {
            committed_positions_rad->push_back(source_positions_rad[sample_index]);
        }
    }
}

void AppendWindowPayload(
    const JointTrackerWindow& window,
    double pop_wait_time_s,
    std::vector<double>* streamed_times_s,
    std::vector<std::vector<double>>* streamed_positions_rad,
    std::vector<WindowStreamRecord>* window_records) {
    if (
        streamed_times_s == nullptr || streamed_positions_rad == nullptr ||
        window_records == nullptr) {
        throw std::invalid_argument("Output containers must not be null.");
    }
    for (const JointSample& sample : window.trajectory.samples()) {
        streamed_times_s->push_back(sample.time_s);
        streamed_positions_rad->push_back(sample.positions_rad);
    }
    WindowStreamRecord record;
    record.start_index = window.start_index;
    record.stop_index = window.stop_index;
    record.sample_count = window.trajectory.sample_count();
    record.pop_wait_time_s = pop_wait_time_s;
    record.n_apply = window.n_apply_samples;
    record.solve_index = window.solve_index;
    record.solve_start_index = window.solve_start_index;
    record.solve_stop_index = window.solve_stop_index;
    record.terminal_padding_samples = window.terminal_padding_samples;
    record.hold_samples_added = window.hold_samples_added;
    window_records->push_back(record);
}

[[nodiscard]] std::pair<PlannerSimulationState, std::pair<std::size_t, std::size_t>>
CreateAppendablePlannerSimulationState(
    const std::vector<std::vector<double>>& source_positions_rad) {
    PlannerSimulationState state;
    state.source_positions_rad = &source_positions_rad;
    state.append_stop_index =
        std::min(kInitialPlannerChunkSamples, source_positions_rad.size());
    state.append_chunk_samples = kAppendPlannerChunkSamples;
    state.low_watermark_samples = kLowWatermarkSamples;
    state.delay_thresholds = kDelayAfterSourceSamples;
    state.delay_windows = kDelayWindows;
    state.append_batch_sizes.push_back(state.append_stop_index);
    return {state, {0, state.append_stop_index}};
}

[[nodiscard]] PlannerDecision NextAppendablePlannerDecision(
    PlannerSimulationState* planner_state,
    std::size_t emitted_stop_index) {
    if (planner_state == nullptr || planner_state->source_positions_rad == nullptr) {
        throw std::invalid_argument("planner_state must contain source positions.");
    }
    if (planner_state->has_finished_appending) {
        return {};
    }

    std::optional<std::pair<std::size_t, std::size_t>> append_range;
    if (planner_state->remaining_delay_windows > 0) {
        --planner_state->remaining_delay_windows;
    } else if (
        planner_state->next_delay_index < planner_state->delay_thresholds.size() &&
        planner_state->append_stop_index >=
            planner_state->delay_thresholds[planner_state->next_delay_index]) {
        planner_state->remaining_delay_windows =
            planner_state->delay_windows[planner_state->next_delay_index];
        ++planner_state->next_delay_index;
    } else if (
        planner_state->append_stop_index <=
        emitted_stop_index + planner_state->low_watermark_samples) {
        std::size_t next_chunk_stop_index =
            planner_state->append_stop_index +
            planner_state->append_chunk_samples;
        if (
            planner_state->next_delay_index <
                planner_state->delay_thresholds.size() &&
            planner_state->append_stop_index <
                planner_state
                    ->delay_thresholds[planner_state->next_delay_index]) {
            next_chunk_stop_index = std::min(
                next_chunk_stop_index,
                planner_state
                    ->delay_thresholds[planner_state->next_delay_index]);
        }

        const std::size_t source_sample_count =
            planner_state->source_positions_rad->size();
        const std::size_t next_stop_index =
            std::min(next_chunk_stop_index, source_sample_count);
        if (next_stop_index > planner_state->append_stop_index) {
            append_range = {
                planner_state->append_stop_index,
                next_stop_index,
            };
            planner_state->append_stop_index = next_stop_index;
            planner_state->append_batch_sizes.push_back(
                next_stop_index - append_range->first);
        }
    }

    const std::size_t source_sample_count =
        planner_state->source_positions_rad->size();
    const bool should_finish =
        planner_state->append_stop_index >= source_sample_count &&
        planner_state->remaining_delay_windows == 0;
    if (should_finish) {
        planner_state->has_finished_appending = true;
    }
    return {append_range, should_finish};
}

void AppendHeldDesiredSamples(
    std::size_t hold_samples_added,
    std::vector<std::vector<double>>* committed_positions_rad) {
    if (hold_samples_added == 0) {
        return;
    }
    if (committed_positions_rad == nullptr || committed_positions_rad->empty()) {
        throw std::runtime_error("Cannot append held samples before a reference.");
    }
    const std::vector<double> held_position_rad = committed_positions_rad->back();
    for (std::size_t index = 0; index < hold_samples_added; ++index) {
        committed_positions_rad->push_back(held_position_rad);
    }
}

[[nodiscard]] double MaxCommandDifferenceRad(
    const std::vector<std::vector<double>>& first_positions_rad,
    const std::vector<std::vector<double>>& second_positions_rad) {
    const std::size_t common_samples =
        std::min(first_positions_rad.size(), second_positions_rad.size());
    if (common_samples == 0) {
        throw std::invalid_argument("Position arrays must share one sample.");
    }
    double max_difference_rad = 0.0;
    for (std::size_t sample_index = 0; sample_index < common_samples;
         ++sample_index) {
        for (std::size_t joint_index = 0; joint_index < kNumJoints;
             ++joint_index) {
            max_difference_rad = std::max(
                max_difference_rad,
                std::abs(
                    first_positions_rad[sample_index][joint_index] -
                    second_positions_rad[sample_index][joint_index]));
        }
    }
    return max_difference_rad;
}

[[nodiscard]] std::vector<std::vector<double>> InterpPositions(
    const std::vector<double>& source_time_s,
    const std::vector<std::vector<double>>& source_positions_rad,
    const std::vector<double>& target_time_s) {
    if (source_time_s.empty() || source_time_s.size() != source_positions_rad.size()) {
        throw std::invalid_argument("Source interpolation arrays are invalid.");
    }
    std::vector<std::vector<double>> interpolated(
        target_time_s.size(), Zeros(kNumJoints));
    for (std::size_t joint_index = 0; joint_index < kNumJoints; ++joint_index) {
        std::size_t source_index = 0;
        for (std::size_t target_index = 0; target_index < target_time_s.size();
             ++target_index) {
            const double target_time = target_time_s[target_index];
            while (
                source_index + 1 < source_time_s.size() &&
                source_time_s[source_index + 1] < target_time) {
                ++source_index;
            }
            if (target_time <= source_time_s.front()) {
                interpolated[target_index][joint_index] =
                    source_positions_rad.front()[joint_index];
            } else if (
                target_time >= source_time_s.back() ||
                source_index + 1 >= source_time_s.size()) {
                interpolated[target_index][joint_index] =
                    source_positions_rad.back()[joint_index];
            } else {
                const double t0 = source_time_s[source_index];
                const double t1 = source_time_s[source_index + 1];
                const double alpha = (target_time - t0) / (t1 - t0);
                interpolated[target_index][joint_index] =
                    (1.0 - alpha) *
                        source_positions_rad[source_index][joint_index] +
                    alpha *
                        source_positions_rad[source_index + 1][joint_index];
            }
        }
    }
    return interpolated;
}

[[nodiscard]] std::string ReadTextFile(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot read file: " + path.string());
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

[[nodiscard]] std::vector<double> ParseNumberArray(std::string_view array_text) {
    static const std::regex number_regex(
        R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
    const std::string text(array_text);
    std::vector<double> values;
    for (
        std::sregex_iterator iterator(text.begin(), text.end(), number_regex);
        iterator != std::sregex_iterator();
        ++iterator) {
        values.push_back(std::stod(iterator->str()));
    }
    return values;
}

[[nodiscard]] std::vector<TransferFunctionModel> LoadExampleDynamics() {
    const std::filesystem::path artifact_path =
        std::filesystem::path(REFORGE_JOINT_TRACKER_EXAMPLE_MODELS_DIR) /
        "joint_controller_models.json";
    const std::string text = ReadTextFile(artifact_path);
    static const std::regex axis_regex(
        R"("axis"\s*:\s*([0-9]+)[\s\S]*?"denominator"\s*:\s*\[([^\]]+)\][\s\S]*?"numerator"\s*:\s*\[([^\]]+)\])",
        std::regex::ECMAScript);

    std::vector<std::pair<std::size_t, TransferFunctionModel>> indexed_models;
    for (
        std::sregex_iterator iterator(text.begin(), text.end(), axis_regex);
        iterator != std::sregex_iterator();
        ++iterator) {
        TransferFunctionModel model;
        model.denominator = ParseNumberArray((*iterator)[2].str());
        model.numerator = ParseNumberArray((*iterator)[3].str());
        indexed_models.push_back(
            {static_cast<std::size_t>(std::stoul((*iterator)[1].str())), model});
    }
    std::sort(
        indexed_models.begin(),
        indexed_models.end(),
        [](const auto& first, const auto& second) {
            return first.first < second.first;
        });
    std::vector<TransferFunctionModel> models;
    models.reserve(indexed_models.size());
    for (const auto& indexed_model : indexed_models) {
        models.push_back(indexed_model.second);
    }
    if (models.size() < kModeledAxes.size()) {
        throw std::runtime_error(
            "Example dynamics file did not contain every modeled axis.");
    }
    return models;
}

[[nodiscard]] StateSpaceModel RealizeTransferFunction(
    const TransferFunctionModel& transfer_function) {
    if (transfer_function.denominator.size() != 4 ||
        transfer_function.numerator.size() != 3 ||
        transfer_function.denominator.front() == 0.0) {
        throw std::runtime_error(
            "Example response simulation expects third-order models.");
    }
    const double denominator_leading = transfer_function.denominator.front();
    std::array<double, 4> denominator{};
    std::array<double, 4> numerator{};
    const std::size_t numerator_offset =
        numerator.size() - transfer_function.numerator.size();
    for (std::size_t index = 0; index < denominator.size(); ++index) {
        denominator[index] = transfer_function.denominator[index] /
            denominator_leading;
    }
    for (std::size_t index = 0; index < transfer_function.numerator.size();
         ++index) {
        numerator[numerator_offset + index] =
            transfer_function.numerator[index] / denominator_leading;
    }

    StateSpaceModel model;
    for (std::size_t col = 0; col < 3; ++col) {
        model.a.value[0][col] = -denominator[col + 1];
    }
    model.a.value[1][0] = 1.0;
    model.a.value[2][1] = 1.0;
    model.b.value[0] = 1.0;
    model.d = numerator.front();
    for (std::size_t col = 0; col < 3; ++col) {
        model.c.value[col] = numerator[col + 1] -
            model.d * denominator[col + 1];
    }
    return model;
}

[[nodiscard]] Vector3 StateDerivative(
    const StateSpaceModel& model,
    const Vector3& state,
    double input) {
    Vector3 derivative;
    for (std::size_t row = 0; row < 3; ++row) {
        derivative.value[row] = model.b.value[row] * input;
        for (std::size_t col = 0; col < 3; ++col) {
            derivative.value[row] += model.a.value[row][col] * state.value[col];
        }
    }
    return derivative;
}

[[nodiscard]] Vector3 AddScaled(
    const Vector3& state,
    const Vector3& derivative,
    double scale) {
    Vector3 result;
    for (std::size_t index = 0; index < 3; ++index) {
        result.value[index] = state.value[index] + scale * derivative.value[index];
    }
    return result;
}

[[nodiscard]] double OutputValue(
    const StateSpaceModel& model,
    const Vector3& state,
    double input) {
    double output = model.d * input;
    for (std::size_t index = 0; index < 3; ++index) {
        output += model.c.value[index] * state.value[index];
    }
    return output;
}

[[nodiscard]] std::vector<double> SimulateAxisResponse(
    const TransferFunctionModel& transfer_function,
    const std::vector<double>& time_s,
    const std::vector<double>& command_rad) {
    if (time_s.size() != command_rad.size() || time_s.empty()) {
        throw std::invalid_argument("Response simulation inputs are invalid.");
    }
    const StateSpaceModel model = RealizeTransferFunction(transfer_function);
    const double initial_command_rad = command_rad.front();
    Vector3 state;
    std::vector<double> response_rad(command_rad.size(), initial_command_rad);

    for (std::size_t sample_index = 0; sample_index + 1 < command_rad.size();
         ++sample_index) {
        const double u0 = command_rad[sample_index] - initial_command_rad;
        const double u1 = command_rad[sample_index + 1] - initial_command_rad;
        response_rad[sample_index] =
            OutputValue(model, state, u0) + initial_command_rad;
        const double dt = time_s[sample_index + 1] - time_s[sample_index];
        const double sub_dt =
            dt / static_cast<double>(kResponseIntegrationSubsteps);
        for (int substep = 0; substep < kResponseIntegrationSubsteps;
             ++substep) {
            const double alpha0 =
                static_cast<double>(substep) /
                static_cast<double>(kResponseIntegrationSubsteps);
            const double alpha_mid =
                (static_cast<double>(substep) + 0.5) /
                static_cast<double>(kResponseIntegrationSubsteps);
            const double alpha1 =
                (static_cast<double>(substep) + 1.0) /
                static_cast<double>(kResponseIntegrationSubsteps);
            const double input0 = (1.0 - alpha0) * u0 + alpha0 * u1;
            const double input_mid =
                (1.0 - alpha_mid) * u0 + alpha_mid * u1;
            const double input1 = (1.0 - alpha1) * u0 + alpha1 * u1;

            const Vector3 k1 = StateDerivative(model, state, input0);
            const Vector3 k2 = StateDerivative(
                model,
                AddScaled(state, k1, 0.5 * sub_dt),
                input_mid);
            const Vector3 k3 = StateDerivative(
                model,
                AddScaled(state, k2, 0.5 * sub_dt),
                input_mid);
            const Vector3 k4 = StateDerivative(
                model,
                AddScaled(state, k3, sub_dt),
                input1);
            for (std::size_t index = 0; index < 3; ++index) {
                state.value[index] +=
                    sub_dt / 6.0 *
                    (k1.value[index] + 2.0 * k2.value[index] +
                     2.0 * k3.value[index] + k4.value[index]);
            }
        }
    }
    response_rad.back() =
        OutputValue(
            model,
            state,
            command_rad.back() - initial_command_rad) +
        initial_command_rad;
    return response_rad;
}

[[nodiscard]] std::vector<std::vector<double>> SimulateJointResponse(
    const std::vector<double>& time_s,
    const std::vector<std::vector<double>>& command_positions_rad) {
    if (time_s.size() != command_positions_rad.size() || time_s.empty()) {
        throw std::invalid_argument("Response simulation inputs are invalid.");
    }
    std::vector<std::vector<double>> response_rad = command_positions_rad;
    const std::vector<TransferFunctionModel> dynamics = LoadExampleDynamics();
    for (std::size_t model_index = 0; model_index < kModeledAxes.size();
         ++model_index) {
        const int axis_index = kModeledAxes[model_index];
        std::vector<double> command_axis_rad;
        command_axis_rad.reserve(command_positions_rad.size());
        for (const std::vector<double>& sample_rad : command_positions_rad) {
            command_axis_rad.push_back(sample_rad.at(axis_index));
        }
        const std::vector<double> response_axis_rad =
            SimulateAxisResponse(dynamics[model_index], time_s, command_axis_rad);
        for (std::size_t sample_index = 0; sample_index < response_rad.size();
             ++sample_index) {
            response_rad[sample_index][axis_index] =
                response_axis_rad[sample_index];
        }
    }
    return response_rad;
}

[[nodiscard]] double TrackingRmsError(
    const std::vector<std::vector<double>>& desired_positions_rad,
    const std::vector<std::vector<double>>& response_rad,
    std::size_t axis_index) {
    const std::size_t sample_count =
        std::min(desired_positions_rad.size(), response_rad.size());
    if (sample_count == 0) {
        throw std::invalid_argument("Tracking RMS requires samples.");
    }
    double squared_error_sum = 0.0;
    for (std::size_t sample_index = 0; sample_index < sample_count;
         ++sample_index) {
        const double error_rad =
            response_rad[sample_index][axis_index] -
            desired_positions_rad[sample_index][axis_index];
        squared_error_sum += error_rad * error_rad;
    }
    return std::sqrt(squared_error_sum / static_cast<double>(sample_count));
}

[[nodiscard]] TrackingSummary BuildTrackingSummary(
    const TrajectoryArrays& desired,
    const std::vector<double>& offline_time_s,
    const std::vector<std::vector<double>>& offline_positions_rad,
    const StreamResult& streamed_result) {
    const std::vector<std::vector<double>> desired_response_rad =
        SimulateJointResponse(desired.time_s, desired.positions_rad);
    const std::vector<std::vector<double>> offline_response_rad =
        SimulateJointResponse(offline_time_s, offline_positions_rad);
    const std::vector<std::vector<double>> streamed_response_rad =
        SimulateJointResponse(
            streamed_result.optimized_time_s,
            streamed_result.optimized_positions_rad);
    const std::vector<std::vector<double>> offline_desired_rad =
        InterpPositions(
            desired.time_s,
            desired.positions_rad,
            offline_time_s);
    const std::vector<std::vector<double>> streamed_desired_rad =
        InterpPositions(
            desired.time_s,
            desired.positions_rad,
            streamed_result.optimized_time_s);

    TrackingSummary summary;
    for (const int axis_index : kModeledAxes) {
        summary.unoptimized_rms_rad.push_back(
            TrackingRmsError(
                desired.positions_rad,
                desired_response_rad,
                static_cast<std::size_t>(axis_index)));
        summary.example_1_rms_rad.push_back(
            TrackingRmsError(
                offline_desired_rad,
                offline_response_rad,
                static_cast<std::size_t>(axis_index)));
        summary.streamed_rms_rad.push_back(
            TrackingRmsError(
                streamed_desired_rad,
                streamed_response_rad,
                static_cast<std::size_t>(axis_index)));
    }
    return summary;
}

[[nodiscard]] ExampleResults RunJointTrackerExamples() {
    // SIMULATION ONLY: this helper creates a feasible desired trajectory so the
    // example can run without a robot SDK. In your application, replace this
    // block with the trajectory from your planner, teach pendant, or robot SDK.
    TrajectoryArrays desired = GenerateTbiTrajectory();
    const JointTrajectory desired_trajectory = ToJointTrajectory(
        desired.time_s,
        desired.positions_rad,
        &desired.velocities_rad_s,
        &desired.accelerations_rad_s2);

    // =========================================================================
    // Start Example 1: Optimize a complete trajectory with Joint Tracker.
    //
    // CONTROLLER CODE TO COPY: use this pattern when the complete desired
    // trajectory is available before execution starts.
    // =========================================================================
    JointTrajectory JTC_offline_mode;
    {
        JointTracker initializing_JTC = MakeTracker();
        JTC_offline_mode = initializing_JTC.OptimizeTrajectory(desired_trajectory);
        // REPLACE WITH YOUR ROBOT SDK: send `JTC_offline_mode.samples()` to the
        // robot if your SDK accepts full trajectory uploads.
    }
    // =========================================================================
    // End Example 1.
    // =========================================================================

    // =========================================================================
    // Start Example 2A: Optimize a known trajectory while the robot is moving.
    //
    // CONTROLLER CODE TO COPY: use this pattern when the complete desired
    // trajectory is known before execution, but you want JTC to optimize and emit
    // windows while your robot loop consumes them.
    // =========================================================================
    StreamResult lookahead_stream_result;
    {
        JointTracker initializing_JTC = MakeTracker();
        JointTrackerStream JTC_stream_mode = initializing_JTC.CreateStream();

        // Append the complete known trajectory before execution starts.
        JTC_stream_mode.AppendReference(desired_trajectory);

        // Tell Joint Tracker that no more desired samples will be appended.
        JTC_stream_mode.Finish();

        // SIMULATION/REPORTING ONLY: these lists store emitted windows for
        // plots and metrics. Your robot loop can send each command window
        // directly instead.
        lookahead_stream_result.desired_time_s = desired.time_s;
        lookahead_stream_result.desired_positions_rad = desired.positions_rad;
        lookahead_stream_result.appended_source_samples =
            desired.positions_rad.size();

        // Consume optimized command windows and send each window to the robot
        // SDK.
        while (true) {
            const auto pop_start_wall_s = std::chrono::steady_clock::now();
            const std::optional<JointTrackerWindow> command_window =
                JTC_stream_mode.PopWindow();
            const double pop_wait_time_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - pop_start_wall_s)
                                               .count();
            if (!command_window.has_value()) {
                if (JTC_stream_mode.finished()) {
                    break;
                }
                throw std::runtime_error(
                    "Appendable Joint Tracker stream did not produce a window.");
            }

            // REPLACE WITH YOUR ROBOT SDK: send
            // `command_window->trajectory.samples()` to the robot here. Keep
            // the helper below only when running this demo.
            AppendWindowPayload(
                *command_window,
                pop_wait_time_s,
                &lookahead_stream_result.optimized_time_s,
                &lookahead_stream_result.optimized_positions_rad,
                &lookahead_stream_result.window_records);
        }

        for (const WindowStreamRecord& record :
             lookahead_stream_result.window_records) {
            lookahead_stream_result.terminal_padding_samples +=
                record.terminal_padding_samples;
            lookahead_stream_result.hold_samples_added += record.hold_samples_added;
        }
    }
    // =========================================================================
    // End Example 2A.
    // =========================================================================
    const std::vector<std::vector<double>> example_1_positions_rad =
        PositionsFromTrajectory(JTC_offline_mode);
    const double max_example_2a_difference_rad = MaxCommandDifferenceRad(
        example_1_positions_rad,
        lookahead_stream_result.optimized_positions_rad);

    // =========================================================================
    // Start Example 2B: Optimize a trajectory generated by an online planner.
    //
    // CONTROLLER CODE TO COPY: use this pattern when your online planner generates
    // trajectory samples while the robot is moving, including pauses between
    // point-to-point moves. JTC keeps holding the last desired sample until more
    // samples arrive.
    // =========================================================================
    StreamResult online_stream_result;
    {
        // Single-sample online streaming setting: append one desired sample at
        // a time and make one optimized sample available to send to the robot.
        JointTrackerOptimizerOptions single_sample_options;
        single_sample_options.n_apply_scale = kSingleSampleNApplyScale;
        JointTracker initializing_JTC = MakeTracker(single_sample_options);
        JointTrackerStream JTC_online_mode = initializing_JTC.CreateStream();

        // SIMULATION ONLY: this helper mimics an online planner that sometimes
        // delays new trajectory samples. Replace it with your actual planner
        // state.
        auto [simulated_planner_state, desired_trajectory_beginning_range] =
            CreateAppendablePlannerSimulationState(desired.positions_rad);

        online_stream_result.appended_source_samples = desired.positions_rad.size();

        // Append exactly one first desired sample before running the stream.
        // Each planner update below also appends one sample.
        AppendSamplesToStream(
            &JTC_online_mode,
            desired.positions_rad,
            desired_trajectory_beginning_range.first,
            desired_trajectory_beginning_range.second,
            &online_stream_result.desired_positions_rad);

        // Real-time loop: each iteration sends one optimized command window to
        // the robot, then appends the planner's latest samples when available.
        while (true) {
            const auto pop_start_wall_s = std::chrono::steady_clock::now();
            const std::optional<JointTrackerWindow> command_window =
                JTC_online_mode.PopWindow();
            const double pop_wait_time_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - pop_start_wall_s)
                                               .count();
            if (!command_window.has_value()) {
                if (simulated_planner_state.has_finished_appending) {
                    break;
                }
                throw std::runtime_error(
                    "Appendable Joint Tracker stream did not produce a window.");
            }

            // REPLACE WITH YOUR ROBOT SDK: send
            // `command_window->trajectory.samples()` to the robot here. Keep
            // the helper below only when running this demo.
            AppendWindowPayload(
                *command_window,
                pop_wait_time_s,
                &online_stream_result.optimized_time_s,
                &online_stream_result.optimized_positions_rad,
                &online_stream_result.window_records);
            AppendHeldDesiredSamples(
                command_window->hold_samples_added,
                &online_stream_result.desired_positions_rad);

            // SIMULATION ONLY: this computes whether the fake planner has
            // produced more desired samples. Replace it with your own planner
            // output check.
            const PlannerDecision planner_decision =
                NextAppendablePlannerDecision(
                    &simulated_planner_state,
                    command_window->stop_index);
            if (planner_decision.append_range.has_value()) {
                // Append newly planned desired samples.
                AppendSamplesToStream(
                    &JTC_online_mode,
                    desired.positions_rad,
                    planner_decision.append_range->first,
                    planner_decision.append_range->second,
                    &online_stream_result.desired_positions_rad);
            }
            if (planner_decision.should_finish) {
                // Tell JTC that no more desired samples will be appended.
                JTC_online_mode.Finish();
            }
        }

        online_stream_result.desired_time_s =
            UniformTimeVector(online_stream_result.desired_positions_rad.size());
        online_stream_result.planner_update_sample_counts =
            simulated_planner_state.append_batch_sizes;
        for (const WindowStreamRecord& record :
             online_stream_result.window_records) {
            online_stream_result.terminal_padding_samples +=
                record.terminal_padding_samples;
            online_stream_result.hold_samples_added += record.hold_samples_added;
        }
    }
    // =========================================================================
    // End Example 2B.
    // =========================================================================
    auto [delayed_velocity_rad_s, delayed_acceleration_rad_s2] =
        EstimateDerivatives(online_stream_result.desired_positions_rad);
    JointTracker delayed_reference_tracker = MakeTracker();
    const JointTrajectory delayed_reference = ToJointTrajectory(
        online_stream_result.desired_time_s,
        online_stream_result.desired_positions_rad,
        &delayed_velocity_rad_s,
        &delayed_acceleration_rad_s2);
    const JointTrajectory JTC_delayed_reference_offline_mode =
        delayed_reference_tracker.OptimizeTrajectory(delayed_reference);
    const std::vector<std::vector<double>> delayed_reference_positions_rad =
        PositionsFromTrajectory(JTC_delayed_reference_offline_mode);
    const double max_example_2b_difference_rad = MaxCommandDifferenceRad(
        delayed_reference_positions_rad,
        online_stream_result.optimized_positions_rad);

    TrackingSummary tracking_summary = BuildTrackingSummary(
        desired,
        TimeFromTrajectory(JTC_offline_mode),
        example_1_positions_rad,
        lookahead_stream_result);

    return {
        std::move(desired),
        TimeFromTrajectory(JTC_offline_mode),
        example_1_positions_rad,
        std::move(lookahead_stream_result),
        std::move(online_stream_result),
        TimeFromTrajectory(JTC_delayed_reference_offline_mode),
        delayed_reference_positions_rad,
        std::move(tracking_summary),
        max_example_2a_difference_rad,
        max_example_2b_difference_rad,
    };
}

void PrintTrackingSummary(
    const std::string& label,
    const std::vector<double>& rms_by_axis_rad) {
    std::cout << label << " full-trajectory RMS tracking error: ";
    for (std::size_t axis_index = 0; axis_index < rms_by_axis_rad.size();
         ++axis_index) {
        if (axis_index > 0) {
            std::cout << ", ";
        }
        std::cout << "J" << (axis_index + 1) << "=" << std::fixed
                  << std::setprecision(6) << rms_by_axis_rad[axis_index]
                  << " rad";
    }
    std::cout << "\n";
}

void PrintExampleSummary(const ExampleResults& results) {
    std::cout << "\nAppendable-stream JTC comparison\n";
    std::cout << "Max command difference vs Example 1 offline: "
              << std::fixed << std::setprecision(6)
              << results.example_2a_max_difference_rad << " rad\n";
    std::cout << "Held-reference samples: "
              << results.example_2a_stream.hold_samples_added << "\n";
    std::cout << "Terminal padding samples: "
              << results.example_2a_stream.terminal_padding_samples << "\n";
    std::cout << "\nDelayed appendable-stream JTC comparison\n";
    std::cout << "Max command difference vs delayed-reference offline benchmark: "
              << std::fixed << std::setprecision(6)
              << results.example_2b_max_difference_rad << " rad\n";
    std::cout << "Held-reference samples: "
              << results.example_2b_stream.hold_samples_added << "\n";
    std::cout << "Terminal padding samples: "
              << results.example_2b_stream.terminal_padding_samples << "\n";
    std::cout << "Planner updates: "
              << results.example_2b_stream.planner_update_sample_counts.size()
              << " (max samples per update: "
              << *std::max_element(
                     results.example_2b_stream.planner_update_sample_counts.begin(),
                     results.example_2b_stream.planner_update_sample_counts.end())
              << ")\n";
    std::cout << "\nCovalent Joint Tracker example complete.\n";
    PrintTrackingSummary(
        "Unoptimized desired command",
        results.tracking_summary.unoptimized_rms_rad);
    PrintTrackingSummary(
        "Offline Joint Tracker command",
        results.tracking_summary.example_1_rms_rad);
    PrintTrackingSummary(
        "Appendable-stream Joint Tracker command",
        results.tracking_summary.streamed_rms_rad);
}

void WriteJsonNumberArray(
    std::ostream& output,
    const std::vector<double>& values,
    int indent_spaces) {
    const std::string indent(static_cast<std::size_t>(indent_spaces), ' ');
    output << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            output << ", ";
        }
        output << std::setprecision(17) << values[index];
    }
    output << "]";
    if (indent_spaces < 0) {
        output << indent;
    }
}

void WriteJsonIntegerArray(
    std::ostream& output,
    const std::vector<std::size_t>& values) {
    output << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            output << ", ";
        }
        output << values[index];
    }
    output << "]";
}

void WriteMetricsJson(
    const std::filesystem::path& path,
    const ExampleResults& results) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Cannot write metrics JSON: " + path.string());
    }
    output << std::setprecision(17);
    const WindowMetricsSummary example_2a_window_metrics =
        SummarizeWindows(results.example_2a_stream.window_records);
    const WindowMetricsSummary example_2b_window_metrics =
        SummarizeWindows(results.example_2b_stream.window_records);
    output << "{\n";
    output << "  \"sample_time_s\": " << kSampleTimeS << ",\n";
    output << "  \"sample_counts\": {\n";
    output << "    \"desired\": " << results.desired.time_s.size() << ",\n";
    output << "    \"example_1\": " << results.example_1_time_s.size()
           << ",\n";
    output << "    \"example_2a\": "
           << results.example_2a_stream.optimized_time_s.size() << ",\n";
    output << "    \"example_2b\": "
           << results.example_2b_stream.optimized_time_s.size() << ",\n";
    output << "    \"example_2b_committed_desired\": "
           << results.example_2b_stream.desired_time_s.size() << "\n";
    output << "  },\n";
    output << "  \"example_1\": {\n";
    output << "    \"sample_count\": " << results.example_1_time_s.size()
           << "\n";
    output << "  },\n";
    output << "  \"example_2a\": {\n";
    output << "    \"sample_count\": "
           << results.example_2a_stream.optimized_time_s.size() << ",\n";
    output << "    \"max_command_difference_vs_example_1_rad\": "
           << results.example_2a_max_difference_rad << ",\n";
    output << "    \"held_reference_samples\": "
           << results.example_2a_stream.hold_samples_added << ",\n";
    output << "    \"terminal_padding_samples\": "
           << results.example_2a_stream.terminal_padding_samples << ",\n";
    output << "    \"min_window_samples\": "
           << example_2a_window_metrics.min_window_samples << ",\n";
    output << "    \"max_window_samples\": "
           << example_2a_window_metrics.max_window_samples << ",\n";
    output << "    \"min_n_apply_samples\": "
           << example_2a_window_metrics.min_n_apply_samples << ",\n";
    output << "    \"max_n_apply_samples\": "
           << example_2a_window_metrics.max_n_apply_samples << "\n";
    output << "  },\n";
    output << "  \"example_2b\": {\n";
    output << "    \"sample_count\": "
           << results.example_2b_stream.optimized_time_s.size() << ",\n";
    output << "    \"committed_desired_sample_count\": "
           << results.example_2b_stream.desired_time_s.size() << ",\n";
    output << "    \"max_command_difference_vs_delayed_reference_rad\": "
           << results.example_2b_max_difference_rad << ",\n";
    output << "    \"held_reference_samples\": "
           << results.example_2b_stream.hold_samples_added << ",\n";
    output << "    \"terminal_padding_samples\": "
           << results.example_2b_stream.terminal_padding_samples << ",\n";
    output << "    \"min_window_samples\": "
           << example_2b_window_metrics.min_window_samples << ",\n";
    output << "    \"max_window_samples\": "
           << example_2b_window_metrics.max_window_samples << ",\n";
    output << "    \"min_n_apply_samples\": "
           << example_2b_window_metrics.min_n_apply_samples << ",\n";
    output << "    \"max_n_apply_samples\": "
           << example_2b_window_metrics.max_n_apply_samples << ",\n";
    output << "    \"planner_update_count\": "
           << results.example_2b_stream.planner_update_sample_counts.size()
           << ",\n";
    output << "    \"planner_input_sample_count\": "
           << results.example_2b_stream.appended_source_samples << ",\n";
    output << "    \"planner_update_sample_counts\": ";
    WriteJsonIntegerArray(
        output,
        results.example_2b_stream.planner_update_sample_counts);
    output << ",\n";
    output << "    \"planner_constants\": {\n";
    output << "      \"initial_chunk_samples\": "
           << kInitialPlannerChunkSamples << ",\n";
    output << "      \"append_chunk_samples\": "
           << kAppendPlannerChunkSamples << ",\n";
    output << "      \"low_watermark_samples\": " << kLowWatermarkSamples
           << ",\n";
    output << "      \"delay_after_source_samples\": [56, 111, 170],\n";
    output << "      \"delay_windows\": [3, 3, 4]\n";
    output << "    }\n";
    output << "  },\n";
    output << "  \"tracking_summary\": {\n";
    output << "    \"unoptimized_desired_command_rms_rad\": ";
    WriteJsonNumberArray(
        output,
        results.tracking_summary.unoptimized_rms_rad,
        4);
    output << ",\n";
    output << "    \"example_1_joint_tracker_command_rms_rad\": ";
    WriteJsonNumberArray(
        output,
        results.tracking_summary.example_1_rms_rad,
        4);
    output << ",\n";
    output << "    \"streamed_joint_tracker_command_rms_rad\": ";
    WriteJsonNumberArray(
        output,
        results.tracking_summary.streamed_rms_rad,
        4);
    output << "\n";
    output << "  }\n";
    output << "}\n";
}

void WritePlotCsv(const std::filesystem::path& path, const ExampleResults& results) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    const std::vector<std::vector<double>> desired_response_rad =
        SimulateJointResponse(results.desired.time_s, results.desired.positions_rad);
    const std::vector<std::vector<double>> example_1_response_rad =
        SimulateJointResponse(
            results.example_1_time_s,
            results.example_1_positions_rad);
    const std::vector<std::vector<double>> streamed_response_rad =
        SimulateJointResponse(
            results.example_2a_stream.optimized_time_s,
            results.example_2a_stream.optimized_positions_rad);
    const auto [desired_velocity_rad_s, desired_acceleration_rad_s2] =
        EstimateDerivatives(results.desired.positions_rad);
    const auto [example_1_velocity_rad_s, example_1_acceleration_rad_s2] =
        EstimateDerivatives(results.example_1_positions_rad);
    const auto [streamed_velocity_rad_s, streamed_acceleration_rad_s2] =
        EstimateDerivatives(results.example_2a_stream.optimized_positions_rad);

    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("Cannot write plot CSV: " + path.string());
    }
    output << "series,profile,joint,time_s,value\n";
    const auto write_series = [&output](
                                  const std::string& series,
                                  const std::string& profile,
                                  const std::vector<double>& time_s,
                                  const std::vector<std::vector<double>>& values) {
        for (std::size_t sample_index = 0; sample_index < time_s.size();
             ++sample_index) {
            for (std::size_t joint_index = 0; joint_index < kNumJoints;
                 ++joint_index) {
                output << series << ',' << profile << ',' << (joint_index + 1)
                       << ',' << std::setprecision(17) << time_s[sample_index]
                       << ',' << values[sample_index][joint_index] << '\n';
            }
        }
    };
    write_series(
        "Desired command",
        "position",
        results.desired.time_s,
        results.desired.positions_rad);
    write_series(
        "Desired command",
        "velocity",
        results.desired.time_s,
        desired_velocity_rad_s);
    write_series(
        "Desired command",
        "acceleration",
        results.desired.time_s,
        desired_acceleration_rad_s2);
    write_series(
        "Example 1 command",
        "position",
        results.example_1_time_s,
        results.example_1_positions_rad);
    write_series(
        "Example 1 command",
        "velocity",
        results.example_1_time_s,
        example_1_velocity_rad_s);
    write_series(
        "Example 1 command",
        "acceleration",
        results.example_1_time_s,
        example_1_acceleration_rad_s2);
    write_series(
        "Example 2A command",
        "position",
        results.example_2a_stream.optimized_time_s,
        results.example_2a_stream.optimized_positions_rad);
    write_series(
        "Example 2A command",
        "velocity",
        results.example_2a_stream.optimized_time_s,
        streamed_velocity_rad_s);
    write_series(
        "Example 2A command",
        "acceleration",
        results.example_2a_stream.optimized_time_s,
        streamed_acceleration_rad_s2);
    write_series(
        "Desired trajectory",
        "response",
        results.desired.time_s,
        results.desired.positions_rad);
    write_series(
        "Response to desired command",
        "response",
        results.desired.time_s,
        desired_response_rad);
    write_series(
        "Response to JTC Example 1 command",
        "response",
        results.example_1_time_s,
        example_1_response_rad);
    write_series(
        "Response to JTC Example 2A command",
        "response",
        results.example_2a_stream.optimized_time_s,
        streamed_response_rad);
}

void RenderPlots(
    const std::filesystem::path& output_directory,
    const ExampleResults& results,
    bool show_plots) {
    std::filesystem::create_directories(output_directory);
    const std::filesystem::path csv_path =
        output_directory / "joint_tracker_plot_data.csv";
    WritePlotCsv(csv_path, results);
    reforge_example_plotting::RenderJointTrackerPlots(
        csv_path,
        output_directory,
        results.desired.motion_end_time_s,
        show_plots);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = ParseOptions(argc, argv);
        const ExampleResults results = RunJointTrackerExamples();
        PrintExampleSummary(results);

        if (!options.metrics_json.empty()) {
            WriteMetricsJson(options.metrics_json, results);
        }
        if (!options.save_plots.empty()) {
            RenderPlots(options.save_plots, results, !options.no_plots);
        } else if (!options.no_plots) {
            RenderPlots(
                std::filesystem::temp_directory_path() /
                    "reforge_joint_tracker_plots",
                results,
                true);
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Joint Tracker example failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
