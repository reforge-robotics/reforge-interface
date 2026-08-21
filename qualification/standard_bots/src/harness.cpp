#include "reforge_qualification/harness.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <builtin_interfaces/msg/duration.hpp>

#include "reforge_core/control/runtime/modal_inference.hpp"
#include "reforge_core/control/shaper/backend/backend_configuration.hpp"
#include "reforge_core/control/shaper/backend/backend_requests.hpp"
#include "reforge_core/control/shaper/backend/native_shaper.hpp"
#include "reforge_core/control/shaper/ros2/joint_trajectory_adapter.hpp"

namespace reforge::qualification {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
constexpr double kNumericTolerance = 1.0e-9;

[[nodiscard]] double DurationSeconds(
    const builtin_interfaces::msg::Duration& duration) {
    return static_cast<double>(duration.sec) +
           static_cast<double>(duration.nanosec) * 1.0e-9;
}

[[nodiscard]] builtin_interfaces::msg::Duration RosDuration(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0) {
        throw std::invalid_argument("trajectory time must be finite and nonnegative");
    }
    const auto nanoseconds = static_cast<std::int64_t>(
        std::llround(seconds * static_cast<double>(kNanosecondsPerSecond)));
    builtin_interfaces::msg::Duration result;
    result.sec = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
    result.nanosec = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
    return result;
}

[[nodiscard]] std::vector<double> Gradient(
    const std::vector<double>& values,
    double sample_period_s) {
    if (values.size() < 2) {
        return std::vector<double>(values.size(), 0.0);
    }
    std::vector<double> result(values.size(), 0.0);
    result.front() = (values[1] - values[0]) / sample_period_s;
    for (std::size_t index = 1; index + 1 < values.size(); ++index) {
        result[index] =
            (values[index + 1] - values[index - 1]) / (2.0 * sample_period_s);
    }
    result.back() =
        (values.back() - values[values.size() - 2]) / sample_period_s;
    return result;
}

[[nodiscard]] reforge::control::shaper::backend::BackendConfiguration
MakeBackendConfiguration(const std::filesystem::path& assets_directory) {
    using reforge::control::runtime::SharedImpulsePolicy;
    using reforge::control::shaper::ResidualPathScope;
    using reforge::control::shaper::backend::BackendConfiguration;
    using reforge::native::Vector;

    const auto model_directory = assets_directory / "models/current/shaper";
    const auto urdf_path = assets_directory / "urdf/modelone.urdf";
    if (!std::filesystem::is_regular_file(model_directory / "model_bundle.json") ||
        !std::filesystem::is_regular_file(
            model_directory / "shaper_models.native.json") ||
        !std::filesystem::is_regular_file(urdf_path)) {
        throw std::invalid_argument(
            "assets directory must contain models/current/shaper and urdf/modelone.urdf: " +
            assets_directory.string());
    }
    BackendConfiguration configuration;
    configuration.sample_time_s = kSamplePeriodS;
    configuration.model_directory = model_directory;
    configuration.urdf_filepath = urdf_path;
    configuration.num_axes = 3;
    configuration.side_length_m = 0.0007;
    configuration.base_height_m = 0.364;
    configuration.num_joints = kJointCount;
    configuration.probability_threshold = 0.5;
    configuration.shared_impulse_policy = SharedImpulsePolicy::kCombineAllModes;
    configuration.shared_impulse_shapes_all_joints = true;
    configuration.residual_path_scope = ResidualPathScope::kShapedAxes;
    configuration.aligned_tail_require_zero_acceleration_region = false;
    configuration.train_base_angles_rad = Vector::Zero(1);
    configuration.feature_frame_translation_m = std::nullopt;
    return configuration;
}

void RequireFinite(const std::vector<double>& values, const std::string& field) {
    if (!std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value);
        })) {
        throw std::invalid_argument(field + " contains a non-finite value");
    }
}

[[nodiscard]] std::size_t NearestCommandIndex(
    const trajectory_msgs::msg::JointTrajectory& command,
    double elapsed_s) {
    const auto iterator = std::lower_bound(
        command.points.begin(), command.points.end(), elapsed_s,
        [](const auto& point, double value) {
            return DurationSeconds(point.time_from_start) < value;
        });
    if (iterator == command.points.begin()) {
        return 0;
    }
    if (iterator == command.points.end()) {
        return command.points.size() - 1;
    }
    const auto upper = static_cast<std::size_t>(iterator - command.points.begin());
    const auto lower = upper - 1;
    return elapsed_s - DurationSeconds(command.points[lower].time_from_start) <=
                   DurationSeconds(command.points[upper].time_from_start) - elapsed_s
               ? lower
               : upper;
}

}  // namespace

TrialDefinition FrozenPhaseATrial() {
    return TrialDefinition{
        {0.8031466159123666, 0.4929384598766246, 1.2931427771424118,
         1.29476267022952, 0.01714594449146777, -1.5119635056830696},
        {0.017748452514918255, 0.4929384598766246, 1.2931427771424118,
         1.29476267022952, 0.01714594449146777, -1.5119635056830696},
        kSamplePeriodS,
        1.25,
        2.0,
        4.0};
}

SafetyLimits ConservativeSafetyLimits() {
    SafetyLimits limits;
    limits.minimum_position_rad = {
        -6.283185307179586 + 0.05, -2.356194490192345 + 0.05,
        -3.141592653589793 + 0.05, -6.283185307179586 + 0.05,
        -6.283185307179586 + 0.05, -6.283185307179586 + 0.05};
    limits.maximum_position_rad = {
        6.283185307179586 - 0.05, 2.356194490192345 - 0.05,
        3.141592653589793 - 0.05, 6.283185307179586 - 0.05,
        6.283185307179586 - 0.05, 6.283185307179586 - 0.05};
    return limits;
}

trajectory_msgs::msg::JointTrajectory GenerateUnshapedTrajectory(
    const TrialDefinition& definition) {
    if (!(definition.sample_period_s > 0.0) ||
        !(definition.max_velocity_rad_s > 0.0) ||
        !(definition.max_acceleration_rad_s2 > 0.0) ||
        definition.settling_window_s < 0.0) {
        throw std::invalid_argument("trial timing and limits must be positive");
    }
    const double signed_displacement =
        definition.goal_rad[0] - definition.start_rad[0];
    const double displacement = std::abs(signed_displacement);
    const double direction = signed_displacement < 0.0 ? -1.0 : 1.0;
    const auto acceleration_samples = static_cast<std::size_t>(std::ceil(
        std::sqrt(displacement / definition.max_acceleration_rad_s2) /
        definition.sample_period_s));
    const double acceleration_time_s =
        static_cast<double>(acceleration_samples) * definition.sample_period_s;
    const double adjusted_acceleration =
        displacement / (acceleration_time_s * acceleration_time_s);
    const double peak_velocity = adjusted_acceleration * acceleration_time_s;
    const std::size_t move_samples = 2 * acceleration_samples + 1;
    const std::size_t dwell_samples = static_cast<std::size_t>(std::llround(
        definition.settling_window_s / definition.sample_period_s));
    const std::size_t sample_count = move_samples + dwell_samples;
    std::array<std::vector<double>, kJointCount> positions;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        positions[joint].assign(sample_count, definition.goal_rad[joint]);
    }
    for (std::size_t sample = 0; sample < move_samples; ++sample) {
        const double time_s = static_cast<double>(sample) * definition.sample_period_s;
        double traveled = 0.0;
        if (sample <= acceleration_samples) {
            traveled = 0.5 * adjusted_acceleration * time_s * time_s;
        } else {
            const double deceleration_time_s = time_s - acceleration_time_s;
            traveled = 0.5 * adjusted_acceleration * acceleration_time_s *
                           acceleration_time_s +
                       peak_velocity * deceleration_time_s -
                       0.5 * adjusted_acceleration * deceleration_time_s *
                           deceleration_time_s;
        }
        positions[0][sample] = definition.start_rad[0] + direction * traveled;
    }
    std::array<std::vector<double>, kJointCount> velocities;
    std::array<std::vector<double>, kJointCount> accelerations;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        velocities[joint] = Gradient(positions[joint], definition.sample_period_s);
        accelerations[joint] =
            Gradient(velocities[joint], definition.sample_period_s);
    }

    trajectory_msgs::msg::JointTrajectory message;
    message.header.frame_id = "standard_bots_base";
    message.joint_names.assign(kJointOrder.begin(), kJointOrder.end());
    message.points.reserve(sample_count);
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.reserve(kJointCount);
        point.velocities.reserve(kJointCount);
        point.accelerations.reserve(kJointCount);
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            point.positions.push_back(positions[joint][sample]);
            point.velocities.push_back(velocities[joint][sample]);
            point.accelerations.push_back(accelerations[joint][sample]);
        }
        point.time_from_start =
            RosDuration(static_cast<double>(sample) * definition.sample_period_s);
        message.points.push_back(std::move(point));
    }
    return message;
}

trajectory_msgs::msg::JointTrajectory ShapeTrajectoryWithParity(
    const trajectory_msgs::msg::JointTrajectory& input,
    const std::filesystem::path& assets_directory) {
    using reforge::control::shaper::backend::JointTrajectoryInput;
    using reforge::control::shaper::backend::NativeShaper;
    using reforge::control::shaper::backend::ProcessTrajectoryRequest;
    using reforge::control::shaper::ros2::TrajectoryOptions;
    using reforge::native::Matrix;
    using reforge::native::Vector;

    NativeShaper adapter_shaper(MakeBackendConfiguration(assets_directory));
    TrajectoryOptions adapter_options;
    adapter_options.expected_joint_names.assign(kJointOrder.begin(), kJointOrder.end());
    adapter_options.vibration_shaping_weight = 1.0;
    adapter_options.residual_shaping_strategy = std::nullopt;
    adapter_options.finalize_tail = false;
    const auto shaped = reforge::control::shaper::ros2::ProcessTrajectory(
        adapter_shaper, input, adapter_options);

    const auto rows = static_cast<Eigen::Index>(input.points.size());
    Matrix positions(rows, static_cast<Eigen::Index>(kJointCount));
    Matrix velocities(rows, static_cast<Eigen::Index>(kJointCount));
    Matrix accelerations(rows, static_cast<Eigen::Index>(kJointCount));
    Vector time(rows);
    for (Eigen::Index row = 0; row < rows; ++row) {
        const auto& point = input.points[static_cast<std::size_t>(row)];
        time(row) = DurationSeconds(point.time_from_start);
        for (Eigen::Index column = 0;
             column < static_cast<Eigen::Index>(kJointCount); ++column) {
            const auto joint = static_cast<std::size_t>(column);
            positions(row, column) = point.positions[joint];
            velocities(row, column) = point.velocities[joint];
            accelerations(row, column) = point.accelerations[joint];
        }
    }
    ProcessTrajectoryRequest request;
    request.input = JointTrajectoryInput{
        std::move(positions), std::move(velocities), std::move(accelerations),
        std::move(time)};
    request.vibration_shaping_weight = 1.0;
    request.residual_shaping_strategy = std::nullopt;
    request.finalize_tail = false;
    NativeShaper native_shaper(MakeBackendConfiguration(assets_directory));
    const auto native = native_shaper.ProcessTrajectory(request);
    if (shaped.points.size() != static_cast<std::size_t>(native.positions.rows())) {
        throw std::runtime_error("ROS/native parity failed: sample count differs");
    }
    for (std::size_t row = 0; row < shaped.points.size(); ++row) {
        const auto& point = shaped.points[row];
        if (std::abs(DurationSeconds(point.time_from_start) -
                     native.time(static_cast<Eigen::Index>(row))) > kNumericTolerance) {
            throw std::runtime_error("ROS/native parity failed: timestamp differs");
        }
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            const auto r = static_cast<Eigen::Index>(row);
            const auto c = static_cast<Eigen::Index>(joint);
            if (std::abs(point.positions[joint] - native.positions(r, c)) >
                    kNumericTolerance ||
                std::abs(point.velocities[joint] - native.velocities(r, c)) >
                    kNumericTolerance ||
                std::abs(point.accelerations[joint] - native.accelerations(r, c)) >
                    kNumericTolerance) {
                throw std::runtime_error("ROS/native parity failed: kinematics differ");
            }
        }
    }
    return shaped;
}

void ValidateTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message,
    const TrialDefinition& definition,
    const SafetyLimits& limits) {
    if (message.joint_names !=
        std::vector<std::string>(kJointOrder.begin(), kJointOrder.end())) {
        throw std::invalid_argument("trajectory joint order must be joint0 through joint5");
    }
    if (message.points.size() != 1053) {
        throw std::invalid_argument("trajectory must contain exactly 1,053 points");
    }
    double previous_time_s = -definition.sample_period_s;
    for (std::size_t sample = 0; sample < message.points.size(); ++sample) {
        const auto& point = message.points[sample];
        if (point.positions.size() != kJointCount ||
            point.velocities.size() != kJointCount ||
            point.accelerations.size() != kJointCount || !point.effort.empty()) {
            throw std::invalid_argument("trajectory point has invalid field sizes or effort");
        }
        RequireFinite(point.positions, "positions");
        RequireFinite(point.velocities, "velocities");
        RequireFinite(point.accelerations, "accelerations");
        const double time_s = DurationSeconds(point.time_from_start);
        if (sample == 0 && std::abs(time_s) > kNumericTolerance) {
            throw std::invalid_argument("first trajectory timestamp must be zero");
        }
        if (sample > 0 &&
            (time_s <= previous_time_s ||
             std::abs(time_s - previous_time_s - definition.sample_period_s) >
                 1.0e-9)) {
            throw std::invalid_argument(
                "trajectory timestamps must be strictly increasing at 0.005 s");
        }
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            if (point.positions[joint] < limits.minimum_position_rad[joint] ||
                point.positions[joint] > limits.maximum_position_rad[joint]) {
                throw std::invalid_argument("trajectory exceeds a joint position limit");
            }
            if (std::abs(point.velocities[joint]) >
                    limits.max_velocity_rad_s + kNumericTolerance ||
                std::abs(point.accelerations[joint]) >
                    limits.max_acceleration_rad_s2 + kNumericTolerance) {
                throw std::invalid_argument("trajectory exceeds a derivative limit");
            }
            if (std::abs(point.positions[joint] -
                         message.points.front().positions[joint]) >
                limits.max_displacement_rad + kNumericTolerance) {
                throw std::invalid_argument("trajectory exceeds displacement limit");
            }
            if (joint > 0 &&
                std::abs(point.positions[joint] - definition.start_rad[joint]) >
                    kNumericTolerance) {
                throw std::invalid_argument("frozen Phase A trial may move only joint0");
            }
        }
        previous_time_s = time_s;
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (std::abs(message.points.front().positions[joint] -
                     definition.start_rad[joint]) > kNumericTolerance ||
            std::abs(message.points.back().positions[joint] -
                     definition.goal_rad[joint]) > kNumericTolerance) {
            throw std::invalid_argument("trajectory start or goal differs from Phase A");
        }
    }
    if (std::abs(previous_time_s - limits.max_duration_s) > kNumericTolerance) {
        throw std::invalid_argument("trajectory duration must be exactly 5.26 s");
    }
}

void ValidatePreflight(
    const PreflightSnapshot& snapshot,
    const trajectory_msgs::msg::JointTrajectory& message,
    const SafetyLimits& limits) {
    if (!(limits.current_pose_tolerance_rad > 0.0) ||
        !(limits.following_error_limit_rad > 0.0)) {
        throw std::invalid_argument(
            "execution requires explicit positive pose and following-error limits");
    }
    if (!snapshot.command_subscriber_present) {
        throw std::runtime_error("expected trajectory command subscriber is absent");
    }
    if (!snapshot.control_active) {
        throw std::runtime_error("ROS control state is not active");
    }
    if (!snapshot.joint_state_fresh || !snapshot.imu_fresh) {
        throw std::runtime_error("joint-state or IMU feedback is stale or missing");
    }
    if (snapshot.joint_names !=
            std::vector<std::string>(kJointOrder.begin(), kJointOrder.end()) ||
        snapshot.current_position_rad.size() != kJointCount) {
        throw std::runtime_error("joint-state feedback order is not joint0 through joint5");
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (!std::isfinite(snapshot.current_position_rad[joint]) ||
            std::abs(snapshot.current_position_rad[joint] -
                     message.points.front().positions[joint]) >
                limits.current_pose_tolerance_rad) {
            throw std::runtime_error("current pose does not match the first trajectory point");
        }
    }
}

Analysis AnalyzeCapture(
    const trajectory_msgs::msg::JointTrajectory& command,
    const Capture& capture,
    double maximum_gap_s) {
    const CaptureTiming timing = ValidateCaptureTiming(capture, maximum_gap_s);
    double squared_error_sum = 0.0;
    std::size_t error_values = 0;
    double maximum_error = 0.0;
    for (const auto& record : capture.joint_states) {
        if (record.message.name !=
                std::vector<std::string>(kJointOrder.begin(), kJointOrder.end()) ||
            record.message.position.size() != kJointCount) {
            throw std::runtime_error("recorded joint state has invalid order or size");
        }
        const auto command_index = NearestCommandIndex(command, record.elapsed_s);
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            if (!std::isfinite(record.message.position[joint])) {
                throw std::runtime_error(
                    "recorded joint state contains a non-finite value");
            }
            const double error = record.message.position[joint] -
                                 command.points[command_index].positions[joint];
            maximum_error = std::max(maximum_error, std::abs(error));
            squared_error_sum += error * error;
            ++error_values;
        }
    }
    double peak_acceleration = 0.0;
    for (const auto& record : capture.imu) {
        const auto& value = record.message.linear_acceleration;
        const double magnitude = std::sqrt(
            value.x * value.x + value.y * value.y + value.z * value.z);
        if (!std::isfinite(magnitude)) {
            throw std::runtime_error("recorded IMU contains a non-finite value");
        }
        peak_acceleration = std::max(peak_acceleration, magnitude);
    }
    return Analysis{
        capture.joint_states.size(), maximum_error,
        std::sqrt(squared_error_sum / static_cast<double>(error_values)),
        peak_acceleration, timing};
}

CaptureTiming ValidateCaptureTiming(
    const Capture& capture,
    double maximum_gap_s) {
    if (capture.joint_states.empty() || capture.imu.empty()) {
        throw std::runtime_error("capture requires both joint-state and IMU samples");
    }
    if (!(maximum_gap_s > 0.0)) {
        throw std::invalid_argument("maximum recorder gap must be positive");
    }
    const CaptureTiming timing = AnalyzeCaptureTiming(capture);
    const auto require_gap = [maximum_gap_s](
                                 std::string_view stream,
                                 const StreamTiming& stream_timing) {
        if (!stream_timing.arrival_times_strictly_increasing ||
            !(stream_timing.maximum_arrival_gap_s > 0.0)) {
            throw std::runtime_error(
                "recorder stream '" + std::string(stream) +
                "' has duplicate or non-monotonic callback arrival times");
        }
        if (stream_timing.maximum_arrival_gap_s > maximum_gap_s) {
            std::ostringstream error;
            error << std::setprecision(9)
                  << "recorder stream '" << stream
                  << "' exceeded the frozen callback-arrival gap limit: measured "
                  << stream_timing.maximum_arrival_gap_s << " s > "
                  << maximum_gap_s << " s";
            if (stream_timing.maximum_source_stamp_gap_s.has_value()) {
                error << "; measured maximum source-header gap "
                      << *stream_timing.maximum_source_stamp_gap_s << " s";
            } else {
                error << "; source-header gap unavailable (valid stamps "
                      << stream_timing.source_stamp_sample_count << '/'
                      << stream_timing.sample_count << ')';
            }
            throw std::runtime_error(error.str());
        }
    };
    if (capture.command_subscriber_lost) {
        throw std::runtime_error(
            "trajectory command subscriber was lost during trial");
    }
    require_gap("joint_state", timing.joint_state);
    require_gap("imu", timing.imu);
    if (capture.joint_state_age_at_end_s > maximum_gap_s) {
        std::ostringstream error;
        error << std::setprecision(9)
              << "recorder stream 'joint_state' was stale at trial end: measured age "
              << capture.joint_state_age_at_end_s << " s > " << maximum_gap_s << " s";
        throw std::runtime_error(error.str());
    }
    if (capture.imu_age_at_end_s > maximum_gap_s) {
        std::ostringstream error;
        error << std::setprecision(9)
              << "recorder stream 'imu' was stale at trial end: measured age "
              << capture.imu_age_at_end_s << " s > " << maximum_gap_s << " s";
        throw std::runtime_error(error.str());
    }
    return timing;
}

CaptureTiming AnalyzeCaptureTiming(const Capture& capture) {
    const auto stream_timing = [](const auto& records) {
        StreamTiming timing;
        timing.sample_count = records.size();
        if (records.size() > 1) {
            for (std::size_t index = 1; index < records.size(); ++index) {
                const double gap_s =
                    records[index].elapsed_s - records[index - 1].elapsed_s;
                if (!(gap_s > 0.0)) {
                    timing.arrival_times_strictly_increasing = false;
                } else {
                    timing.maximum_arrival_gap_s = std::max(
                        timing.maximum_arrival_gap_s, gap_s);
                }
            }
        }

        bool all_source_stamps_valid = !records.empty();
        double maximum_source_gap_s = 0.0;
        double previous_source_s = 0.0;
        bool have_previous_source = false;
        for (const auto& record : records) {
            const double source_s =
                static_cast<double>(record.message.header.stamp.sec) +
                static_cast<double>(record.message.header.stamp.nanosec) * 1.0e-9;
            if (!(source_s > 0.0) || !std::isfinite(source_s)) {
                all_source_stamps_valid = false;
                continue;
            }
            ++timing.source_stamp_sample_count;
            if (have_previous_source) {
                const double gap_s = source_s - previous_source_s;
                if (!(gap_s > 0.0)) {
                    all_source_stamps_valid = false;
                } else {
                    maximum_source_gap_s = std::max(maximum_source_gap_s, gap_s);
                }
            }
            previous_source_s = source_s;
            have_previous_source = true;
        }
        if (all_source_stamps_valid && timing.source_stamp_sample_count > 1) {
            timing.maximum_source_stamp_gap_s = maximum_source_gap_s;
        }
        return timing;
    };
    return CaptureTiming{
        stream_timing(capture.joint_states), stream_timing(capture.imu)};
}

Capture ExecutePreparedTrial(
    QualificationTransport& transport,
    const trajectory_msgs::msg::JointTrajectory& command,
    const TrialDefinition& definition,
    const SafetyLimits& limits,
    const std::function<void(const Capture&)>& capture_sink) {
    ValidateTrajectory(command, definition, limits);
    ValidatePreflight(transport.Snapshot(), command, limits);
    const double duration_s = DurationSeconds(command.points.back().time_from_start) +
                              definition.sample_period_s;
    Capture capture = transport.PublishAndRecord(command, duration_s);
    if (capture_sink) {
        capture_sink(capture);
    }
    const Analysis analysis = AnalyzeCapture(
        command, capture, limits.recorder_max_gap_s);
    if (analysis.maximum_following_error_rad > limits.following_error_limit_rad) {
        throw std::runtime_error("measured following error crossed the frozen limit");
    }
    return capture;
}

void WriteTrajectoryCsv(
    const std::filesystem::path& path,
    const trajectory_msgs::msg::JointTrajectory& message) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot open trajectory output: " + path.string());
    }
    output << "time_s";
    for (const auto& name : kJointOrder) output << ',' << name << "_position_rad";
    for (const auto& name : kJointOrder) output << ',' << name << "_velocity_rad_s";
    for (const auto& name : kJointOrder) output << ',' << name << "_acceleration_rad_s2";
    output << '\n' << std::setprecision(17);
    for (const auto& point : message.points) {
        output << DurationSeconds(point.time_from_start);
        for (const auto value : point.positions) output << ',' << value;
        for (const auto value : point.velocities) output << ',' << value;
        for (const auto value : point.accelerations) output << ',' << value;
        output << '\n';
    }
}

void WriteCaptureCsv(
    const std::filesystem::path& directory,
    const Capture& capture) {
    std::filesystem::create_directories(directory);
    std::ofstream joints(directory / "joint_states.csv");
    std::ofstream imu(directory / "imu.csv");
    if (!joints || !imu) {
        throw std::runtime_error("cannot open recorder evidence outputs");
    }
    joints << "arrival_elapsed_s,source_stamp_s";
    for (const auto& name : kJointOrder) joints << ',' << name << "_position_rad";
    joints << '\n' << std::setprecision(17);
    for (const auto& record : capture.joint_states) {
        const double source_s = static_cast<double>(record.message.header.stamp.sec) +
                                static_cast<double>(record.message.header.stamp.nanosec) * 1.0e-9;
        joints << record.elapsed_s << ',' << source_s;
        for (const auto value : record.message.position) joints << ',' << value;
        joints << '\n';
    }
    imu << "arrival_elapsed_s,source_stamp_s,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s\n"
        << std::setprecision(17);
    for (const auto& record : capture.imu) {
        const double source_s = static_cast<double>(record.message.header.stamp.sec) +
                                static_cast<double>(record.message.header.stamp.nanosec) * 1.0e-9;
        imu << record.elapsed_s << ',' << source_s << ','
            << record.message.linear_acceleration.x << ','
            << record.message.linear_acceleration.y << ','
            << record.message.linear_acceleration.z << ','
            << record.message.angular_velocity.x << ','
            << record.message.angular_velocity.y << ','
            << record.message.angular_velocity.z << '\n';
    }
}

void WriteResultManifest(
    const std::filesystem::path& path,
    TrialKind kind,
    bool executed,
    const trajectory_msgs::msg::JointTrajectory& command,
    const std::optional<Analysis>& analysis,
    const std::string& endpoint,
    const std::string& robot_id) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot open result manifest: " + path.string());
    }
    output << std::setprecision(17)
           << "{\n"
           << "  \"schema\": \"reforge_standard_bots_hardware_qualification_result\",\n"
           << "  \"schema_version\": 1,\n"
           << "  \"reforge_interface_revision\": \"22dc9426cf889d127408a8374b59a37263b87da4\",\n"
           << "  \"reforge_core_revision\": \"c323c6ebf973d17aa39355de5b4964b6ea3009ca\",\n"
           << "  \"package_version\": \"2.0.9-908\",\n"
           << "  \"artifact_sha256\": \"632ee3e10b721b5ae8e50fd7505a6b16a02bfe148e250af56262e8f27e8df6c1\",\n"
           << "  \"base_package_sha256\": \"35828371832860fabc8e2f88d48368e648034a86695a5e2e0c935c148e80415d\",\n"
           << "  \"ros2_package_sha256\": \"4d1c90dea9c07d393752e4e5216733bda710093be447b68802eeced3f77c76b9\",\n"
           << "  \"dataset_sha256\": \"fab8ec7860e7b9a860d61642d06f63075b8aa3200d33de21d3008bd5479917a6\",\n"
           << "  \"source_file\": \"robotData_motion_pose10_axis0.csv\",\n"
           << "  \"source_file_sha256\": \"273223833392f0d5050442bcc05f107c215c36881673a078ec4bb2904147b473\",\n"
           << "  \"model_bundle_sha256\": \"8b1ce914af225f691cfe0ac14a29bb53321fded2400af42ff7834187f12ae996\",\n"
           << "  \"native_model_sha256\": \"903daffc5d72d6b43a461c4691ee50f2d84768d687148ab6eb9cf6118704259e\",\n"
           << "  \"urdf_sha256\": \"f917aca45ba89d741c15cab53e308eb6e4a107ccb2539f95ea26ad33aa3bc8e5\",\n"
           << "  \"trial\": \"" << (kind == TrialKind::kShaped ? "shaped" : "unshaped") << "\",\n"
           << "  \"executed\": " << (executed ? "true" : "false") << ",\n"
           << "  \"no_io\": " << (executed ? "false" : "true") << ",\n"
           << "  \"endpoint\": \"" << endpoint << "\",\n"
           << "  \"robot_id\": \"" << robot_id << "\",\n"
           << "  \"joint_order\": [\"joint0\", \"joint1\", \"joint2\", \"joint3\", \"joint4\", \"joint5\"],\n"
           << "  \"command_topic_template\": \"/<BOT_ID>/ro1/hardware/joint_trajectory\",\n"
           << "  \"joint_state_topic_template\": \"/<BOT_ID>/ro1/hardware/joint_state\",\n"
           << "  \"imu_topic_template\": \"/<BOT_ID>/ro1/hardware/end_effector_imu\",\n"
           << "  \"required_ros_control_state\": \"Enabled\",\n"
           << "  \"sample_period_s\": " << kSamplePeriodS << ",\n"
           << "  \"maximum_feedback_age_s\": 0.05,\n"
           << "  \"maximum_recorder_gap_s\": 0.05,\n"
           << "  \"maximum_first_point_error_rad_per_joint\": 0.02,\n"
           << "  \"maximum_following_error_rad_per_joint\": 0.05,\n"
           << "  \"urdf_position_limit_margin_rad\": 0.05,\n"
           << "  \"max_velocity_rad_s\": 1.25,\n"
           << "  \"max_acceleration_rad_s2\": 2.0,\n"
           << "  \"max_displacement_rad\": 0.7853981633974483,\n"
           << "  \"settling_window_s\": 4.0,\n"
           << "  \"required_point_count\": 1053,\n"
           << "  \"required_duration_s\": 5.26,\n"
           << "  \"point_count\": " << command.points.size() << ",\n"
           << "  \"duration_s\": " << DurationSeconds(command.points.back().time_from_start);
    if (analysis.has_value()) {
        output << ",\n  \"analysis\": {\n"
               << "    \"aligned_samples\": " << analysis->aligned_samples << ",\n"
               << "    \"maximum_following_error_rad\": "
               << analysis->maximum_following_error_rad << ",\n"
               << "    \"rms_following_error_rad\": "
               << analysis->rms_following_error_rad << ",\n"
               << "    \"peak_linear_acceleration_m_s2\": "
               << analysis->peak_linear_acceleration_m_s2 << ",\n"
               << "    \"joint_state_maximum_arrival_gap_s\": "
               << analysis->timing.joint_state.maximum_arrival_gap_s << ",\n"
               << "    \"imu_maximum_arrival_gap_s\": "
               << analysis->timing.imu.maximum_arrival_gap_s << ",\n"
               << "    \"joint_state_maximum_source_stamp_gap_s\": ";
        if (analysis->timing.joint_state.maximum_source_stamp_gap_s.has_value()) {
            output << *analysis->timing.joint_state.maximum_source_stamp_gap_s;
        } else {
            output << "null";
        }
        output << ",\n    \"imu_maximum_source_stamp_gap_s\": ";
        if (analysis->timing.imu.maximum_source_stamp_gap_s.has_value()) {
            output << *analysis->timing.imu.maximum_source_stamp_gap_s;
        } else {
            output << "null";
        }
        output << "\n  }\n";
    } else {
        output << "\n";
    }
    output << "}\n";
}

}  // namespace reforge::qualification
