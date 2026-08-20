#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace reforge::qualification {

inline constexpr std::size_t kJointCount = 6;
inline constexpr double kSamplePeriodS = 0.005;
inline const std::array<std::string, kJointCount> kJointOrder = {
    "joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};

enum class TrialKind { kUnshaped, kShaped };

struct TrialDefinition final {
    std::array<double, kJointCount> start_rad{};
    std::array<double, kJointCount> goal_rad{};
    double sample_period_s = kSamplePeriodS;
    double max_velocity_rad_s = 1.25;
    double max_acceleration_rad_s2 = 2.0;
    double settling_window_s = 4.0;
};

struct SafetyLimits final {
    std::array<double, kJointCount> minimum_position_rad{};
    std::array<double, kJointCount> maximum_position_rad{};
    double max_velocity_rad_s = 1.25;
    double max_acceleration_rad_s2 = 2.0;
    double max_displacement_rad = 0.7853981633974483;
    double max_duration_s = 5.26;
    double current_pose_tolerance_rad = 0.02;
    double following_error_limit_rad = 0.05;
    double feedback_max_age_s = 0.05;
    double recorder_max_gap_s = 0.05;
};

struct PreflightSnapshot final {
    bool command_subscriber_present = false;
    bool control_active = false;
    bool joint_state_fresh = false;
    bool imu_fresh = false;
    std::vector<std::string> joint_names;
    std::vector<double> current_position_rad;
};

struct JointStateRecord final {
    double elapsed_s = 0.0;
    sensor_msgs::msg::JointState message;
};

struct ImuRecord final {
    double elapsed_s = 0.0;
    sensor_msgs::msg::Imu message;
};

struct Capture final {
    std::vector<JointStateRecord> joint_states;
    std::vector<ImuRecord> imu;
};

struct Analysis final {
    std::size_t aligned_samples = 0;
    double maximum_following_error_rad = 0.0;
    double rms_following_error_rad = 0.0;
    double peak_linear_acceleration_m_s2 = 0.0;
};

class QualificationTransport {
public:
    virtual ~QualificationTransport() = default;
    [[nodiscard]] virtual PreflightSnapshot Snapshot() = 0;
    [[nodiscard]] virtual Capture PublishAndRecord(
        const trajectory_msgs::msg::JointTrajectory& message,
        double recording_duration_s) = 0;
};

[[nodiscard]] TrialDefinition FrozenPhaseATrial();
[[nodiscard]] SafetyLimits ConservativeSafetyLimits();
[[nodiscard]] trajectory_msgs::msg::JointTrajectory GenerateUnshapedTrajectory(
    const TrialDefinition& definition);
[[nodiscard]] trajectory_msgs::msg::JointTrajectory ShapeTrajectoryWithParity(
    const trajectory_msgs::msg::JointTrajectory& input,
    const std::filesystem::path& assets_directory);
void ValidateTrajectory(
    const trajectory_msgs::msg::JointTrajectory& message,
    const TrialDefinition& definition,
    const SafetyLimits& limits);
void ValidatePreflight(
    const PreflightSnapshot& snapshot,
    const trajectory_msgs::msg::JointTrajectory& message,
    const SafetyLimits& limits);
[[nodiscard]] Analysis AnalyzeCapture(
    const trajectory_msgs::msg::JointTrajectory& command,
    const Capture& capture,
    double maximum_gap_s = 0.05);
[[nodiscard]] Capture ExecutePreparedTrial(
    QualificationTransport& transport,
    const trajectory_msgs::msg::JointTrajectory& command,
    const TrialDefinition& definition,
    const SafetyLimits& limits);
void WriteTrajectoryCsv(
    const std::filesystem::path& path,
    const trajectory_msgs::msg::JointTrajectory& message);
void WriteCaptureCsv(
    const std::filesystem::path& directory,
    const Capture& capture);
void WriteResultManifest(
    const std::filesystem::path& path,
    TrialKind kind,
    bool executed,
    const trajectory_msgs::msg::JointTrajectory& command,
    const std::optional<Analysis>& analysis,
    const std::string& endpoint,
    const std::string& robot_id);

}  // namespace reforge::qualification
