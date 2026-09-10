#pragma once

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace standard_bots_shaper {

inline constexpr std::size_t kJointCount = 6;
inline constexpr std::size_t kExpectedPointCount = 1053;
inline constexpr std::int64_t kSamplePeriodNanoseconds = 5'000'000;
inline constexpr double kSamplePeriodSeconds = 0.005;

inline const std::vector<std::string> kJointOrder = {
    "joint0", "joint1", "joint2", "joint3", "joint4", "joint5"};

struct PublicationRecord final {
  std::size_t point_index = 0;
  std::int64_t wire_time_nanoseconds = 0;
};

using PublishFunction =
    std::function<void(const trajectory_msgs::msg::JointTrajectory&)>;
using ClockFunction = std::function<std::int64_t()>;
using SleepFunction = std::function<void(std::chrono::nanoseconds)>;

/** Validate the complete trajectory before crossing the robot boundary.
 *
 * This is deliberately limited to transport-shape invariants. It does not
 * perform robot readiness, safety, telemetry, or evidence operations.
 */
void ValidateCompleteTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

/** Make the one-point message expected by the Standard Bots bridge.
 *
 * The bridge interprets `time_from_start` as the current ROS clock timestamp,
 * rather than as a duration from the trajectory start. The caller supplies
 * that clock value explicitly so the convention is visible and testable.
 */
[[nodiscard]] trajectory_msgs::msg::JointTrajectory MakeOnePointMessage(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::size_t point_index,
    std::int64_t wire_time_nanoseconds);

/** Publish one ordered point per sample using the bridge wire convention.
 *
 * With no `sleep` callback, the function schedules publication on a monotonic
 * 0.005-second grid. Tests and an application-owned scheduler may provide a
 * callback to replace that wait.
 */
[[nodiscard]] std::vector<PublicationRecord> StreamOnePointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const PublishFunction& publish,
    const ClockFunction& clock_now_nanoseconds,
    const SleepFunction& sleep = {});

}  // namespace standard_bots_shaper
