#include "standard_bots_ros_transport.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace standard_bots_shaper {
namespace {

[[nodiscard]] std::int64_t DurationNanoseconds(
    const builtin_interfaces::msg::Duration& duration) {
  if (duration.sec < 0 || duration.nanosec >= 1'000'000'000U) {
    throw std::invalid_argument("trajectory duration is not normalized");
  }
  constexpr auto kMaxSeconds =
      std::numeric_limits<std::int64_t>::max() / 1'000'000'000;
  if (duration.sec > kMaxSeconds) {
    throw std::invalid_argument("trajectory duration is too large");
  }
  return static_cast<std::int64_t>(duration.sec) * 1'000'000'000 +
         static_cast<std::int64_t>(duration.nanosec);
}

void RequireFinite(const std::vector<double>& values, const char* field) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string(field) + " contains a non-finite value");
    }
  }
}

[[nodiscard]] trajectory_msgs::msg::JointTrajectory MakeOnePointMessageUnchecked(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::size_t point_index,
    std::int64_t wire_time_nanoseconds) {
  trajectory_msgs::msg::JointTrajectory result;
  result.header = trajectory.header;
  result.joint_names = trajectory.joint_names;
  result.points.push_back(trajectory.points[point_index]);
  result.points.front().time_from_start.sec = static_cast<std::int32_t>(
      wire_time_nanoseconds / 1'000'000'000);
  result.points.front().time_from_start.nanosec = static_cast<std::uint32_t>(
      wire_time_nanoseconds % 1'000'000'000);
  return result;
}

void ValidateWireTimestamp(std::int64_t wire_time_nanoseconds) {
  if (wire_time_nanoseconds < 0) {
    throw std::invalid_argument("wire timestamp must be non-negative");
  }
  constexpr auto kMaxWireNanoseconds =
      (static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) +
       1) *
          1'000'000'000 -
      1;
  if (wire_time_nanoseconds > kMaxWireNanoseconds) {
    throw std::invalid_argument("wire timestamp is too large for ROS time");
  }
}

}  // namespace

void ValidateCompleteTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
  if (trajectory.joint_names != kJointOrder) {
    throw std::invalid_argument("trajectory joint order must be joint0 through joint5");
  }
  if (trajectory.points.size() != kExpectedPointCount) {
    throw std::invalid_argument("trajectory must contain exactly 1,053 points");
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto& point = trajectory.points[index];
    if (point.positions.size() != kJointCount ||
        point.velocities.size() != kJointCount ||
        point.accelerations.size() != kJointCount || !point.effort.empty()) {
      throw std::invalid_argument(
          "trajectory points require six positions, velocities, and accelerations");
    }
    RequireFinite(point.positions, "positions");
    RequireFinite(point.velocities, "velocities");
    RequireFinite(point.accelerations, "accelerations");
    const auto timestamp = DurationNanoseconds(point.time_from_start);
    const auto expected = static_cast<std::int64_t>(index) *
                          kSamplePeriodNanoseconds;
    if (timestamp != expected) {
      throw std::invalid_argument(
          "trajectory timestamps must start at zero and advance by 0.005 s");
    }
  }
}

trajectory_msgs::msg::JointTrajectory MakeOnePointMessage(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    std::size_t point_index,
    std::int64_t wire_time_nanoseconds) {
  ValidateCompleteTrajectory(trajectory);
  if (point_index >= trajectory.points.size()) {
    throw std::out_of_range("trajectory point index is out of range");
  }
  ValidateWireTimestamp(wire_time_nanoseconds);
  return MakeOnePointMessageUnchecked(
      trajectory, point_index, wire_time_nanoseconds);
}

std::vector<PublicationRecord> StreamOnePointTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const PublishFunction& publish,
    const ClockFunction& clock_now_nanoseconds,
    const SleepFunction& sleep) {
  if (!publish || !clock_now_nanoseconds) {
    throw std::invalid_argument("publish and clock callbacks are required");
  }
  ValidateCompleteTrajectory(trajectory);
  const auto epoch = std::chrono::steady_clock::now();
  std::vector<PublicationRecord> records;
  records.reserve(trajectory.points.size());
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    if (index != 0) {
      if (sleep) {
        sleep(std::chrono::nanoseconds(kSamplePeriodNanoseconds));
      } else {
        std::this_thread::sleep_until(
            epoch + std::chrono::nanoseconds(
                        static_cast<std::int64_t>(index) *
                        kSamplePeriodNanoseconds));
      }
    }
    const auto wire_time = clock_now_nanoseconds();
    ValidateWireTimestamp(wire_time);
    publish(MakeOnePointMessageUnchecked(trajectory, index, wire_time));
    records.push_back(PublicationRecord{index, wire_time});
  }
  return records;
}

}  // namespace standard_bots_shaper
