#include "../standard_bots_ros_transport.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {

void Check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

trajectory_msgs::msg::JointTrajectory ValidTrajectory() {
  trajectory_msgs::msg::JointTrajectory result;
  result.joint_names = standard_bots_shaper::kJointOrder;
  result.points.resize(standard_bots_shaper::kExpectedPointCount);
  for (std::size_t index = 0; index < result.points.size(); ++index) {
    auto& point = result.points[index];
    point.positions.assign(standard_bots_shaper::kJointCount, 0.0);
    point.velocities.assign(standard_bots_shaper::kJointCount, 0.0);
    point.accelerations.assign(standard_bots_shaper::kJointCount, 0.0);
    point.positions[0] = 0.25 * static_cast<double>(index) /
                         static_cast<double>(result.points.size() - 1);
    const auto ns = static_cast<std::int64_t>(index) *
                    standard_bots_shaper::kSamplePeriodNanoseconds;
    point.time_from_start.sec = static_cast<std::int32_t>(ns / 1'000'000'000);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000);
  }
  return result;
}

template <typename Function>
void ExpectInvalid(Function&& function) {
  bool rejected = false;
  try {
    function();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "invalid trajectory was accepted");
}

void TestValidation() {
  auto trajectory = ValidTrajectory();
  standard_bots_shaper::ValidateCompleteTrajectory(trajectory);
  trajectory.points.pop_back();
  ExpectInvalid([&] { standard_bots_shaper::ValidateCompleteTrajectory(trajectory); });
  trajectory = ValidTrajectory();
  trajectory.points[2].time_from_start.nanosec++;
  ExpectInvalid([&] { standard_bots_shaper::ValidateCompleteTrajectory(trajectory); });
  trajectory = ValidTrajectory();
  trajectory.joint_names[0] = "wrong_joint";
  ExpectInvalid([&] { standard_bots_shaper::ValidateCompleteTrajectory(trajectory); });
  trajectory = ValidTrajectory();
  trajectory.points[0].positions[0] = NAN;
  ExpectInvalid([&] { standard_bots_shaper::ValidateCompleteTrajectory(trajectory); });
}

void TestStreamOrderAndWireTime() {
  const auto trajectory = ValidTrajectory();
  std::size_t count = 0;
  std::int64_t waited_ns = 0;
  const auto records = standard_bots_shaper::StreamOnePointTrajectory(
      trajectory,
      [&](const auto& message) {
        Check(message.joint_names == trajectory.joint_names,
              "stream joint order changed");
        Check(message.points.size() == 1, "stream message was not one point");
        Check(message.points.front().positions ==
                  trajectory.points[count].positions,
              "stream point order changed");
        const auto expected_wire_ns =
            123'000'000'000 + static_cast<std::int64_t>(count) *
                                  standard_bots_shaper::kSamplePeriodNanoseconds;
        Check(message.points.front().time_from_start.sec ==
                  expected_wire_ns / 1'000'000'000,
              "wire timestamp seconds changed");
        Check(message.points.front().time_from_start.nanosec ==
                  expected_wire_ns % 1'000'000'000,
              "wire timestamp nanoseconds changed");
        ++count;
      },
      [index = std::int64_t{0}]() mutable {
        return 123'000'000'000 + index++ *
               standard_bots_shaper::kSamplePeriodNanoseconds;
      },
      [&waited_ns](std::chrono::nanoseconds duration) {
        Check(duration.count() ==
                  standard_bots_shaper::kSamplePeriodNanoseconds,
              "stream wait period changed");
        waited_ns += duration.count();
      });
  Check(count == standard_bots_shaper::kExpectedPointCount,
        "stream did not publish all points");
  Check(records.size() == count, "publication record count changed");
  Check(records.front().point_index == 0, "stream did not start at point zero");
  Check(records.back().point_index == count - 1,
        "stream did not finish at the final point");
  Check(records.front().wire_time_nanoseconds == 123'000'000'000,
        "first wire timestamp changed");
  const auto last_expected = 123'000'000'000 +
                             static_cast<std::int64_t>(count - 1) *
                                 standard_bots_shaper::kSamplePeriodNanoseconds;
  Check(records.back().wire_time_nanoseconds == last_expected,
        "last wire timestamp changed");
  Check(waited_ns == static_cast<std::int64_t>(count - 1) *
                         standard_bots_shaper::kSamplePeriodNanoseconds,
        "stream cadence changed");
}

}  // namespace

int main() {
  try {
    TestValidation();
    TestStreamOrderAndWireTime();
    return 0;
  } catch (const std::exception& error) {
    return (std::cerr << "transport test failed: " << error.what() << '\n', 1);
  }
}
