#include "standard_bots_ros_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <builtin_interfaces/msg/duration.hpp>

#include "reforge_core/control/shaper/backend/backend_configuration.hpp"
#include "reforge_core/control/shaper/backend/backend_requests.hpp"
#include "reforge_core/control/shaper/backend/native_shaper.hpp"
#include "reforge_core/control/shaper/ros2/joint_trajectory_adapter.hpp"

#ifndef SHAPER_CPP_DEFAULT_MODEL_DIRECTORY
#define SHAPER_CPP_DEFAULT_MODEL_DIRECTORY "."
#endif
#ifndef SHAPER_CPP_DEFAULT_URDF
#define SHAPER_CPP_DEFAULT_URDF "modelone.urdf"
#endif
#ifndef SHAPER_CPP_REFORGE_VERSION
#define SHAPER_CPP_REFORGE_VERSION "unknown"
#endif

namespace {

using standard_bots_shaper::kJointCount;
using standard_bots_shaper::kJointOrder;
using trajectory_msgs::msg::JointTrajectory;

void Require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

builtin_interfaces::msg::Duration DurationFromNanoseconds(std::int64_t ns) {
  builtin_interfaces::msg::Duration result;
  result.sec = static_cast<std::int32_t>(ns / 1'000'000'000);
  result.nanosec = static_cast<std::uint32_t>(ns % 1'000'000'000);
  return result;
}

JointTrajectory MakeTrajectory() {
  constexpr std::size_t kPointCount = standard_bots_shaper::kExpectedPointCount;
  constexpr double kMoveDurationS = 1.25;
  constexpr double kGoalRad = 0.35;
  JointTrajectory trajectory;
  trajectory.joint_names = kJointOrder;
  trajectory.points.reserve(kPointCount);
  for (std::size_t index = 0; index < kPointCount; ++index) {
    const double time_s = static_cast<double>(index) *
                          standard_bots_shaper::kSamplePeriodSeconds;
    const double phase = std::clamp(time_s / kMoveDurationS, 0.0, 1.0);
    const double blend = 10.0 * std::pow(phase, 3) - 15.0 * std::pow(phase, 4) +
                         6.0 * std::pow(phase, 5);
    const double blend_dot =
        phase >= 1.0 ? 0.0
                     : (30.0 * std::pow(phase, 2) - 60.0 * std::pow(phase, 3) +
                        30.0 * std::pow(phase, 4)) /
                           kMoveDurationS;
    const double blend_ddot =
        phase >= 1.0
            ? 0.0
            : (60.0 * phase - 180.0 * std::pow(phase, 2) +
               120.0 * std::pow(phase, 3)) /
                  (kMoveDurationS * kMoveDurationS);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(kJointCount, 0.0);
    point.velocities.assign(kJointCount, 0.0);
    point.accelerations.assign(kJointCount, 0.0);
    point.positions[0] = kGoalRad * blend;
    point.velocities[0] = kGoalRad * blend_dot;
    point.accelerations[0] = kGoalRad * blend_ddot;
    point.time_from_start = DurationFromNanoseconds(
        static_cast<std::int64_t>(index) *
        standard_bots_shaper::kSamplePeriodNanoseconds);
    trajectory.points.push_back(std::move(point));
  }
  return trajectory;
}

reforge::control::shaper::backend::BackendConfiguration Configuration(
    const std::filesystem::path& model_directory,
    const std::filesystem::path& urdf) {
  using reforge::control::runtime::SharedImpulsePolicy;
  using reforge::control::shaper::ResidualPathScope;
  using reforge::control::shaper::backend::BackendConfiguration;
  using reforge::native::Vector;
  BackendConfiguration configuration;
  configuration.sample_time_s = standard_bots_shaper::kSamplePeriodSeconds;
  configuration.model_directory = model_directory;
  configuration.urdf_filepath = urdf;
  configuration.num_axes = 3;
  configuration.num_joints = kJointCount;
  configuration.side_length_m = 0.0007;
  configuration.base_height_m = 0.364;
  configuration.probability_threshold = 0.5;
  configuration.shared_impulse_policy = SharedImpulsePolicy::kCombineAllModes;
  configuration.shared_impulse_shapes_all_joints = true;
  configuration.residual_path_scope = ResidualPathScope::kShapedAxes;
  configuration.train_base_angles_rad = Vector::Zero(1);
  configuration.feature_frame_translation_m = std::nullopt;
  return configuration;
}

void RequireParity(const JointTrajectory& ros, const reforge::native::ShapedTrajectory& native) {
  Require(ros.points.size() == static_cast<std::size_t>(native.positions.rows()),
          "native and ROS outputs have different point counts");
  for (std::size_t row = 0; row < ros.points.size(); ++row) {
    const auto& point = ros.points[row];
    const auto r = static_cast<Eigen::Index>(row);
    const auto timestamp_ns =
        static_cast<std::int64_t>(point.time_from_start.sec) * 1'000'000'000 +
        static_cast<std::int64_t>(point.time_from_start.nanosec);
    Require(std::abs(static_cast<double>(timestamp_ns) * 1.0e-9 -
                     native.time(r)) < 1.0e-9,
            "native and ROS timestamps differ");
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const auto c = static_cast<Eigen::Index>(joint);
      Require(std::abs(point.positions[joint] - native.positions(r, c)) < 1.0e-9,
              "native and ROS position outputs differ");
      Require(std::abs(point.velocities[joint] - native.velocities(r, c)) < 1.0e-9,
              "native and ROS velocity outputs differ");
      Require(std::abs(point.accelerations[joint] - native.accelerations(r, c)) < 1.0e-9,
              "native and ROS acceleration outputs differ");
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const std::filesystem::path model_directory =
        argc > 1 ? argv[1] : SHAPER_CPP_DEFAULT_MODEL_DIRECTORY;
    const std::filesystem::path urdf = argc > 2 ? argv[2] : SHAPER_CPP_DEFAULT_URDF;
    const auto input = MakeTrajectory();
    standard_bots_shaper::ValidateCompleteTrajectory(input);

    auto adapter_shaper = reforge::control::shaper::backend::NativeShaper(
        Configuration(model_directory, urdf));
    reforge::control::shaper::ros2::TrajectoryOptions options;
    options.expected_joint_names = kJointOrder;
    options.residual_shaping_strategy = std::nullopt;
    options.finalize_tail = false;
    const auto shape_start = std::chrono::steady_clock::now();
    const auto ros_shaped = reforge::control::shaper::ros2::ProcessTrajectory(
        adapter_shaper, input, options);
    const auto shape_elapsed_ms = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() -
                                      shape_start)
                                      .count();

    auto native_shaper = reforge::control::shaper::backend::NativeShaper(
        Configuration(model_directory, urdf));
    reforge::control::shaper::backend::ProcessTrajectoryRequest request;
    request.input.position_rad = reforge::native::Matrix(
        static_cast<Eigen::Index>(input.points.size()),
        static_cast<Eigen::Index>(kJointCount));
    request.input.velocity_rad_per_s = reforge::native::Matrix(
        static_cast<Eigen::Index>(input.points.size()),
        static_cast<Eigen::Index>(kJointCount));
    request.input.acceleration_rad_per_s2 = reforge::native::Matrix(
        static_cast<Eigen::Index>(input.points.size()),
        static_cast<Eigen::Index>(kJointCount));
    request.input.time_s = reforge::native::Vector(
        static_cast<Eigen::Index>(input.points.size()));
    for (std::size_t row = 0; row < input.points.size(); ++row) {
      const auto r = static_cast<Eigen::Index>(row);
      const auto& point = input.points[row];
      request.input.time_s.value()(r) =
          static_cast<double>(row) * standard_bots_shaper::kSamplePeriodSeconds;
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        const auto c = static_cast<Eigen::Index>(joint);
        request.input.position_rad(r, c) = point.positions[joint];
        request.input.velocity_rad_per_s.value()(r, c) = point.velocities[joint];
        request.input.acceleration_rad_per_s2.value()(r, c) = point.accelerations[joint];
      }
    }
    request.residual_shaping_strategy = std::nullopt;
    request.finalize_tail = false;
    const auto native_shaped = native_shaper.ProcessTrajectory(request);
    RequireParity(ros_shaped, native_shaped);

    std::size_t published = 0;
    const auto records = standard_bots_shaper::StreamOnePointTrajectory(
        ros_shaped,
        [&](const JointTrajectory& message) {
          Require(message.points.size() == 1, "stream message is not one point");
          ++published;
        },
        [index = std::int64_t{0}]() mutable {
          return 10'000'000'000 + index++ *
                 standard_bots_shaper::kSamplePeriodNanoseconds;
        },
        [](std::chrono::nanoseconds) {});
    Require(published == standard_bots_shaper::kExpectedPointCount,
            "stream did not publish every trajectory point");
    Require(records.back().point_index == published - 1,
            "stream point order is not complete");
    std::cout << "ReforgeShaper package: " << SHAPER_CPP_REFORGE_VERSION << '\n'
              << "Model: Standard Bots Model One "
                 "(shaper_models.native.json)\n"
              << "Shaping time: " << shape_elapsed_ms << " ms\n"
              << "Validation: pass; native/ROS parity: pass\n"
              << "Transport: " << published
              << " ordered points at 0.005 s.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Standard Bots Shaper example failed: " << error.what() << '\n';
    return 1;
  }
}
