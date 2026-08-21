#include <cmath>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "reforge_qualification/harness.hpp"
#include "reforge_qualification/ros_transport.hpp"

namespace {

using reforge::qualification::Analysis;
using reforge::qualification::Capture;
using reforge::qualification::FrozenPhaseATrial;
using reforge::qualification::ImuRecord;
using reforge::qualification::JointStateRecord;
using reforge::qualification::PreflightSnapshot;
using reforge::qualification::QualificationTransport;
using reforge::qualification::SafetyLimits;
using reforge::qualification::kJointCount;
using reforge::qualification::kJointOrder;

[[nodiscard]] double TimeS(
    const trajectory_msgs::msg::JointTrajectoryPoint& point) {
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

[[nodiscard]] PreflightSnapshot ReadySnapshot(
    const trajectory_msgs::msg::JointTrajectory& command) {
    return PreflightSnapshot{
        true,
        true,
        true,
        true,
        std::vector<std::string>(kJointOrder.begin(), kJointOrder.end()),
        command.points.front().positions};
}

[[nodiscard]] Capture DeterministicCapture(
    const trajectory_msgs::msg::JointTrajectory& command) {
    Capture capture;
    for (std::size_t sample = 0; sample < command.points.size(); sample += 5) {
        sensor_msgs::msg::JointState state;
        state.name.assign(kJointOrder.begin(), kJointOrder.end());
        state.position = command.points[sample].positions;
        capture.joint_states.push_back(
            JointStateRecord{TimeS(command.points[sample]), std::move(state)});
        sensor_msgs::msg::Imu imu;
        imu.linear_acceleration.z = 9.81;
        capture.imu.push_back(
            ImuRecord{TimeS(command.points[sample]), std::move(imu)});
    }
    return capture;
}

class FakeTransport final : public QualificationTransport {
public:
    PreflightSnapshot snapshot;
    Capture capture;
    std::size_t publication_count = 0;
    trajectory_msgs::msg::JointTrajectory published_message;

    PreflightSnapshot Snapshot() override { return snapshot; }

    Capture PublishAndRecord(
        const trajectory_msgs::msg::JointTrajectory& message,
        double recording_duration_s) override {
        EXPECT_GT(recording_duration_s, 5.0);
        ++publication_count;
        published_message = message;
        return capture;
    }

    Capture RecordFeedback(double recording_duration_s) override {
        EXPECT_GT(recording_duration_s, 0.0);
        return capture;
    }
};

[[nodiscard]] SafetyLimits ExecutionLimits() {
    auto limits = reforge::qualification::ConservativeSafetyLimits();
    limits.current_pose_tolerance_rad = 0.01;
    limits.following_error_limit_rad = 0.05;
    return limits;
}

TEST(TrajectoryGeneration, ReproducesFrozenPhaseAPreview) {
    const auto definition = FrozenPhaseATrial();
    const auto message =
        reforge::qualification::GenerateUnshapedTrajectory(definition);
    EXPECT_EQ(message.points.size(), 1053U);
    EXPECT_NEAR(TimeS(message.points.back()), 5.26, 1.0e-12);
    double peak_velocity = 0.0;
    double peak_acceleration = 0.0;
    for (const auto& point : message.points) {
        for (std::size_t joint = 0; joint < kJointCount; ++joint) {
            peak_velocity = std::max(peak_velocity, std::abs(point.velocities[joint]));
            peak_acceleration =
                std::max(peak_acceleration, std::abs(point.accelerations[joint]));
        }
    }
    EXPECT_NEAR(peak_velocity, 1.2417166730458506, 1.0e-12);
    EXPECT_NEAR(peak_acceleration, 1.9788313514712552, 1.0e-10);
    EXPECT_EQ(message.points.front().positions,
              std::vector<double>(definition.start_rad.begin(), definition.start_rad.end()));
    for (const auto& point : message.points) {
        for (std::size_t joint = 1; joint < kJointCount; ++joint) {
            EXPECT_DOUBLE_EQ(point.positions[joint], definition.start_rad[joint]);
        }
    }
}

TEST(Adapter, RosOutputMatchesNativeOutputAtFiveMilliseconds) {
    const auto input =
        reforge::qualification::GenerateUnshapedTrajectory(FrozenPhaseATrial());
    const auto output = reforge::qualification::ShapeTrajectoryWithParity(
        input, std::filesystem::path(REFORGE_INTERFACE_ROOT) / "src/robot");
    EXPECT_EQ(output.joint_names, input.joint_names);
    EXPECT_GE(output.points.size(), input.points.size());
}

TEST(Execution, PublishesExactlyOneCompleteMessageAndAlignsRecorderData) {
    const auto definition = FrozenPhaseATrial();
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(definition);
    FakeTransport transport;
    transport.snapshot = ReadySnapshot(command);
    transport.capture = DeterministicCapture(command);
    const auto capture = reforge::qualification::ExecutePreparedTrial(
        transport, command, definition, ExecutionLimits());
    EXPECT_EQ(transport.publication_count, 1U);
    EXPECT_EQ(transport.published_message.points.size(), command.points.size());
    const Analysis analysis =
        reforge::qualification::AnalyzeCapture(command, capture);
    EXPECT_EQ(analysis.aligned_samples, capture.joint_states.size());
    EXPECT_NEAR(analysis.maximum_following_error_rad, 0.0, 1.0e-15);
    EXPECT_NEAR(analysis.peak_linear_acceleration_m_s2, 9.81, 1.0e-12);
}

TEST(Safety, RejectsEveryPrepublicationReadinessFailure) {
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(FrozenPhaseATrial());
    const auto limits = ExecutionLimits();
    auto snapshot = ReadySnapshot(command);

    snapshot.command_subscriber_present = false;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(snapshot, command, limits),
        std::runtime_error);
    snapshot = ReadySnapshot(command);
    snapshot.control_active = false;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(snapshot, command, limits),
        std::runtime_error);
    snapshot = ReadySnapshot(command);
    snapshot.joint_state_fresh = false;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(snapshot, command, limits),
        std::runtime_error);
    snapshot = ReadySnapshot(command);
    snapshot.imu_fresh = false;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(snapshot, command, limits),
        std::runtime_error);
    snapshot = ReadySnapshot(command);
    std::swap(snapshot.joint_names[0], snapshot.joint_names[1]);
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(snapshot, command, limits),
        std::runtime_error);
    snapshot = ReadySnapshot(command);
    snapshot.current_position_rad[0] += 0.1;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(snapshot, command, limits),
        std::runtime_error);
}

TEST(Safety, RejectsMissingExplicitExecutionThresholds) {
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(FrozenPhaseATrial());
    auto limits = ExecutionLimits();
    limits.current_pose_tolerance_rad = 0.0;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(
            ReadySnapshot(command), command, limits),
        std::invalid_argument);
    limits = ExecutionLimits();
    limits.following_error_limit_rad = 0.0;
    EXPECT_THROW(
        reforge::qualification::ValidatePreflight(
            ReadySnapshot(command), command, limits),
        std::invalid_argument);
}

TEST(Safety, RejectsMalformedAndOutOfBoundsCommandsBeforeTransport) {
    const auto definition = FrozenPhaseATrial();
    const auto valid =
        reforge::qualification::GenerateUnshapedTrajectory(definition);
    const auto limits = ExecutionLimits();

    auto invalid = valid;
    invalid.joint_names[0] = "wrong_joint";
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points.pop_back();
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points.front().positions[0] += 0.01;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].positions[1] += 0.01;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].positions.pop_back();
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].effort.assign(kJointCount, 0.0);
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].positions[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].time_from_start = invalid.points[0].time_from_start;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].velocities[0] = limits.max_velocity_rad_s + 1.0;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].accelerations[0] = limits.max_acceleration_rad_s2 + 1.0;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points[1].positions[0] = limits.maximum_position_rad[0] + 1.0;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
    invalid = valid;
    invalid.points.back().time_from_start.nanosec -= 1;
    EXPECT_THROW(
        reforge::qualification::ValidateTrajectory(invalid, definition, limits),
        std::invalid_argument);
}

TEST(Safety, RejectsFollowingErrorAfterRecording) {
    const auto definition = FrozenPhaseATrial();
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(definition);
    FakeTransport transport;
    transport.snapshot = ReadySnapshot(command);
    transport.capture = DeterministicCapture(command);
    transport.capture.joint_states.front().message.position[0] += 0.1;
    EXPECT_THROW(
        reforge::qualification::ExecutePreparedTrial(
            transport, command, definition, ExecutionLimits()),
        std::runtime_error);
    EXPECT_EQ(transport.publication_count, 1U);
}

TEST(Recorder, RejectsMissingOrMisorderedStreams) {
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(FrozenPhaseATrial());
    Capture capture = DeterministicCapture(command);
    capture.imu.clear();
    EXPECT_THROW(
        static_cast<void>(
            reforge::qualification::AnalyzeCapture(command, capture)),
        std::runtime_error);
    capture = DeterministicCapture(command);
    std::swap(
        capture.joint_states.front().message.name[0],
        capture.joint_states.front().message.name[1]);
    EXPECT_THROW(
        static_cast<void>(
            reforge::qualification::AnalyzeCapture(command, capture)),
        std::runtime_error);
    capture = DeterministicCapture(command);
    capture.joint_states.front().message.position[0] =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(
        static_cast<void>(
            reforge::qualification::AnalyzeCapture(command, capture)),
        std::runtime_error);
    capture = DeterministicCapture(command);
    capture.imu[1].elapsed_s = capture.imu[0].elapsed_s;
    EXPECT_THROW(
        static_cast<void>(
            reforge::qualification::AnalyzeCapture(command, capture)),
        std::runtime_error);
}

TEST(Recorder, ReportsFailingStreamAndArrivalAndSourceGaps) {
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(FrozenPhaseATrial());
    Capture capture = DeterministicCapture(command);
    for (std::size_t index = 0; index < capture.imu.size(); ++index) {
        const std::uint64_t source_nanoseconds =
            100'000'000'000ULL + index * 25'000'000ULL;
        capture.imu[index].message.header.stamp.sec =
            static_cast<std::int32_t>(source_nanoseconds / 1'000'000'000ULL);
        capture.imu[index].message.header.stamp.nanosec =
            static_cast<std::uint32_t>(source_nanoseconds % 1'000'000'000ULL);
    }
    capture.imu[2].elapsed_s = capture.imu[1].elapsed_s + 0.105;
    for (std::size_t index = 3; index < capture.imu.size(); ++index) {
        capture.imu[index].elapsed_s = capture.imu[index - 1].elapsed_s + 0.025;
    }
    try {
        static_cast<void>(
            reforge::qualification::AnalyzeCapture(command, capture));
        FAIL() << "expected an IMU arrival-gap failure";
    } catch (const std::runtime_error& error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("stream 'imu'"), std::string::npos);
        EXPECT_NE(message.find("0.105"), std::string::npos);
        EXPECT_NE(message.find("source-header gap 0.025"), std::string::npos);
    }
}

TEST(Recorder, PersistsCaptureBeforeGapValidationFailure) {
    const auto definition = FrozenPhaseATrial();
    const auto command =
        reforge::qualification::GenerateUnshapedTrajectory(definition);
    FakeTransport transport;
    transport.snapshot = ReadySnapshot(command);
    transport.capture = DeterministicCapture(command);
    transport.capture.imu[2].elapsed_s =
        transport.capture.imu[1].elapsed_s + 0.105;
    bool capture_persisted = false;
    EXPECT_THROW(
        static_cast<void>(reforge::qualification::ExecutePreparedTrial(
            transport, command, definition, ExecutionLimits(),
            [&capture_persisted](const Capture& capture) {
                capture_persisted = !capture.imu.empty();
            })),
        std::runtime_error);
    EXPECT_TRUE(capture_persisted);
}

TEST(RosIntegration, PublishesOneCompleteMessageWithDeterministicStreams) {
    int argument_count = 1;
    char program_name[] = "standard_bots_qualification_tests";
    char* arguments[] = {program_name, nullptr};
    rclcpp::init(argument_count, arguments);
    {
        const reforge::qualification::RosTopics topics{
            "/qualification_test/command",
            "/qualification_test/joint_state",
            "/qualification_test/imu"};
        auto transport = std::make_shared<
            reforge::qualification::RosQualificationTransport>(
                topics, 0.25, []() { return true; });
        auto mock = std::make_shared<rclcpp::Node>("standard_bots_mock_robot");

        std::atomic<std::size_t> publication_count{0};
        std::atomic<std::size_t> published_point_count{0};
        const auto command_subscription =
            mock->create_subscription<trajectory_msgs::msg::JointTrajectory>(
                topics.command, rclcpp::QoS(1).reliable(),
                [&](const trajectory_msgs::msg::JointTrajectory& message) {
                    ++publication_count;
                    published_point_count = message.points.size();
                });
        const auto joint_publisher =
            mock->create_publisher<sensor_msgs::msg::JointState>(
                topics.joint_state, rclcpp::SensorDataQoS());
        const auto imu_publisher = mock->create_publisher<sensor_msgs::msg::Imu>(
            topics.imu, rclcpp::QoS(10).reliable());
        const auto timer = mock->create_wall_timer(
            std::chrono::milliseconds(2),
            [joint_publisher, imu_publisher]() {
                sensor_msgs::msg::JointState state;
                state.name.assign(kJointOrder.begin(), kJointOrder.end());
                state.position.assign(kJointCount, 0.0);
                joint_publisher->publish(state);
                sensor_msgs::msg::Imu imu;
                imu.linear_acceleration.z = 9.81;
                imu_publisher->publish(imu);
            });

        rclcpp::executors::SingleThreadedExecutor mock_executor;
        mock_executor.add_node(mock);
        std::thread mock_thread([&mock_executor]() { mock_executor.spin(); });
        const auto snapshot = transport->Snapshot();
        EXPECT_TRUE(snapshot.command_subscriber_present);
        EXPECT_TRUE(snapshot.control_active);
        EXPECT_TRUE(snapshot.joint_state_fresh);
        EXPECT_TRUE(snapshot.imu_fresh);

        auto command = reforge::qualification::GenerateUnshapedTrajectory(
            FrozenPhaseATrial());
        command.points.resize(2);
        command.points[1].time_from_start.sec = 0;
        command.points[1].time_from_start.nanosec = 5'000'000;
        const auto capture = transport->PublishAndRecord(command, 0.05);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        EXPECT_EQ(publication_count.load(), 1U);
        EXPECT_EQ(published_point_count.load(), command.points.size());
        EXPECT_FALSE(capture.joint_states.empty());
        EXPECT_FALSE(capture.imu.empty());

        mock_executor.cancel();
        mock_thread.join();
        mock_executor.remove_node(mock);
        static_cast<void>(command_subscription);
        static_cast<void>(timer);
    }
    rclcpp::shutdown();
}

}  // namespace
