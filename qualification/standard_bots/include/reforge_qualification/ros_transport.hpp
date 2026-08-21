#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "reforge_qualification/harness.hpp"

namespace reforge::qualification {

struct RosTopics final {
    std::string command;
    std::string joint_state;
    std::string imu;
};

class RosQualificationTransport final :
    public rclcpp::Node,
    public QualificationTransport {
public:
    RosQualificationTransport(
        RosTopics topics,
        double feedback_max_age_s,
        std::function<bool()> control_state_probe,
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    [[nodiscard]] PreflightSnapshot Snapshot() override;
    [[nodiscard]] Capture PublishAndRecord(
        const trajectory_msgs::msg::JointTrajectory& message,
        double recording_duration_s) override;
    [[nodiscard]] Capture RecordFeedback(double recording_duration_s) override;

private:
    [[nodiscard]] double SteadyNowS() const;
    void OnJointState(const sensor_msgs::msg::JointState& message);
    void OnImu(const sensor_msgs::msg::Imu& message);

    double feedback_max_age_s_;
    std::function<bool()> control_state_probe_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    mutable std::mutex mutex_;
    sensor_msgs::msg::JointState latest_joint_state_;
    bool have_joint_state_ = false;
    bool have_imu_ = false;
    double latest_joint_receive_s_ = 0.0;
    double latest_imu_receive_s_ = 0.0;
    bool recording_ = false;
    double recording_epoch_s_ = 0.0;
    Capture capture_;
};

}  // namespace reforge::qualification
