#include "reforge_qualification/ros_transport.hpp"

#include <algorithm>
#include <thread>
#include <utility>

namespace reforge::qualification {

RosQualificationTransport::RosQualificationTransport(
    RosTopics topics,
    double feedback_max_age_s,
    std::function<bool()> control_state_probe,
    const rclcpp::NodeOptions& options)
    : Node("reforge_standard_bots_hardware_qualification", options),
      feedback_max_age_s_(feedback_max_age_s),
      control_state_probe_(std::move(control_state_probe)) {
    if (topics.command.empty() || topics.joint_state.empty() ||
        topics.imu.empty() || !control_state_probe_) {
        throw std::invalid_argument("all ROS qualification topics must be explicit");
    }
    publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
        topics.command, rclcpp::QoS(10).reliable());
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        topics.joint_state, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState& message) {
            OnJointState(message);
        });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        topics.imu, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Imu& message) { OnImu(message); });
}

double RosQualificationTransport::SteadyNowS() const {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void RosQualificationTransport::OnJointState(
    const sensor_msgs::msg::JointState& message) {
    const double receive_s = SteadyNowS();
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joint_state_ = message;
    have_joint_state_ = true;
    latest_joint_receive_s_ = receive_s;
    if (recording_) {
        capture_.joint_states.push_back(
            JointStateRecord{receive_s - recording_epoch_s_, message});
    }
}

void RosQualificationTransport::OnImu(const sensor_msgs::msg::Imu& message) {
    const double receive_s = SteadyNowS();
    std::lock_guard<std::mutex> lock(mutex_);
    have_imu_ = true;
    latest_imu_receive_s_ = receive_s;
    if (recording_) {
        capture_.imu.push_back(ImuRecord{receive_s - recording_epoch_s_, message});
    }
}

PreflightSnapshot RosQualificationTransport::Snapshot() {
    const double deadline_s = SteadyNowS() + 2.0;
    while (rclcpp::ok() && SteadyNowS() < deadline_s) {
        rclcpp::spin_some(shared_from_this());
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (have_joint_state_ && have_imu_ &&
                publisher_->get_subscription_count() > 0) {
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const double now_s = SteadyNowS();
    const bool control_active = control_state_probe_();
    std::lock_guard<std::mutex> lock(mutex_);
    return PreflightSnapshot{
        publisher_->get_subscription_count() > 0,
        control_active,
        have_joint_state_ && now_s - latest_joint_receive_s_ <= feedback_max_age_s_,
        have_imu_ && now_s - latest_imu_receive_s_ <= feedback_max_age_s_,
        latest_joint_state_.name,
        latest_joint_state_.position};
}

Capture RosQualificationTransport::PublishAndRecord(
    const trajectory_msgs::msg::JointTrajectory& message,
    double recording_duration_s) {
    if (!(recording_duration_s > 0.0)) {
        throw std::invalid_argument("recording duration must be positive");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        capture_ = Capture{};
        recording_epoch_s_ = SteadyNowS();
        recording_ = true;
    }

    // This is the only command publication in the qualification path.
    publisher_->publish(message);
    const double deadline_s = SteadyNowS() + recording_duration_s;
    bool command_subscriber_lost = false;
    while (rclcpp::ok() && SteadyNowS() < deadline_s) {
        rclcpp::spin_some(shared_from_this());
        if (publisher_->get_subscription_count() == 0) {
            command_subscriber_lost = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const double now_s = SteadyNowS();
    std::lock_guard<std::mutex> lock(mutex_);
    recording_ = false;
    capture_.recording_duration_s = now_s - recording_epoch_s_;
    capture_.command_subscriber_lost = command_subscriber_lost;
    capture_.joint_state_age_at_end_s = have_joint_state_
                                            ? now_s - latest_joint_receive_s_
                                            : capture_.recording_duration_s;
    capture_.imu_age_at_end_s = have_imu_
                                    ? now_s - latest_imu_receive_s_
                                    : capture_.recording_duration_s;
    return capture_;
}

Capture RosQualificationTransport::RecordFeedback(double recording_duration_s) {
    if (!(recording_duration_s > 0.0)) {
        throw std::invalid_argument("recording duration must be positive");
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        capture_ = Capture{};
        recording_epoch_s_ = SteadyNowS();
        recording_ = true;
    }

    // Read-only telemetry qualification: this path never publishes a command.
    const double deadline_s = SteadyNowS() + recording_duration_s;
    while (rclcpp::ok() && SteadyNowS() < deadline_s) {
        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const double now_s = SteadyNowS();
    std::lock_guard<std::mutex> lock(mutex_);
    recording_ = false;
    capture_.recording_duration_s = now_s - recording_epoch_s_;
    capture_.joint_state_age_at_end_s = have_joint_state_
                                            ? now_s - latest_joint_receive_s_
                                            : capture_.recording_duration_s;
    capture_.imu_age_at_end_s = have_imu_
                                    ? now_s - latest_imu_receive_s_
                                    : capture_.recording_duration_s;
    return capture_;
}

}  // namespace reforge::qualification
