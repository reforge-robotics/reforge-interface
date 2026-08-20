#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include <sys/wait.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include "reforge_qualification/harness.hpp"
#include "reforge_qualification/ros_transport.hpp"

namespace {

using reforge::qualification::TrialKind;

struct Options final {
    bool execute = false;
    bool preflight = false;
    std::string trial = "both";
    std::filesystem::path assets_directory = "src/robot";
    std::filesystem::path output_directory = "qualification-output";
    std::string endpoint;
    std::string robot_id;
    std::string command_topic;
    std::string joint_state_topic;
    std::string imu_topic;
    std::filesystem::path control_state_probe;
};

[[nodiscard]] Options ParseOptions(int count, char** values) {
    Options options;
    for (int index = 1; index < count; ++index) {
        const std::string argument(values[index]);
        const auto next = [&]() -> std::string {
            if (++index >= count) {
                throw std::invalid_argument("missing value for " + argument);
            }
            return values[index];
        };
        if (argument == "--execute") {
            options.execute = true;
        } else if (argument == "--preflight") {
            options.preflight = true;
        } else if (argument == "--trial") {
            options.trial = next();
        } else if (argument == "--assets-dir") {
            options.assets_directory = next();
        } else if (argument == "--output-dir") {
            options.output_directory = next();
        } else if (argument == "--endpoint") {
            options.endpoint = next();
        } else if (argument == "--robot-id") {
            options.robot_id = next();
        } else if (argument == "--command-topic") {
            options.command_topic = next();
        } else if (argument == "--joint-state-topic") {
            options.joint_state_topic = next();
        } else if (argument == "--imu-topic") {
            options.imu_topic = next();
        } else if (argument == "--control-state-probe") {
            options.control_state_probe = next();
        } else if (argument == "--help" || argument == "-h") {
            std::cout
                << "Usage: standard_bots_qualification_cli [options]\n"
                   "Dry-run is the default and performs no ROS or network I/O.\n\n"
                   "  --trial unshaped|shaped|both   Default: both (dry-run only)\n"
                   "  --assets-dir PATH             Directory containing models/ and urdf/\n"
                   "  --output-dir PATH             Evidence output directory\n"
                   "  --execute                     Explicitly arm one real trial\n\n"
                   "  --preflight                   Read-only ROS readiness check\n\n"
                   "Execution also requires --endpoint, --robot-id, all three topic\n"
                   "options, a Standard Bots SDK control-state probe, and an exact\n"
                   "terminal confirmation. Safety limits are frozen in Phase A.\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown option: " + argument);
        }
    }
    if (options.trial != "unshaped" && options.trial != "shaped" &&
        options.trial != "both") {
        throw std::invalid_argument("--trial must be unshaped, shaped, or both");
    }
    if (options.execute && options.preflight) {
        throw std::invalid_argument("--execute and --preflight are mutually exclusive");
    }
    return options;
}

void RequireWritableOutput(const std::filesystem::path& directory) {
    std::filesystem::create_directories(directory);
    const auto probe = directory / ".write-probe";
    {
        std::ofstream output(probe);
        output << "qualification write probe\n";
        if (!output) {
            throw std::runtime_error("qualification output directory is not writable");
        }
    }
    std::filesystem::remove(probe);
}

void RequireRosOptions(const Options& options) {
    if (options.endpoint.empty() || options.robot_id.empty() ||
        options.command_topic.empty() || options.joint_state_topic.empty() ||
        options.imu_topic.empty() || options.control_state_probe.empty()) {
        throw std::invalid_argument(
            "ROS operation requires endpoint, robot ID, three topics, and SDK probe");
    }
}

[[nodiscard]] bool RunControlStateProbe(
    const std::filesystem::path& probe,
    const std::string& endpoint,
    const std::string& robot_id) {
    const auto executable = std::filesystem::canonical(probe);
    if (!std::filesystem::is_regular_file(executable)) {
        throw std::invalid_argument("control-state probe must be a regular file");
    }
    if (::setenv("REFORGE_QUALIFICATION_ENDPOINT", endpoint.c_str(), 1) != 0 ||
        ::setenv("REFORGE_QUALIFICATION_ROBOT_ID", robot_id.c_str(), 1) != 0) {
        throw std::runtime_error("could not prepare control-state probe environment");
    }
    const pid_t child = ::fork();
    if (child < 0) {
        throw std::runtime_error("could not start control-state probe");
    }
    if (child == 0) {
        ::execl(executable.c_str(), executable.c_str(), nullptr);
        ::_exit(127);
    }
    int status = 0;
    if (::waitpid(child, &status, 0) != child) {
        throw std::runtime_error("could not collect control-state probe result");
    }
    return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

void RequireExecutionOptions(const Options& options) {
    RequireRosOptions(options);
    if (options.trial == "both") {
        throw std::invalid_argument("--execute requires exactly one --trial");
    }
}

void ConfirmExecution(const Options& options) {
    const std::string expected =
        "EXECUTE " + options.trial + " " + options.robot_id;
    std::cout
        << "\nREAL ROBOT EXECUTION ARMED\n"
        << "Endpoint: " << options.endpoint << "\n"
        << "Robot ID: " << options.robot_id << "\n"
        << "Command topic: " << options.command_topic << "\n"
        << "Confirm Nosa is observing the robot, the swept workspace is clear,\n"
           "the E-stop is reachable, and the displayed identity is correct.\n"
        << "Type exactly: " << expected << "\n> " << std::flush;
    std::string confirmation;
    std::getline(std::cin, confirmation);
    if (confirmation != expected) {
        throw std::runtime_error("terminal confirmation did not match; nothing published");
    }
}

[[nodiscard]] TrialKind Kind(const std::string& trial) {
    return trial == "shaped" ? TrialKind::kShaped : TrialKind::kUnshaped;
}

}  // namespace

int main(int count, char** values) {
    bool ros_initialized = false;
    try {
        const Options options = ParseOptions(count, values);
        const auto definition = reforge::qualification::FrozenPhaseATrial();
        auto limits = reforge::qualification::ConservativeSafetyLimits();
        RequireWritableOutput(options.output_directory);

        const auto unshaped =
            reforge::qualification::GenerateUnshapedTrajectory(definition);
        reforge::qualification::ValidateTrajectory(unshaped, definition, limits);
        const auto shaped = reforge::qualification::ShapeTrajectoryWithParity(
            unshaped, options.assets_directory);
        reforge::qualification::ValidateTrajectory(shaped, definition, limits);

        if (!options.execute && !options.preflight) {
            if (options.trial == "unshaped" || options.trial == "both") {
                reforge::qualification::WriteTrajectoryCsv(
                    options.output_directory / "unshaped_trajectory.csv", unshaped);
                reforge::qualification::WriteResultManifest(
                    options.output_directory / "unshaped_manifest.json",
                    TrialKind::kUnshaped, false, unshaped, std::nullopt, "", "");
            }
            if (options.trial == "shaped" || options.trial == "both") {
                reforge::qualification::WriteTrajectoryCsv(
                    options.output_directory / "shaped_trajectory.csv", shaped);
                reforge::qualification::WriteResultManifest(
                    options.output_directory / "shaped_manifest.json",
                    TrialKind::kShaped, false, shaped, std::nullopt, "", "");
            }
            std::cout << "DRY-RUN PASS: generated and validated complete "
                      << options.trial
                      << " trajectory evidence; ROS was not initialized and no "
                         "robot or network I/O was performed.\n";
            return 0;
        }

        if (options.preflight) {
            RequireRosOptions(options);
            rclcpp::init(count, values);
            ros_initialized = true;
            auto transport = std::make_shared<
                reforge::qualification::RosQualificationTransport>(
                reforge::qualification::RosTopics{
                    options.command_topic, options.joint_state_topic,
                    options.imu_topic},
                limits.feedback_max_age_s,
                [&options]() {
                    return RunControlStateProbe(
                        options.control_state_probe,
                        options.endpoint,
                        options.robot_id);
                });
            const auto snapshot = transport->Snapshot();
            std::cout << "READ-ONLY PREFLIGHT (no command published)\n"
                      << "Endpoint: " << options.endpoint << "\n"
                      << "Robot ID: " << options.robot_id << "\n"
                      << "Command subscriber: "
                      << (snapshot.command_subscriber_present ? "present" : "missing")
                      << "\nControl state: "
                      << (snapshot.control_active ? "active" : "not active")
                      << "\nJoint state: "
                      << (snapshot.joint_state_fresh ? "fresh" : "stale/missing")
                      << "\nIMU: "
                      << (snapshot.imu_fresh ? "fresh" : "stale/missing") << '\n';
            rclcpp::shutdown();
            ros_initialized = false;
            return snapshot.command_subscriber_present && snapshot.control_active &&
                           snapshot.joint_state_fresh && snapshot.imu_fresh
                       ? 0
                       : 1;
        }

        RequireExecutionOptions(options);
        ConfirmExecution(options);
        const auto& command = options.trial == "shaped" ? shaped : unshaped;
        rclcpp::init(count, values);
        ros_initialized = true;
        auto transport = std::make_shared<
            reforge::qualification::RosQualificationTransport>(
            reforge::qualification::RosTopics{
                options.command_topic, options.joint_state_topic,
                options.imu_topic},
            limits.feedback_max_age_s,
            [&options]() {
                return RunControlStateProbe(
                    options.control_state_probe,
                    options.endpoint,
                    options.robot_id);
            });
        const auto capture = reforge::qualification::ExecutePreparedTrial(
            *transport, command, definition, limits);
        const auto analysis =
            reforge::qualification::AnalyzeCapture(command, capture);
        reforge::qualification::WriteTrajectoryCsv(
            options.output_directory / (options.trial + "_trajectory.csv"), command);
        reforge::qualification::WriteCaptureCsv(options.output_directory, capture);
        reforge::qualification::WriteResultManifest(
            options.output_directory / (options.trial + "_manifest.json"),
            Kind(options.trial), true, command, analysis,
            options.endpoint, options.robot_id);
        rclcpp::shutdown();
        ros_initialized = false;
        std::cout << "EXECUTION PASS: exactly one complete " << options.trial
                  << " trajectory was published and recorded.\n";
        return 0;
    } catch (const std::exception& error) {
        if (ros_initialized && rclcpp::ok()) {
            rclcpp::shutdown();
        }
        std::cerr << "Qualification failed: " << error.what() << '\n';
        return 1;
    }
}
