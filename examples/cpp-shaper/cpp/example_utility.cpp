#include "example_utility.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

#include <unsupported/Eigen/MatrixFunctions>

namespace reforge::examples::shaper {
namespace {

using reforge::native::Matrix;
using reforge::native::Vector;

/** Differentiate a sample-major matrix with second-order edge estimates.
 *
 * Args:
 *     samples: Sample-major values.
 *     sample_time_s: Uniform sample period [s].
 *
 * Returns:
 *     First derivative on the input timing grid.
 */
[[nodiscard]] Matrix Gradient(
    const Matrix& samples,
    double sample_time_s) {
    if (samples.rows() < 2) {
        throw std::invalid_argument(
            "at least two samples are required to estimate derivatives");
    }
    Matrix derivative(samples.rows(), samples.cols());
    if (samples.rows() == 2) {
        const auto slope =
            (samples.row(1) - samples.row(0)) / sample_time_s;
        derivative.row(0) = slope;
        derivative.row(1) = slope;
        return derivative;
    }

    derivative.row(0) =
        (-3.0 * samples.row(0) + 4.0 * samples.row(1) - samples.row(2)) /
        (2.0 * sample_time_s);
    for (Eigen::Index row = 1; row + 1 < samples.rows(); ++row) {
        derivative.row(row) =
            (samples.row(row + 1) - samples.row(row - 1)) /
            (2.0 * sample_time_s);
    }
    const Eigen::Index last = samples.rows() - 1;
    derivative.row(last) =
        (3.0 * samples.row(last) - 4.0 * samples.row(last - 1) +
         samples.row(last - 2)) /
        (2.0 * sample_time_s);
    return derivative;
}

/** Return whether every element of an Eigen expression is finite.
 *
 * Args:
 *     values: Numeric Eigen expression.
 *
 * Returns:
 *     True only when every element is finite.
 */
template <typename Derived>
[[nodiscard]] bool AllFinite(const Eigen::MatrixBase<Derived>& values) {
    return values.array().isFinite().all();
}

}  // namespace

Trajectory GeneratePointToPointTrajectory(
    const Vector& start_position_rad,
    const Vector& goal_position_rad,
    double sample_time_s,
    double move_duration_s,
    double dwell_duration_s) {
    if (start_position_rad.size() != goal_position_rad.size()) {
        throw std::invalid_argument(
            "start and goal position vectors must have equal sizes");
    }
    if (!(sample_time_s > 0.0) || !(move_duration_s > 0.0) ||
        dwell_duration_s < 0.0) {
        throw std::invalid_argument(
            "sample and move durations must be positive and dwell nonnegative");
    }

    const auto move_samples = static_cast<std::size_t>(
                                  std::llround(move_duration_s / sample_time_s)) +
                              1;
    const auto dwell_samples = static_cast<std::size_t>(
        std::llround(dwell_duration_s / sample_time_s));
    const auto total_samples = move_samples + dwell_samples;
    Matrix positions_rad(
        static_cast<Eigen::Index>(total_samples), start_position_rad.size());
    const Vector move_delta_rad = goal_position_rad - start_position_rad;

    for (std::size_t sample = 0; sample < move_samples; ++sample) {
        const double phase = static_cast<double>(sample) /
                             static_cast<double>(move_samples - 1);
        const double blend = 10.0 * std::pow(phase, 3) -
                             15.0 * std::pow(phase, 4) +
                             6.0 * std::pow(phase, 5);
        positions_rad.row(static_cast<Eigen::Index>(sample)) =
            (start_position_rad + blend * move_delta_rad).transpose();
    }
    for (std::size_t sample = move_samples; sample < total_samples; ++sample) {
        positions_rad.row(static_cast<Eigen::Index>(sample)) =
            goal_position_rad.transpose();
    }

    Vector time_s(static_cast<Eigen::Index>(total_samples));
    for (std::size_t sample = 0; sample < total_samples; ++sample) {
        time_s(static_cast<Eigen::Index>(sample)) =
            static_cast<double>(sample) * sample_time_s;
    }
    auto derivatives = EstimateDerivatives(positions_rad, sample_time_s);
    return Trajectory{
        std::move(time_s),
        std::move(positions_rad),
        std::move(derivatives.first),
        std::move(derivatives.second),
    };
}

Trajectory GeneratePointToPointSample(
    const Vector& start_position_rad,
    const Vector& goal_position_rad,
    double current_time_s,
    double move_duration_s) {
    if (start_position_rad.size() != goal_position_rad.size()) {
        throw std::invalid_argument(
            "start and goal position vectors must have equal sizes");
    }
    if (!(move_duration_s > 0.0)) {
        throw std::invalid_argument("move duration must be positive");
    }

    const double phase = std::clamp(
        current_time_s / move_duration_s, 0.0, 1.0);
    const double blend = 10.0 * std::pow(phase, 3) -
                         15.0 * std::pow(phase, 4) +
                         6.0 * std::pow(phase, 5);
    double blend_dot = 30.0 * std::pow(phase, 2) -
                       60.0 * std::pow(phase, 3) +
                       30.0 * std::pow(phase, 4);
    double blend_ddot = 60.0 * phase - 180.0 * std::pow(phase, 2) +
                        120.0 * std::pow(phase, 3);
    if (phase == 0.0 || phase == 1.0) {
        blend_dot = 0.0;
        blend_ddot = 0.0;
    }

    const Vector move_delta_rad = goal_position_rad - start_position_rad;
    Trajectory result;
    result.time_s = Vector::Constant(1, current_time_s);
    result.positions_rad.resize(1, start_position_rad.size());
    result.velocities_rad_s.resize(1, start_position_rad.size());
    result.accelerations_rad_s2.resize(1, start_position_rad.size());
    result.positions_rad.row(0) =
        (start_position_rad + blend * move_delta_rad).transpose();
    result.velocities_rad_s.row(0) =
        ((blend_dot / move_duration_s) * move_delta_rad).transpose();
    result.accelerations_rad_s2.row(0) =
        ((blend_ddot / (move_duration_s * move_duration_s)) * move_delta_rad)
            .transpose();
    return result;
}

std::pair<Matrix, Matrix> EstimateDerivatives(
    const Matrix& positions_rad,
    double sample_time_s) {
    if (!(sample_time_s > 0.0)) {
        throw std::invalid_argument("sample time must be positive");
    }
    Matrix velocities_rad_s = Gradient(positions_rad, sample_time_s);
    Matrix accelerations_rad_s2 = Gradient(velocities_rad_s, sample_time_s);
    return {std::move(velocities_rad_s), std::move(accelerations_rad_s2)};
}

void ValidateTrajectory(
    const Trajectory& trajectory,
    std::size_t expected_joint_count,
    double sample_time_s,
    const std::string& label) {
    const Eigen::Index sample_count = trajectory.positions_rad.rows();
    const Eigen::Index joint_count =
        static_cast<Eigen::Index>(expected_joint_count);
    if (sample_count < 2 || trajectory.positions_rad.cols() != joint_count ||
        trajectory.velocities_rad_s.rows() != sample_count ||
        trajectory.velocities_rad_s.cols() != joint_count ||
        trajectory.accelerations_rad_s2.rows() != sample_count ||
        trajectory.accelerations_rad_s2.cols() != joint_count ||
        trajectory.time_s.size() != sample_count) {
        throw std::runtime_error(label + " has inconsistent array dimensions");
    }
    if (!AllFinite(trajectory.time_s) ||
        !AllFinite(trajectory.positions_rad) ||
        !AllFinite(trajectory.velocities_rad_s) ||
        !AllFinite(trajectory.accelerations_rad_s2)) {
        throw std::runtime_error(label + " contains a non-finite value");
    }
    for (Eigen::Index sample = 1; sample < sample_count; ++sample) {
        const double interval_s =
            trajectory.time_s(sample) - trajectory.time_s(sample - 1);
        if (std::abs(interval_s - sample_time_s) > 1.0e-12) {
            throw std::runtime_error(label + " is not on the configured timing grid");
        }
    }
}

Trajectory ConcatenateWindowedOutputs(
    const std::vector<ShapedWindow>& windows,
    std::size_t expected_sample_count) {
    if (windows.empty()) {
        throw std::invalid_argument("at least one shaped window is required");
    }
    const Eigen::Index joint_count = windows.front().trajectory.positions_rad.cols();
    Trajectory output;
    output.time_s.resize(static_cast<Eigen::Index>(expected_sample_count));
    output.positions_rad.resize(
        static_cast<Eigen::Index>(expected_sample_count), joint_count);
    output.velocities_rad_s.resize(
        static_cast<Eigen::Index>(expected_sample_count), joint_count);
    output.accelerations_rad_s2.resize(
        static_cast<Eigen::Index>(expected_sample_count), joint_count);

    std::size_t cursor = 0;
    for (const ShapedWindow& window : windows) {
        const auto window_samples = static_cast<std::size_t>(
            window.trajectory.positions_rad.rows());
        if (window.is_tail || window.start_index != cursor ||
            window.stop_index != cursor + window_samples ||
            window.stop_index > expected_sample_count) {
            throw std::runtime_error(
                "window ranges do not reconstruct the source sample order");
        }
        const Eigen::Index destination_row = static_cast<Eigen::Index>(cursor);
        const Eigen::Index row_count = static_cast<Eigen::Index>(window_samples);
        output.time_s.segment(destination_row, row_count) =
            window.trajectory.time_s;
        output.positions_rad.middleRows(destination_row, row_count) =
            window.trajectory.positions_rad;
        output.velocities_rad_s.middleRows(destination_row, row_count) =
            window.trajectory.velocities_rad_s;
        output.accelerations_rad_s2.middleRows(destination_row, row_count) =
            window.trajectory.accelerations_rad_s2;
        cursor = window.stop_index;
    }
    if (cursor != expected_sample_count) {
        throw std::runtime_error(
            "window ranges leave a gap in the source sample order");
    }
    return output;
}

SimulatedResponse SimulateModalPositionResponse(
    const Vector& time_s,
    const Vector& command_rad,
    const ModalPlant& plant) {
    if (time_s.size() != command_rad.size() || time_s.size() < 2) {
        throw std::invalid_argument(
            "simulation time and command arrays must have equal nontrivial size");
    }
    if (!(plant.natural_frequency_rad_s > 0.0) ||
        !(plant.damping_ratio > 0.0) || plant.damping_ratio >= 1.0) {
        throw std::invalid_argument("modal plant parameters are not physical");
    }
    const double sample_time_s = time_s(1) - time_s(0);
    if (!(sample_time_s > 0.0)) {
        throw std::invalid_argument("simulation time must be strictly increasing");
    }
    for (Eigen::Index sample = 2; sample < time_s.size(); ++sample) {
        if (std::abs((time_s(sample) - time_s(sample - 1)) - sample_time_s) >
            1.0e-12) {
            throw std::invalid_argument("simulation time must be uniformly spaced");
        }
    }

    const double wn = plant.natural_frequency_rad_s;
    const double zeta = plant.damping_ratio;
    Eigen::Matrix4d augmented = Eigen::Matrix4d::Zero();
    augmented(0, 1) = sample_time_s;
    augmented(1, 0) = -wn * wn * sample_time_s;
    augmented(1, 1) = -2.0 * zeta * wn * sample_time_s;
    augmented(1, 2) = wn * wn * sample_time_s;
    // The last state stores the complete step-to-step input delta, so this
    // normalized first-order-hold transition is intentionally not scaled by dt.
    augmented(2, 3) = 1.0;
    const Eigen::Matrix4d transition = augmented.exp();
    const Eigen::Matrix2d state_transition = transition.block<2, 2>(0, 0);
    const Eigen::Vector2d previous_input =
        transition.block<2, 1>(0, 2) - transition.block<2, 1>(0, 3);
    const Eigen::Vector2d current_input = transition.block<2, 1>(0, 3);

    Vector response_rad = Vector::Zero(time_s.size());
    Eigen::Vector2d state = Eigen::Vector2d::Zero();
    for (Eigen::Index sample = 1; sample < time_s.size(); ++sample) {
        state = state_transition * state +
                previous_input * command_rad(sample - 1) +
                current_input * command_rad(sample);
        response_rad(sample) = state(0);
    }
    return SimulatedResponse{time_s, command_rad, std::move(response_rad)};
}

double ResidualVibrationRad(
    const SimulatedResponse& response,
    double final_value_rad,
    double start_time_s) {
    double residual_rad = 0.0;
    for (Eigen::Index sample = 0; sample < response.time_s.size(); ++sample) {
        if (response.time_s(sample) >= start_time_s) {
            residual_rad = std::max(
                residual_rad,
                std::abs(response.response_rad(sample) - final_value_rad));
        }
    }
    return residual_rad;
}

double FirstChangeTimeS(
    const Trajectory& candidate,
    const Trajectory& reference,
    std::size_t axis_index,
    double tolerance_rad,
    double fallback_time_s) {
    if (axis_index >= static_cast<std::size_t>(candidate.positions_rad.cols()) ||
        axis_index >= static_cast<std::size_t>(reference.positions_rad.cols())) {
        throw std::invalid_argument(
            "candidate and reference trajectories must contain the selected axis");
    }
    const Eigen::Index axis = static_cast<Eigen::Index>(axis_index);
    for (Eigen::Index sample = 0; sample < candidate.time_s.size(); ++sample) {
        const double candidate_time_s = candidate.time_s(sample);
        if (candidate_time_s < reference.time_s(0) ||
            candidate_time_s > reference.time_s(reference.time_s.size() - 1)) {
            throw std::invalid_argument(
                "reference trajectory does not cover every candidate time");
        }
        const double* reference_begin = reference.time_s.data();
        const double* reference_end = reference_begin + reference.time_s.size();
        const double* upper =
            std::lower_bound(reference_begin, reference_end, candidate_time_s);
        double reference_position_rad = 0.0;
        if (upper == reference_begin) {
            reference_position_rad = reference.positions_rad(0, axis);
        } else if (upper == reference_end) {
            reference_position_rad =
                reference.positions_rad(reference.time_s.size() - 1, axis);
        } else {
            const Eigen::Index upper_index = upper - reference_begin;
            if (std::abs(*upper - candidate_time_s) <= 1.0e-12) {
                reference_position_rad =
                    reference.positions_rad(upper_index, axis);
            } else {
                const Eigen::Index lower_index = upper_index - 1;
                const double fraction =
                    (candidate_time_s - reference.time_s(lower_index)) /
                    (reference.time_s(upper_index) -
                     reference.time_s(lower_index));
                reference_position_rad =
                    reference.positions_rad(lower_index, axis) +
                    fraction *
                        (reference.positions_rad(upper_index, axis) -
                         reference.positions_rad(lower_index, axis));
            }
        }
        if (std::abs(
                candidate.positions_rad(sample, axis) -
                reference_position_rad) > tolerance_rad) {
            return candidate_time_s;
        }
    }
    return fallback_time_s;
}

void WriteTrajectoryCsv(
    const std::filesystem::path& output_path,
    const Trajectory& trajectory,
    const std::vector<std::string>& joint_names) {
    if (joint_names.size() !=
        static_cast<std::size_t>(trajectory.positions_rad.cols())) {
        throw std::invalid_argument(
            "joint names must match trajectory column count");
    }
    std::filesystem::create_directories(output_path.parent_path());
    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error(
            "could not open trajectory artifact: " + output_path.string());
    }
    output << "time_s";
    for (const char* field : {"position_rad", "velocity_rad_s", "acceleration_rad_s2"}) {
        for (const std::string& joint_name : joint_names) {
            output << ',' << field << '.' << joint_name;
        }
    }
    output << '\n' << std::setprecision(17);
    for (Eigen::Index sample = 0; sample < trajectory.time_s.size(); ++sample) {
        output << trajectory.time_s(sample);
        for (const Matrix* values : {
                 &trajectory.positions_rad,
                 &trajectory.velocities_rad_s,
                 &trajectory.accelerations_rad_s2}) {
            for (Eigen::Index joint = 0; joint < values->cols(); ++joint) {
                output << ',' << (*values)(sample, joint);
            }
        }
        output << '\n';
    }
    if (!output) {
        throw std::runtime_error(
            "failed while writing trajectory artifact: " + output_path.string());
    }
}

}  // namespace reforge::examples::shaper
