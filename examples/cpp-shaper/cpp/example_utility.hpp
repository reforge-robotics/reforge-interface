#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "reforge_core/native/common/array_types.hpp"

namespace reforge::examples::shaper {

/** Own sample-aligned joint commands and their derivatives. */
struct Trajectory final {
    reforge::native::Vector time_s;
    reforge::native::Matrix positions_rad;
    reforge::native::Matrix velocities_rad_s;
    reforge::native::Matrix accelerations_rad_s2;
};

/** Describe one dominant flexible mode for the hardware-free plant. */
struct ModalPlant final {
    double natural_frequency_rad_s = 0.0;
    double damping_ratio = 0.0;
};

/** Own one simulated command and modal-plant response. */
struct SimulatedResponse final {
    reforge::native::Vector time_s;
    reforge::native::Vector command_rad;
    reforge::native::Vector response_rad;
};

/** Own one shaped window and its stable source range. */
struct ShapedWindow final {
    Trajectory trajectory;
    std::size_t start_index = 0;
    std::size_t stop_index = 0;
    bool is_tail = false;
};

/** Generate a quintic point-to-point move followed by a target dwell.
 *
 * Args:
 *     start_position_rad: Initial joint positions [rad].
 *     goal_position_rad: Final joint positions [rad].
 *     sample_time_s: Controller sample period [s].
 *     move_duration_s: Point-to-point move duration [s].
 *     dwell_duration_s: Final target dwell duration [s].
 *
 * Returns:
 *     Sample-aligned position, velocity, and acceleration commands.
 */
[[nodiscard]] Trajectory GeneratePointToPointTrajectory(
    const reforge::native::Vector& start_position_rad,
    const reforge::native::Vector& goal_position_rad,
    double sample_time_s,
    double move_duration_s,
    double dwell_duration_s);

/** Generate one analytic sample of a quintic point-to-point move.
 *
 * Args:
 *     start_position_rad: Initial joint positions [rad].
 *     goal_position_rad: Final joint positions [rad].
 *     current_time_s: Current time from the move start [s].
 *     move_duration_s: Point-to-point move duration [s].
 *
 * Returns:
 *     One-row position, velocity, and acceleration command.
 */
[[nodiscard]] Trajectory GeneratePointToPointSample(
    const reforge::native::Vector& start_position_rad,
    const reforge::native::Vector& goal_position_rad,
    double current_time_s,
    double move_duration_s);

/** Estimate velocity and acceleration with NumPy-compatible gradients.
 *
 * Args:
 *     positions_rad: Sample-major joint positions [rad].
 *     sample_time_s: Controller sample period [s].
 *
 * Returns:
 *     Sample-major velocity [rad/s] and acceleration [rad/s^2].
 */
[[nodiscard]] std::pair<reforge::native::Matrix, reforge::native::Matrix>
EstimateDerivatives(
    const reforge::native::Matrix& positions_rad,
    double sample_time_s);

/** Validate dimensions, finiteness, and a uniform timing grid.
 *
 * Args:
 *     trajectory: Trajectory to validate.
 *     expected_joint_count: Required number of joint columns.
 *     sample_time_s: Required uniform sample period [s].
 *     label: Human-readable trajectory identity for diagnostics.
 */
void ValidateTrajectory(
    const Trajectory& trajectory,
    std::size_t expected_joint_count,
    double sample_time_s,
    const std::string& label);

/** Concatenate fixed-window outputs while proving exact source coverage.
 *
 * Args:
 *     windows: Shaped windows in emitted order.
 *     expected_sample_count: Required total source sample count.
 *
 * Returns:
 *     One reconstructed sample-major trajectory.
 */
[[nodiscard]] Trajectory ConcatenateWindowedOutputs(
    const std::vector<ShapedWindow>& windows,
    std::size_t expected_sample_count);

/** Simulate a unity-gain second-order plant with linear input interpolation.
 *
 * This matches `scipy.signal.lsim` for an equally spaced time grid and its
 * default first-order-hold input interpolation.
 *
 * Args:
 *     time_s: Strictly increasing, uniformly spaced sample times [s].
 *     command_rad: Position command at each sample [rad].
 *     plant: Dominant flexible-mode parameters.
 *
 * Returns:
 *     Sample-aligned position response [rad].
 */
[[nodiscard]] SimulatedResponse SimulateModalPositionResponse(
    const reforge::native::Vector& time_s,
    const reforge::native::Vector& command_rad,
    const ModalPlant& plant);

/** Return maximum absolute final-position error after a selected time.
 *
 * Args:
 *     response: Simulated modal-plant response.
 *     final_value_rad: Expected final position [rad].
 *     start_time_s: Inclusive residual-measurement start time [s].
 *
 * Returns:
 *     Maximum absolute residual vibration [rad].
 */
[[nodiscard]] double ResidualVibrationRad(
    const SimulatedResponse& response,
    double final_value_rad,
    double start_time_s);

/** Return the first time that a trajectory differs from an interpolated reference.
 *
 * Args:
 *     candidate: Candidate shaped trajectory.
 *     reference: Reference trajectory covering every candidate time.
 *     axis_index: Joint column to compare.
 *     tolerance_rad: Absolute change threshold [rad].
 *     fallback_time_s: Value returned when no change is found [s].
 *
 * Returns:
 *     First changed sample time or `fallback_time_s` [s].
 */
[[nodiscard]] double FirstChangeTimeS(
    const Trajectory& candidate,
    const Trajectory& reference,
    std::size_t axis_index,
    double tolerance_rad,
    double fallback_time_s);

/** Write one deterministic CSV artifact for cross-language comparison.
 *
 * Args:
 *     output_path: Destination CSV path.
 *     trajectory: Trajectory whose fields are written.
 *     joint_names: Ordered joint-column names.
 */
void WriteTrajectoryCsv(
    const std::filesystem::path& output_path,
    const Trajectory& trajectory,
    const std::vector<std::string>& joint_names);

}  // namespace reforge::examples::shaper
