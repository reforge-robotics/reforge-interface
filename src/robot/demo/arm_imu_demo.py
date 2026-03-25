"""Linear arm/IMU demo using the current ArmClient and MuseIMUClient APIs.

This script is intentionally written as a tutorial, not a reusable module.
It shows:
1. Programmatic time-offset calibration.
2. Reuse of that same IMU client after updating its fitted software-side offset.
3. Homing the robot with the arm client helper.
4. Building a base-joint sinusoidal trajectory around the current home pose.
5. Running that trajectory once in streaming mode with manual sampling.
6. Running the same trajectory again in recording mode.
"""

from __future__ import annotations

import time

import matplotlib.pyplot as plt
import numpy as np
from reforge_core.imu.client import MuseIMUClient
from reforge_core.imu.data import IMUStateTraj

from robot.arm_client import ArmClient, ArmStateTraj
from robot.arm_imu_time_calibration import (
    run_arm_imu_time_calibration,
    solve_scale_and_time_offset,
)


ROBOT_IP = "192.168.1.208"
CONTROL_HZ = 250.0
IMU_STREAM_HZ = 200.0
IMU_RECORD_HZ = 200.0
SINE_AMPLITUDE_DEG = 5.0
SINE_FREQUENCY_HZ = 1.0
SINE_CYCLES = 3
IMU_GET_TIMEOUT_S = 1.0


def main() -> None:
    print("=== Arm/IMU demo ===")
    print("Step 1: Calibrate IMU time offset against the arm.")
    imu_client = MuseIMUClient(time_offset_sec=0.0)
    calibration_result, imu_client = run_arm_imu_time_calibration(
        arm_control_mode="servo", # or "position" if you will be controlling robot in position mode
        imu_client=imu_client,
        plot=True,
        plot_block=True,
        bias_calibration_sec=1.0,
    )
    print(f"Fitted time_offset_sec = {calibration_result.time_offset_sec:.9f}")
    print(
        "Use this value directly in MuseIMUClient(time_offset_sec=...). "
        "Do not negate it."
    )

    arm_client = ArmClient(ROBOT_IP, recording_data_frequency_hz=CONTROL_HZ)
    q_home: np.ndarray | None = None

    try:
        print("\nStep 2: Reuse the calibrated IMU client with its fitted offset already set.")

        print("\nStep 3: Move the robot to wrapper home.")
        arm_client.move_home()

        # Read the current home joint vector after the homing motion finishes.
        q_home = arm_client.get_latest_state().q

        print("\nStep 4: Build a sinusoidal base-joint trajectory.")
        duration_s = SINE_CYCLES / SINE_FREQUENCY_HZ
        sample_count = int(round(duration_s * CONTROL_HZ)) + 1
        time_data = np.arange(sample_count, dtype=float) / CONTROL_HZ
        amplitude_rad = np.deg2rad(SINE_AMPLITUDE_DEG)
        base_joint_offset = amplitude_rad * np.sin(
            2.0 * np.pi * SINE_FREQUENCY_HZ * time_data
        )

        # Start from the home pose at every sample. Joints 2-6 remain unchanged.
        # Only joint 1 gets the sinusoidal offset.
        position_data = np.tile(q_home, (sample_count, 1))
        position_data[:, 0] = q_home[0] + base_joint_offset

        print(
            f"Built trajectory with shape {position_data.shape} over "
            f"{time_data[-1]:.3f} s."
        )

        print("\nStep 5: Streaming demo with manual sampling.")
        imu_client.set_streaming_data_frequency(IMU_STREAM_HZ)
        # Joint servo mode is the arm-side mode intended for high-frequency
        # control, so use it before sending the fast sinusoidal reference.
        arm_client.set_servo_mode()

        streamed_arm_traj = ArmStateTraj(maxlen=time_data.size + 1)
        streamed_imu_traj = IMUStateTraj(maxlen=time_data.size + 1)

        imu_client.start_streaming()  # only IMU needs start_streaming(), not arm
        stream_start_wall_time = time.time()
        try:
            for tref, q_cmd in zip(time_data, position_data):
                target_time = stream_start_wall_time + float(tref)
                while True:
                    now = time.time()
                    remaining = target_time - now
                    if remaining <= 0.0:
                        break
                    time.sleep(min(remaining, (1.0 / CONTROL_HZ) / 2.0))

                # In streaming mode we sample both devices manually, then send the
                # next high-frequency servo command through the wrapper's
                # set_servo_angle_j(...) interface.
                streamed_arm_traj.append(arm_client.get_state())
                streamed_imu_traj.append(
                    imu_client.get_latest_state(timeout_s=IMU_GET_TIMEOUT_S)
                )

                code = arm_client.arm.set_servo_angle_j(
                    angles=np.asarray(q_cmd, dtype=float).tolist()
                )
                if code != 0:
                    raise RuntimeError(f"set_servo_angle_j(...) returned code {code}")

            # Grab one more sample after the last command so the plots include the
            # settled endpoint.
            time.sleep(1.0 / CONTROL_HZ)
            streamed_arm_traj.append(arm_client.get_state())
            streamed_imu_traj.append(
                imu_client.get_latest_state(timeout_s=IMU_GET_TIMEOUT_S)
            )
        finally:
            imu_client.stop_streaming()

        stream_time_offset_sec, stream_scale, stream_rmse = solve_scale_and_time_offset(
            streamed_arm_traj,
            streamed_imu_traj,
            imu_bias=calibration_result.imu_bias,
        )
        print(
            "Streaming capture fit: "
            f"time_offset_sec={stream_time_offset_sec:.9f}, "
            f"scale={stream_scale:.6f}, rmse={stream_rmse:.6f}"
        )

        ts_arm_stream, q_arm_stream, qd_arm_stream, _tau_arm_stream = streamed_arm_traj.unpack()
        ts_imu_stream, accel_stream, gyro_stream = streamed_imu_traj.unpack()
        t_arm_stream = ts_arm_stream - stream_start_wall_time
        t_imu_stream = ts_imu_stream - stream_start_wall_time

        fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=False)
        axes[0].plot(
            time_data,
            position_data[:, 0],
            color="tab:blue",
            linewidth=2.0,
            label="Desired joint 1",
        )
        axes[0].plot(
            t_arm_stream,
            q_arm_stream[:, 0],
            color="tab:orange",
            linestyle="--",
            linewidth=1.8,
            label="Sampled joint 1",
        )
        axes[0].set_title("Streaming Mode: Desired vs Sampled Joint 1")
        axes[0].set_xlabel("Time since section start [s]")
        axes[0].set_ylabel("Joint 1 angle [rad]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(t_imu_stream, accel_stream[:, 0], label="ax")
        axes[1].plot(t_imu_stream, accel_stream[:, 1], label="ay")
        axes[1].plot(t_imu_stream, accel_stream[:, 2], label="az")
        axes[1].set_title("Streaming Mode: IMU Acceleration")
        axes[1].set_xlabel("IMU time since section start [s]")
        axes[1].set_ylabel("Acceleration [m/s^2]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()

        axes[2].plot(t_imu_stream, gyro_stream[:, 0], label="gx")
        axes[2].plot(t_imu_stream, gyro_stream[:, 1], label="gy")
        axes[2].plot(t_imu_stream, gyro_stream[:, 2], label="gz")
        axes[2].set_title("Streaming Mode: IMU Angular Velocity")
        axes[2].set_xlabel("IMU time since section start [s]")
        axes[2].set_ylabel("Angular velocity [rad/s]")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()

        axes[3].plot(
            ts_arm_stream,
            np.abs(qd_arm_stream[:, 0]),
            color="tab:blue",
            linewidth=1.8,
            label=r"Arm $|\dot{q}_0|$",
        )
        axes[3].plot(
            ts_imu_stream,
            np.linalg.norm(gyro_stream, axis=1),
            color="tab:orange",
            linestyle="--",
            linewidth=1.6,
            label=r"IMU $\|\omega\|$",
        )
        axes[3].plot(
            ts_imu_stream + stream_time_offset_sec,
            np.maximum(np.linalg.norm(gyro_stream, axis=1) - calibration_result.imu_bias, 0.0)
            * stream_scale,
            color="tab:green",
            linestyle=":",
            linewidth=1.8,
            label=(
                rf"IMU aligned $\|\omega\|$ "
                f"(time_offset_sec={stream_time_offset_sec:.3f} s)"
            ),
        )
        axes[3].set_title("Streaming Mode: Arm vs IMU Angular-Speed Magnitude")
        axes[3].set_xlabel("Timestamp [s]")
        axes[3].set_ylabel("Magnitude [rad/s]")
        axes[3].grid(True, alpha=0.3)
        axes[3].legend()
        plt.tight_layout()
        plt.show()

        print("\nStep 6: Reset to home before the recording demo.")
        arm_client.set_position_mode()
        code = arm_client.command_position(q_home, wait=True)
        if code != 0:
            raise RuntimeError(f"command_position(q_home) returned code {code}")
        time.sleep(1.0)

        print("\nStep 7: Recording demo with the built-in recorders.")
        imu_client.set_recording_data_frequency(IMU_RECORD_HZ)
        arm_client.set_recording_data_frequency(CONTROL_HZ)
        # Again, use joint servo mode because this is the wrapper path designed
        # for high-frequency command updates.
        arm_client.set_servo_mode()

        imu_client.start_recording()
        arm_client.start_recording()
        record_start_wall_time = time.time()
        try:
            for tref, q_cmd in zip(time_data, position_data):
                target_time = record_start_wall_time + float(tref)
                while True:
                    now = time.time()
                    remaining = target_time - now
                    if remaining <= 0.0:
                        break
                    time.sleep(min(remaining, (1.0 / CONTROL_HZ) / 2.0))

                # In recording mode the clients collect sampled data internally,
                # so we only need to send the high-frequency joint servo command.
                code = arm_client.arm.set_servo_angle_j(
                    angles=np.asarray(q_cmd, dtype=float).tolist()
                )
                if code != 0:
                    raise RuntimeError(f"set_servo_angle_j(...) returned code {code}")
        finally:
            imu_client.stop_recording()
            arm_client.stop_recording()

        recorded_arm_traj = arm_client.get_recording()
        recorded_imu_traj = imu_client.get_recording()
        record_time_offset_sec, record_scale, record_rmse = solve_scale_and_time_offset(
            recorded_arm_traj,
            recorded_imu_traj,
            imu_bias=calibration_result.imu_bias,
        )
        print(
            "Recording capture fit: "
            f"time_offset_sec={record_time_offset_sec:.9f}, "
            f"scale={record_scale:.6f}, rmse={record_rmse:.6f}"
        )

        ts_arm_record, q_arm_record, qd_arm_record, _tau_arm_record = recorded_arm_traj.unpack()
        ts_imu_record, accel_record, gyro_record = recorded_imu_traj.unpack()
        t_arm_record = ts_arm_record - record_start_wall_time
        t_imu_record = ts_imu_record - record_start_wall_time

        fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=False)
        axes[0].plot(
            time_data,
            position_data[:, 0],
            color="tab:blue",
            linewidth=2.0,
            label="Desired joint 1",
        )
        axes[0].plot(
            t_arm_record,
            q_arm_record[:, 0],
            color="tab:orange",
            linestyle="--",
            linewidth=1.8,
            label="Recorded joint 1",
        )
        axes[0].set_title("Recording Mode: Desired vs Recorded Joint 1")
        axes[0].set_xlabel("Time since section start [s]")
        axes[0].set_ylabel("Joint 1 angle [rad]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(t_imu_record, accel_record[:, 0], label="ax")
        axes[1].plot(t_imu_record, accel_record[:, 1], label="ay")
        axes[1].plot(t_imu_record, accel_record[:, 2], label="az")
        axes[1].set_title("Recording Mode: IMU Acceleration")
        axes[1].set_xlabel("IMU time since section start [s]")
        axes[1].set_ylabel("Acceleration [m/s^2]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()

        axes[2].plot(t_imu_record, gyro_record[:, 0], label="gx")
        axes[2].plot(t_imu_record, gyro_record[:, 1], label="gy")
        axes[2].plot(t_imu_record, gyro_record[:, 2], label="gz")
        axes[2].set_title("Recording Mode: IMU Angular Velocity")
        axes[2].set_xlabel("IMU time since section start [s]")
        axes[2].set_ylabel("Angular velocity [rad/s]")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()

        axes[3].plot(
            ts_arm_record,
            np.abs(qd_arm_record[:, 0]),
            color="tab:blue",
            linewidth=1.8,
            label=r"Arm $|\dot{q}_0|$",
        )
        axes[3].plot(
            ts_imu_record,
            np.linalg.norm(gyro_record, axis=1),
            color="tab:orange",
            linestyle="--",
            linewidth=1.6,
            label=r"IMU $\|\omega\|$",
        )
        axes[3].plot(
            ts_imu_record + record_time_offset_sec,
            np.maximum(np.linalg.norm(gyro_record, axis=1) - calibration_result.imu_bias, 0.0)
            * record_scale,
            color="tab:green",
            linestyle=":",
            linewidth=1.8,
            label=(
                rf"IMU aligned $\|\omega\|$ "
                f"(time_offset_sec={record_time_offset_sec:.3f} s)"
            ),
        )
        axes[3].set_title("Recording Mode: Arm vs IMU Angular-Speed Magnitude")
        axes[3].set_xlabel("Timestamp [s]")
        axes[3].set_ylabel("Magnitude [rad/s]")
        axes[3].grid(True, alpha=0.3)
        axes[3].legend()
        plt.tight_layout()
        plt.show()

    finally:
        # Best-effort cleanup so the demo leaves the robot and IMU in a sane state.
        try:
            arm_client.stop_recording()
        except Exception:
            pass
        try:
            imu_client.stop_streaming()
        except Exception:
            pass
        try:
            imu_client.stop_recording()
        except Exception:
            pass
        if q_home is not None:
            try:
                arm_client.set_position_mode()
                arm_client.command_position(q_home, wait=True)
            except Exception:
                pass
        try:
            imu_client.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
