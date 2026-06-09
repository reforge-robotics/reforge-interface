# Trossen WidowX AI Integration Plan

## Implemented

Owner: Reforge engineering

- Integrated `trossen-arm==1.10.0` with `RobotInterface`.
- Configured the WidowX AI follower model and end effector.
- Implemented position mode, streamed joint commands, point-to-point joint and
  Cartesian commands, joint telemetry, and TCP pose conversion.
- Set the arm and Reforge IMU recording limit to 200 Hz.
- Added a six-joint follower dynamics URDF with bundled vendor assets and
  license.
- Added hardware-free tests for command mapping, telemetry, pose conversion,
  and URDF joint count.

## Remaining Steps

### 1. Confirm the physical arm configuration

Owner: Robot operator

- Confirm the controlled arm is a WidowX AI follower, not a leader or base
  variant.
- Read the arm controller firmware version.
- Confirm firmware compatibility with the pinned `trossen-arm==1.10.0`
  package.

Owner: Reforge engineering

- If the variant is not follower, replace both `TROSSEN_END_EFFECTOR` and the
  bundled dynamics URDF. Changing only the SDK enum is not sufficient.
- Adjust the package pin if the controller requires a different driver release.

### 2. Validate cell safety and calibration start pose

Owner: Robot operator

- Clear the workspace and keep an emergency stop within reach.
- With motors disabled or in a vendor-supported safe mode, verify that
  `FULL_STRETCH_JOINTS` is collision-free:

  ```text
  [0, pi/2, pi/2, 0, 0, 0]
  ```

- Report a safer six-joint pose if the configured pose is not valid for the
  installed cell, tooling, or arm mounting.

Owner: Reforge engineering

- Update `FULL_STRETCH_JOINTS` and the fallback Cartesian pose from the
  approved configuration.

### 3. Configure network and verify connection

Owner: Robot operator

- Connect the control computer to the arm controller Ethernet network.
- Configure an unused host address on the controller subnet.
- Verify the arm IP; the factory default is commonly `192.168.1.2`.
- Release the emergency stop and clear controller faults according to Trossen
  procedures.

Owner: Reforge engineering

- Run:

  ```bash
  source venv/bin/activate
  python -m robot.run connect_test <arm_ip> --robot_id wxai_v0
  ```

- Confirm six positions, velocities, and efforts are available and the TCP
  pose contains seven finite values.

### 4. Install and measure the Reforge IMU

Owner: Robot operator

- Rigidly mount the Reforge IMU to the end effector.
- Connect it over USB and provide the device permissions required by the host
  or container.
- Measure IMU origin to TCP translation in meters, resolved in the IMU frame:
  `imu_to_tcp_x`, `imu_to_tcp_y`, and `imu_to_tcp_z`.
- Record the IMU axis orientation relative to the TCP frame.

Owner: Reforge engineering

- Verify USB discovery and a stable 200 Hz stream.
- Confirm timestamps and sample counts remain healthy while arm telemetry is
  recording.
- Thread the measured translation into each calibration command.

### 5. Perform low-risk motion checks

Owner: Robot operator

- Supervise motion with the emergency stop available.
- Approve a small joint displacement that is safe for the current pose.

Owner: Reforge engineering

- Test a slow blocking joint command.
- Test a small Cartesian command using joint-space interpolation.
- Stream a low-amplitude trajectory at 50 Hz, then 100 Hz, then 200 Hz.
- Check command deadlines, telemetry freshness, controller errors, and
  unexpected vibration before increasing the rate.

### 6. Run a reduced calibration

Owner: Robot operator

- Continue supervising the cell and stop the run if clearance, cabling, or
  vibration becomes unsafe.

Owner: Reforge engineering

- Start with one axis, reduced displacement, and a narrow frequency range.
- Inspect generated arm and IMU data for unit, sign, timestamp, and saturation
  errors.
- Expand to the required axes and normal sweep settings only after the reduced
  run passes.

Example starting point:

```bash
python -m robot.run calibrate <arm_ip> \
  --robot_id wxai_v0 \
  --freq 50 \
  --axes 1 \
  --mdisp 0.03 \
  --mvel 0.2 \
  --macc 0.5 \
  --minfreq 1 \
  --maxfreq 3 \
  --imu_to_tcp_x <meters> \
  --imu_to_tcp_y <meters> \
  --imu_to_tcp_z <meters>
```

### 7. Confirm production settings

Owner: Robot operator

- Confirm the installed payload and any tooling beyond the standard follower
  end effector.
- Provide payload mass and center of mass when they are not represented by the
  bundled model.

Owner: Reforge engineering

- Validate or update `MAX_ROBOT_JOINTS_BANDWIDTH`.
- Set payload CLI arguments.
- Run the full calibration and inspect the saved dataset before identification
  or fine-tuning.
