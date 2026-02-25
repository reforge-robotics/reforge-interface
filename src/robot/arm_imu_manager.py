"""
Serves to command an xArm robot through a reference trajectory
and simultaneously record live data through its native API 
and an external IMU.
"""


import time
import numpy as np
from xarm.wrapper import XArmAPI
from xarm.core.utils import convert
from pimu.client import ImuReader, ImuData
from scipy.spatial.transform import Rotation as R

from collections import deque
from dataclasses import dataclass

@dataclass
class ArmData:
    recv_time: float
    positions: list[float]
    velocities: list[float]
    efforts: list[float]

@dataclass
class CommandData:
    cmd_time: float
    cmd_positions: list[float]

class ArmImuManager:
    def __init__(
            self, 
            arm: XArmAPI, 
            imu: ImuReader,
            Ts: float, # sampling period
            time_data: list,
            position_data: list,
            buffer_len_s: float = 3600, # 60 minutes
        ):
        self.arm = arm
        self.imu = imu
        self.Ts = Ts

        # Data to send
        self.time_data = time_data
        self.position_data = position_data
        
        # Prepare data buffers
        maxlen = int(buffer_len_s * (1.0 / Ts))
        self.arm_data: deque[ArmData] = deque(maxlen=maxlen)
        self.imu_data: deque[ImuData] = deque(maxlen=maxlen)
        self.cmd_data: deque[CommandData] = deque(maxlen=maxlen)

        # Output log
        self.aligned_log = deque(maxlen=maxlen)

        # Ensure arm and IMU are ready
        self.prepare_arm()
        self.prepare_imu()

    def prepare_arm(self):
        """Ensure arm is enabled and set to servo control mode."""
        print("Preparing arm for next move...")
        code_en = self.arm.motion_enable(enable=True)
        code_mode = self.arm.set_mode(1) # servo mode
        code_state = self.arm.set_state(0) # start
        print(f"[ArmImuManager] motion_enable->{code_en}, set_mode(1)->{code_mode}, set_state(0)->{code_state}")
        time.sleep(1.0)

    def prepare_imu(self):
        """Wait until IMU data is being received"""
        while True:
            if self.imu.last_data.recv_ts == 0.0:
                print("IMU data not being received. Is server (Pi) running?")
            else:
                break
            time.sleep(1.0)

    def get_imu_data(self) -> ImuData:
        return self.imu.last_data
    
    def get_arm_data(self) -> ArmData:
        _code, (positions, velocities, efforts) = self.arm.get_joint_states()
        return ArmData(
            recv_time=time.time(),
            positions=positions,
            velocities=velocities,
            efforts=efforts
        )


    def publish_and_record(self):
        if len(self.time_data) != len(self.position_data):
            raise ValueError("time_data and position_data must have the same length")
        if not self.time_data:
            return

        tref0 = self.time_data[0]
        t0 = time.time()
        for tref, pref in zip(self.time_data, self.position_data):
            # Wait until it's time for this reference point
            target_time = t0 + (tref - tref0)
            while True:
                now = time.time()
                delta = target_time - now
                if delta <= 0.0:
                    break
                time.sleep(min(delta, self.Ts / 2.0))

            cmd_time = time.time()
            code = self.arm.set_servo_angle_j(angles=pref)
            if code != 0:
                print(f"[ArmImuManager] set_servo_angle_j returned code {code} for pref={pref}")
            self.cmd_data.append(CommandData(cmd_time=cmd_time, cmd_positions=pref))
            # Update sensor data
            self.arm_data.append(self.get_arm_data())
            self.imu_data.append(self.get_imu_data())
        
        print("Trajectory complete!")

        # Package data
        for cmd_d, arm_d, imu_d in zip(self.cmd_data, self.arm_data, self.imu_data):
            cmd_time = cmd_d.cmd_time # all channels have the same since there is one thread
            # Package IMU data
            accel = [imu_d.ax, imu_d.ay, imu_d.az]
            gyro = [imu_d.gx, imu_d.gy, imu_d.gz]

            # Compute TCP orientation from FK TODO: is it necessary that this comes from IMU data?
            q = arm_d.positions
            code, pose = self.arm.get_position_aa()
            # Decompose pose
            position, axang = pose[:3], pose[3:]
            # Convert position coordinates to meters
            position = [coord / 1000.0 for coord in position]
            # Convert rotation coordinates to radians, if necessary
            if not self.arm._is_radian:
                axang = [np.deg2rad(coord) for coord in axang]
            # Convert axis-angle to quaternion 
            tcp_quat = R.from_rotvec(axang).as_quat().tolist()

            self.aligned_log.append({
                    'cmd_time' : cmd_time,
                    'input_positions'  : cmd_d.cmd_positions,
                    'output_positions': q,
                    'velocities': arm_d.velocities,
                    'efforts'   : arm_d.efforts,
                    'imu_time' : cmd_time,
                    'linear_acceleration' : accel,
                    'angular_velocity'    : gyro,
                    'orientation' : tcp_quat,
                })
            
        # Prepare arm to move again
        self.prepare_arm()


if __name__=='__main__':
    ROBOT_IP = "192.168.1.208"
    IMU_IP = "10.12.194.1"

    FREQ_SAMPLING = 250
    TS_SAMPLING = 1 / FREQ_SAMPLING

    imu = ImuReader(IMU_IP)
    arm = XArmAPI(ROBOT_IP, is_radian=True)

    # Move to home
    arm.set_mode(0) # position control
    arm.set_state(0) # start
    time.sleep(1.0)
    arm.move_gohome(wait=True)

    # Come up with slow oscillation reference in joint 1
    freq = 0.2
    amp = 0.1
    data_Ts = 1/250
    time_data = np.arange(0, 5, data_Ts)
    q0ref_data = amp * np.sin(2 * np.pi * freq * time_data)
    position_data = np.zeros((len(time_data), 6))
    position_data[:,0] = q0ref_data

    plt.plot(time_data, q0ref_data)
    plt.show()


    mgr = ArmImuManager(
        arm=arm,
        imu=imu,
        Ts=data_Ts,
        time_data=time_data.tolist(),
        position_data=position_data.tolist()
    )

    mgr.publish_and_record()


    # Sanity check data
    qr = []
    qa = []
    ax = []
    for cmd_d, arm_d, imu_d in zip(mgr.cmd_data, mgr.arm_data, mgr.imu_data):
        qr.append(cmd_d.cmd_positions[0])
        qa.append(arm_d.positions[0])
        ax.append(imu_d.ax)

    plt.plot(qr, 'rx', label='ref')
    plt.plot(qa, 'ko', label='act')
    plt.plot(ax, 'g.', label='imu ax')
    plt.legend()
    plt.show()
