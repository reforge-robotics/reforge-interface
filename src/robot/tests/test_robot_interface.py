"""Live UR5e adapter tests against fake `ur_rtde` clients.

No socket is ever opened: `RobotInterface` accepts pre-built `rtde_c`/`rtde_r`
objects via constructor injection, so these tests exercise the adapter logic
(SDK call shapes, speed/wait mapping, cleanup, failure surfacing) entirely
against in-memory fakes.
"""

import pytest

import robot.robot_interface as robot_interface_module
from robot.robot_interface import RobotInterface


class FakeControl:
    """Records every call made through it; each command succeeds unless `fail_next`."""

    def __init__(self):
        self.calls = []
        self.fail_next = False

    def moveJ(self, q, speed, acceleration, asynchronous):
        self.calls.append(("moveJ", list(q), speed, acceleration, asynchronous))
        return not self.fail_next

    def moveL(self, pose, speed, acceleration, asynchronous):
        self.calls.append(("moveL", list(pose), speed, acceleration, asynchronous))
        return not self.fail_next

    def servoJ(self, q, speed, acceleration, time, lookahead_time, gain):
        self.calls.append(
            ("servoJ", list(q), speed, acceleration, time, lookahead_time, gain)
        )
        return not self.fail_next

    def servoStop(self, a=10.0):
        self.calls.append(("servoStop", a))
        return True

    def stopScript(self):
        self.calls.append(("stopScript",))

    def disconnect(self):
        self.calls.append(("control.disconnect",))

    def getJointTorques(self):
        return [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]


class FakeReceive:
    def __init__(self):
        self.calls = []
        self.q = [0.0] * 6
        self.qd = [0.0] * 6
        self.tcp_pose = [0.5, 0.1, 0.3, 0.0, 0.0, 0.0]

    def getActualQ(self):
        return list(self.q)

    def getActualQd(self):
        return list(self.qd)

    def getActualTCPPose(self):
        return list(self.tcp_pose)

    def disconnect(self):
        self.calls.append(("receive.disconnect",))


@pytest.fixture
def fake_control():
    return FakeControl()


@pytest.fixture
def fake_receive():
    return FakeReceive()


@pytest.fixture
def robot(fake_control, fake_receive):
    return RobotInterface(robot_ip="10.0.0.5", rtde_c=fake_control, rtde_r=fake_receive)


# --- Construction / injectability -------------------------------------------------


def test_construction_uses_injected_clients_without_networking(robot, fake_control, fake_receive):
    assert robot.robot.control is fake_control
    assert robot.robot.receive is fake_receive


def test_construction_resolves_six_joints_and_seven_length_pose(robot):
    assert robot.num_joints == 6
    assert robot.pose_length == 7


def test_construction_surfaces_connection_failures_clearly(monkeypatch):
    class RaisingControl:
        def __init__(self, hostname):
            raise RuntimeError("Could not connect to: 10.0.0.5 at 30004, verify the IP")

    monkeypatch.setattr(robot_interface_module, "RTDEControlInterface", RaisingControl)

    with pytest.raises(RuntimeError) as exc_info:
        RobotInterface(robot_ip="10.0.0.5")

    assert "Error getting 10.0.0.5 operational" in str(exc_info.value)
    assert "Could not connect" in str(exc_info.value)


# --- Telemetry: returned shapes -----------------------------------------------------


def test_get_joint_state_returns_position_velocity_torque_in_urdf_order(robot, fake_receive, fake_control):
    fake_receive.q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    fake_receive.qd = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    q, qd, tau = robot.get_joint_state()

    assert q == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert qd == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert tau == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # from FakeControl.getJointTorques()
    assert len(q) == len(qd) == len(tau) == 6


def test_get_tcp_pose_returns_seven_values_xyz_then_quat(robot, fake_receive):
    fake_receive.tcp_pose = [0.5, 0.1, 0.3, 0.0, 0.0, 0.0]  # identity rotation

    pose = robot.get_tcp_pose()

    assert len(pose) == 7
    assert pose[:3] == [0.5, 0.1, 0.3]
    assert pose[3:] == pytest.approx([0.0, 0.0, 0.0, 1.0])  # identity quaternion


# --- command_move_j: speed%->rad/s mapping, wait->asynchronous -----------------------


def test_command_move_j_maps_speed_percent_and_wait(robot, fake_control):
    code = robot.command_move_j([0.1] * 6, speed=50.0, wait=True)

    assert code == 0
    name, q, speed, accel, asynchronous = fake_control.calls[-1]
    assert name == "moveJ"
    assert q == [0.1] * 6
    assert speed == pytest.approx(0.525)  # 50% of MAX_JOINT_SPEED_RAD_S=1.05
    assert accel == pytest.approx(0.7)  # 50% of MAX_JOINT_ACCEL_RAD_S2=1.4
    assert asynchronous is False  # wait=True


def test_command_move_j_wait_false_is_asynchronous(robot, fake_control):
    robot.command_move_j([0.0] * 6, wait=False)

    assert fake_control.calls[-1][-1] is True


def test_command_move_j_raises_on_sdk_failure(robot, fake_control):
    fake_control.fail_next = True

    with pytest.raises(RuntimeError, match="moveJ"):
        robot.command_move_j([0.0] * 6)


# --- command_move_pose: quat->rotvec, speed%->m/s mapping, locked_joints -------------


def test_command_move_pose_converts_quat_to_rotvec_and_maps_speed(robot, fake_control):
    code = robot.command_move_pose(
        target_quat=[0.0, 0.0, 0.0, 1.0], target_xyz=[0.1, 0.2, 0.3], speed=100.0, wait=False
    )

    assert code == 0
    name, pose, speed, accel, asynchronous = fake_control.calls[-1]
    assert name == "moveL"
    assert pose[:3] == pytest.approx([0.1, 0.2, 0.3])
    assert pose[3:] == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)  # identity quat -> zero rotvec
    assert speed == pytest.approx(0.25)  # 100% of MAX_LINEAR_SPEED_M_S
    assert accel == pytest.approx(1.2)  # 100% of MAX_LINEAR_ACCEL_M_S2
    assert asynchronous is True  # wait=False


def test_command_move_pose_rejects_locked_joints_on_live_adapter(robot):
    with pytest.raises(RuntimeError, match="simulator mode"):
        robot.command_move_pose(
            target_quat=[0, 0, 0, 1], target_xyz=[0, 0, 0], locked_joints={0: 0.1}
        )


def test_command_move_pose_raises_on_sdk_failure(robot, fake_control):
    fake_control.fail_next = True

    with pytest.raises(RuntimeError, match="moveL"):
        robot.command_move_pose(target_quat=[0, 0, 0, 1], target_xyz=[0, 0, 0])


# --- command_servo_j: the streamed command used by the base trajectory publisher ----


def test_command_servo_j_uses_fixed_gains_and_own_command_period(robot, fake_control):
    code = robot.command_servo_j([0.2] * 6)

    assert code == 0
    name, q, speed, accel, t, lookahead, gain = fake_control.calls[-1]
    assert name == "servoJ"
    assert q == [0.2] * 6
    assert speed == 0.0 and accel == 0.0  # unused by current RTDE protocol version
    assert t == pytest.approx(1.0 / robot.max_sampling_frequency_hz)
    assert lookahead == robot_interface_module.SERVO_LOOKAHEAD_TIME_S
    assert gain == robot_interface_module.SERVO_GAIN


def test_command_servo_j_raises_on_sdk_failure(robot, fake_control):
    fake_control.fail_next = True

    with pytest.raises(RuntimeError, match="servoJ"):
        robot.command_servo_j([0.0] * 6)


def test_command_joint_trajectory_streams_every_sample_through_command_servo_j(robot, fake_control):
    """Exercises the real base-class scheduler (ArmClient.stream_joint_positions),
    not just an isolated command_servo_j() call -- this is the actual path
    Shaper acquisition uses."""
    Ts = 1.0 / robot.max_sampling_frequency_hz
    n = 5
    time_data = [i * Ts for i in range(n)]
    position_stream = [[0.01 * i] * 6 for i in range(n)]

    records = robot.command_joint_trajectory(
        time_data=time_data, position_stream=position_stream, Ts=Ts
    )

    assert len(records) == n
    servoj_calls = [c for c in fake_control.calls if c[0] == "servoJ"]
    assert len(servoj_calls) == n
    for i, (_, positions) in enumerate(records):
        assert positions == position_stream[i]
        assert servoj_calls[i][1] == position_stream[i]
        assert servoj_calls[i][4] == pytest.approx(Ts)


# --- mode hooks -----------------------------------------------------------------------


def test_enter_position_mode_stops_any_active_servo_stream(robot, fake_control):
    assert robot.enter_position_mode() == 0
    assert fake_control.calls[-1][0] == "servoStop"


def test_enter_servo_mode_succeeds_when_connected(robot):
    assert robot.enter_servo_mode() == 0


# --- cleanup ----------------------------------------------------------------------------


def test_close_stops_and_disconnects_both_clients(robot, fake_control, fake_receive):
    robot.close()

    call_names = [c[0] for c in fake_control.calls[-3:]]
    assert call_names == ["servoStop", "stopScript", "control.disconnect"]
    assert fake_receive.calls[-1] == ("receive.disconnect",)
    assert robot.robot is None


def test_close_is_a_no_op_when_already_closed(robot):
    robot.close()
    robot.close()  # must not raise
