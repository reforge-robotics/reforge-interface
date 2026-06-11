from types import SimpleNamespace
from importlib.resources import as_file, files

import numpy as np
import trossen_arm

from reforge_core.util.robot_dynamics import Dynamics
from robot.robot_interface import RobotInterface, TROSSEN_GRIPPER_HOLD_POSITION_M


class FakeTrossenArm:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple]] = []

    def set_arm_positions(self, *args) -> None:
        self.calls.append(("arm", args))

    def set_cartesian_positions(self, *args) -> None:
        self.calls.append(("cartesian", args))

    def set_arm_modes(self, *args) -> None:
        self.calls.append(("mode", args))

    def set_arm_external_efforts(self, *args) -> None:
        self.calls.append(("external_effort", args))

    def get_joint_limits(self) -> list[SimpleNamespace]:
        arm_limits = [
            SimpleNamespace(position_min=-np.pi, position_max=np.pi)
            for _ in range(6)
        ]
        return [
            *arm_limits,
            SimpleNamespace(position_min=0.0, position_max=0.04),
        ]

    def set_gripper_mode(self, *args) -> None:
        self.calls.append(("gripper_mode", args))

    def set_gripper_position(self, *args) -> None:
        self.calls.append(("gripper_position", args))

    def get_gripper_position(self) -> float:
        return 0.025

    def get_robot_output(self) -> SimpleNamespace:
        arm = SimpleNamespace(
            positions=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            velocities=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            efforts=[7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
        )
        return SimpleNamespace(joint=SimpleNamespace(arm=arm))

    def get_cartesian_positions(self) -> list[float]:
        return [0.4, 0.1, 0.2, 0.0, 0.0, np.pi / 2]


def make_interface() -> RobotInterface:
    interface = object.__new__(RobotInterface)
    interface.robot = FakeTrossenArm()
    interface.num_joints = 6
    return interface


def test_trossen_dynamics_urdf_has_six_arm_joints() -> None:
    resource = files("robot").joinpath("urdf/trossen/wxai_follower.urdf")
    with as_file(resource) as urdf_path:
        model = Dynamics(str(urdf_path))
        assert model.num_joints == 6


def test_joint_commands_use_trossen_position_api() -> None:
    interface = make_interface()

    assert interface.command_move_j([0.0] * 6) == 0
    assert interface.command_servo_j(np.ones(6)) == 0

    assert interface.robot.calls[0][0] == "arm"
    assert interface.robot.calls[0][1] == ([0.0] * 6, 2.0, True)
    assert interface.robot.calls[1][1] == ([1.0] * 6, 0.0, False)


def test_teach_mode_uses_zero_external_effort_then_brakes() -> None:
    interface = make_interface()

    assert interface.enter_teach_mode() == 0
    assert interface.exit_teach_mode() == 0

    assert interface.robot.calls[0] == (
        "mode",
        (trossen_arm.Mode.external_effort,),
    )
    assert interface.robot.calls[1] == (
        "external_effort",
        ([0.0] * 6, 0.0, False),
    )
    assert interface.robot.calls[2] == ("mode", (trossen_arm.Mode.idle,))


def test_gripper_position_is_validated_and_held_in_position_mode() -> None:
    interface = make_interface()

    assert interface.command_gripper_position(0.025, move_time_s=3.0) == 0
    assert interface.get_gripper_position() == 0.025
    assert interface.robot.calls == [
        ("gripper_mode", (trossen_arm.Mode.position,)),
        ("gripper_position", (0.025, 3.0, True)),
    ]

    with np.testing.assert_raises(ValueError):
        interface.command_gripper_position(0.05)


def test_hold_gripper_uses_installed_imu_position() -> None:
    interface = make_interface()

    assert interface.hold_gripper() == 0
    assert interface.robot.calls[-1] == (
        "gripper_position",
        (TROSSEN_GRIPPER_HOLD_POSITION_M, 2.0, True),
    )


def test_joint_state_uses_one_robot_output_snapshot() -> None:
    interface = make_interface()

    q, qd, tau = interface.get_joint_state()

    assert q == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert qd == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    assert tau == [7.0, 8.0, 9.0, 10.0, 11.0, 12.0]


def test_cartesian_pose_converts_between_quaternion_and_rotation_vector() -> None:
    interface = make_interface()

    assert interface.command_move_pose(
        target_quat=[0.0, 0.0, 0.0, 1.0],
        target_xyz=[0.1, 0.2, 0.3],
    ) == 0
    pose = interface.get_tcp_pose()

    cartesian_call = interface.robot.calls[0]
    assert cartesian_call[0] == "cartesian"
    assert np.allclose(cartesian_call[1][0], [0.1, 0.2, 0.3, 0.0, 0.0, 0.0])
    assert np.allclose(pose[:3], [0.4, 0.1, 0.2])
    assert np.allclose(
        pose[3:],
        [0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)],
    )
