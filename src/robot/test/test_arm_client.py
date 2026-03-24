from __future__ import annotations

import time

import numpy as np

from reforge_core.arm_client import ArmClient, ArmState

# TODO: NEED TO REVIEW

class _FakeArm:
    def __init__(self) -> None:
        self.counter = 0
        self.mode_calls: list[int] = []
        self.state_calls: list[int] = []
        self.motion_enable_calls: list[bool] = []

    def get_joint_states(self):
        self.counter += 1
        base = float(self.counter)
        return 0, (
            [base, base + 1.0],
            [2.0 * base, 2.0 * base + 1.0],
            [3.0 * base, 3.0 * base + 1.0],
        )

    def set_mode(self, mode: int) -> int:
        self.mode_calls.append(mode)
        return 0

    def set_state(self, state: int) -> int:
        self.state_calls.append(state)
        return 0

    def motion_enable(self, enable: bool) -> int:
        self.motion_enable_calls.append(enable)
        return 0

    def set_servo_angle(self, **kwargs) -> int:
        return 0

    def set_servo_angle_j(self, **kwargs) -> int:
        return 0


def test_arm_client_get_state_decodes_xarm_payload() -> None:
    arm = _FakeArm()
    client = ArmClient("127.0.0.1", arm=arm)

    state = client.get_state()

    assert isinstance(state, ArmState)
    assert np.allclose(state.q, [1.0, 2.0])
    assert np.allclose(state.qd, [2.0, 3.0])
    assert np.allclose(state.tau, [3.0, 4.0])


def test_arm_client_recording_accumulates_polled_samples() -> None:
    arm = _FakeArm()
    client = ArmClient("127.0.0.1", arm=arm)

    client.start_recording()
    time.sleep(0.03)
    status = client.stop_recording()
    traj = client.get_recording()

    assert status.sample_count >= 2
    assert len(traj.states) == status.sample_count


def test_arm_client_recording_skips_identical_samples() -> None:
    class _StaticArm:
        def get_joint_states(self):
            return 0, ([1.0, 2.0], [3.0, 4.0], [5.0, 6.0])

    client = ArmClient("127.0.0.1", arm=_StaticArm())
    client.start_recording()
    time.sleep(0.02)
    status = client.stop_recording()
    traj = client.get_recording()

    assert status.sample_count == 1
    assert len(traj.states) == 1


def test_arm_client_reports_get_joint_states_failures_in_status() -> None:
    class _BrokenArm:
        def get_joint_states(self):
            return 1, None

    client = ArmClient("127.0.0.1", arm=_BrokenArm())
    client.start_recording()
    time.sleep(0.02)
    status = client.stop_recording()

    assert status.ok is False
    assert "get_joint_states returned code 1" in status.message
