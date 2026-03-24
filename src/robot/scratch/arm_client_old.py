"""Scratch xArm client and arm trajectory types for arm/IMU sync experiments."""

from __future__ import annotations

import datetime
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from xarm.wrapper import XArmAPI


@dataclass
class ArmState:
    """One sampled arm state.

    Attributes:
        q: Joint positions [rad].
        qd: Joint velocities [rad/s].
        tau: Joint efforts / currents from the xArm wrapper.
        ts: UTC unix epoch timestamp in seconds.
    """

    q: np.ndarray
    qd: np.ndarray
    tau: np.ndarray
    ts: float

    def array(self) -> np.ndarray:
        """Return one flat array containing position, velocity, and effort."""
        return np.concatenate([self.q, self.qd, self.tau])


class ArmStateTraj:
    """In-memory trajectory of timestamped arm states.

    All timestamps are UTC unix epoch seconds.
    """

    def __init__(self, maxlen: int = int(1e6)):
        self.states: deque[ArmState] = deque(maxlen=maxlen)

    def append(self, state: ArmState) -> None:
        self.states.append(state)

    def extend(self, states: list[ArmState]) -> None:
        self.states.extend(states)

    def unpack(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        ts = np.array([state.ts for state in self.states], dtype=float)
        q = np.array([state.q for state in self.states])
        qd = np.array([state.qd for state in self.states])
        tau = np.array([state.tau for state in self.states])
        return ts, q, qd, tau


class ArmClient:
    """Small scratch wrapper around ``xarm.wrapper.XArmAPI``."""

    def __init__(self, robot_ip: str, logging_data_frequency_hz: float = 250.0):
        self.robot_ip = robot_ip
        self.arm = XArmAPI(robot_ip, is_radian=True)

        self._logging_data_frequency_hz = 0.0
        self._record_thread: threading.Thread | None = None
        self._record_stop = threading.Event()
        self._record_lock = threading.Lock()
        self._thread_error: BaseException | None = None
        self._latest_state: ArmState | None = None
        self._recording = ArmStateTraj()

        self.set_logging_data_frequency(logging_data_frequency_hz)

    def get_state(self) -> ArmState:
        code, (q, qd, tau) = self.arm.get_joint_states()
        if code != 0:
            raise RuntimeError(f"get_joint_states returned code {code}")
        state = ArmState(
            ts=datetime.datetime.now(datetime.timezone.utc).timestamp(),
            q=np.asarray(q, dtype=float),
            qd=np.asarray(qd, dtype=float),
            tau=np.asarray(tau, dtype=float),
        )
        with self._record_lock:
            self._latest_state = state
        return state

    def get_latest_state(self) -> ArmState:
        with self._record_lock:
            latest_state = self._latest_state
        if latest_state is not None:
            return latest_state
        return self.get_state()

    def set_logging_data_frequency(self, frequency_hz: float) -> float:
        frequency_hz = float(frequency_hz)
        if frequency_hz <= 0.0:
            raise ValueError("`frequency_hz` must be > 0.")
        self._logging_data_frequency_hz = frequency_hz
        return self._logging_data_frequency_hz

    def set_position_mode(self, settle_s: float = 1.0) -> None:
        code_mode = self.arm.set_mode(0)
        code_state = self.arm.set_state(0)
        print(f"[ArmClient] set_mode(0)->{code_mode}, set_state(0)->{code_state}")
        time.sleep(settle_s)

    def set_servo_mode(self, settle_s: float = 1.0) -> None:
        code_en = self.arm.motion_enable(enable=True)
        code_mode = self.arm.set_mode(1)
        code_state = self.arm.set_state(0)
        print(
            f"[ArmClient] motion_enable->{code_en}, set_mode(1)->{code_mode}, set_state(0)->{code_state}"
        )
        time.sleep(settle_s)

    def command_position(
        self,
        q: np.ndarray | list[float],
        *,
        speed: float = 100.0,
        wait: bool = True,
    ) -> int:
        return self.arm.set_servo_angle(
            angle=np.asarray(q, dtype=float).tolist(),
            speed=speed,
            mvacc=None,
            mvtime=None,
            wait=wait,
        )

    def command_servo(self, q: np.ndarray | list[float], wait: bool = False) -> int:
        return self.arm.set_servo_angle_j(
            angles=np.asarray(q, dtype=float).tolist(),
            wait=wait,
        )

    def _record_loop(self) -> None:
        period_s = 1.0 / self._logging_data_frequency_hz
        next_tick_s = time.perf_counter()
        try:
            while not self._record_stop.is_set():
                state = self.get_state()
                with self._record_lock:
                    self._recording.append(state)
                next_tick_s += period_s
                sleep_s = next_tick_s - time.perf_counter()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        except BaseException as exc:
            self._thread_error = exc

    def start_recording(self) -> None:
        if self._record_thread is not None and self._record_thread.is_alive():
            return
        self._thread_error = None
        self._record_stop.clear()
        with self._record_lock:
            self._recording = ArmStateTraj()
        self._record_thread = threading.Thread(
            target=self._record_loop,
            name="arm-state-recorder",
            daemon=True,
        )
        self._record_thread.start()

    def stop_recording(self) -> None:
        self._record_stop.set()
        if self._record_thread is not None:
            self._record_thread.join(timeout=2.0)
            self._record_thread = None
        if self._thread_error is not None:
            raise RuntimeError("Arm recorder thread failed") from self._thread_error

    def get_recording(self) -> ArmStateTraj:
        with self._record_lock:
            traj = ArmStateTraj(maxlen=self._recording.states.maxlen or int(1e6))
            traj.extend(list(self._recording.states))
        return traj
