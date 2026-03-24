"""Small xArm client and in-memory recording helpers.

This module intentionally targets the concrete ``xarm.wrapper.XArmAPI`` wrapper
instead of introducing a generic provider abstraction. The goal is to keep the
interface easy to understand and parallel to the IMU-side recording workflow.
"""

from __future__ import annotations

import datetime
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation as R
from xarm.wrapper import XArmAPI


MIN_RECORDING_FREQUENCY_HZ = 1.0
MAX_RECORDING_FREQUENCY_HZ = 250.0


@dataclass(frozen=True, slots=True)
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

    def copy(self) -> ArmState:
        """Return a deep-copied snapshot safe for external callers."""
        return ArmState(
            q=self.q.copy(),
            qd=self.qd.copy(),
            tau=self.tau.copy(),
            ts=float(self.ts),
        )

    def array(self) -> np.ndarray:
        """Return one flat array containing position, velocity, and effort."""
        return np.concatenate([self.q, self.qd, self.tau])


class ArmStateTraj:
    """In-memory trajectory of timestamped arm states."""

    def __init__(self, maxlen: int = int(1e6)) -> None:
        self.states: deque[ArmState] = deque(maxlen=maxlen)

    def append(self, state: ArmState) -> None:
        self.states.append(state)

    def extend(self, states: list[ArmState]) -> None:
        self.states.extend(states)

    def __len__(self) -> int:
        return len(self.states)

    def copy(self) -> ArmStateTraj:
        traj = ArmStateTraj(maxlen=self.states.maxlen or int(1e6))
        traj.extend([state.copy() for state in self.states])
        return traj

    def unpack(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        ts = np.array([state.ts for state in self.states], dtype=float)
        q = np.array([state.q for state in self.states], dtype=float)
        qd = np.array([state.qd for state in self.states], dtype=float)
        tau = np.array([state.tau for state in self.states], dtype=float)
        return ts, q, qd, tau


@dataclass(frozen=True, slots=True)
class ArmAcquisitionStatus:
    """Health/status for arm recording."""

    ok: bool
    running: bool
    message: str
    sample_count: int
    last_sample_ts: float | None


class ArmClient:
    """Small concrete wrapper around ``xarm.wrapper.XArmAPI``.

    The xArm wrapper does not expose source timestamps, so this client computes
    timestamps locally when samples are read. Recording continuously polls the
    wrapper at a fixed rate and appends every sampled state.
    """

    def __init__(
        self,
        robot_ip: str,
        max_recording_len: int = int(1e6),
        recording_data_frequency_hz: float = MAX_RECORDING_FREQUENCY_HZ,
        arm: XArmAPI | None = None,
    ) -> None:
        self.robot_ip = robot_ip
        self._record_thread: threading.Thread | None = None
        self._record_stop = threading.Event()
        self._record_lock = threading.Lock()
        self._latest_state: ArmState | None = None
        self._recording = ArmStateTraj(maxlen=max_recording_len)
        self._record_started_monotonic: float | None = None
        self._record_last_sample_monotonic: float | None = None
        self._record_last_sample_ts: float | None = None
        self._record_sample_count = 0
        self._record_error_message = ""
        self._recording_data_frequency_hz = self.set_recording_data_frequency(
            recording_data_frequency_hz
        )

        self.arm = XArmAPI(robot_ip, is_radian=True) if arm is None else arm

    def _stale_timeout_s(self) -> float:
        return 0.5

    def set_recording_data_frequency(
        self,
        frequency_hz: float | int,
        rounding: Literal["up", "down", "nearest"] = "up",
    ) -> float:
        """Set the fixed recording frequency used by ``start_recording()``.

        Numeric input is accepted directly and clamped into the supported
        continuous interval ``[MIN_RECORDING_FREQUENCY_HZ, MAX_RECORDING_FREQUENCY_HZ]``.
        The ``rounding`` argument is accepted for interface parity with the IMU
        client, but because the arm client supports a continuous range rather
        than a discrete enum set, all three modes resolve to clamping.
        """
        requested_hz = float(frequency_hz)
        if not np.isfinite(requested_hz):
            raise ValueError(f"Invalid frequency value: {frequency_hz!r}.")
        if rounding not in {"up", "down", "nearest"}:
            raise ValueError(
                f"Unsupported rounding mode: {rounding!r}. "
                "Use 'up', 'down', or 'nearest'."
            )

        resolved_hz = min(
            max(requested_hz, MIN_RECORDING_FREQUENCY_HZ),
            MAX_RECORDING_FREQUENCY_HZ,
        )
        self._recording_data_frequency_hz = float(resolved_hz)
        return self._recording_data_frequency_hz

    def get_recording_data_frequency(self) -> float:
        """Get the currently configured recording frequency in Hz."""
        return float(self._recording_data_frequency_hz)

    def get_supported_recording_data_frequency_range_hz(self) -> tuple[float, float]:
        """Return the supported continuous recording-frequency interval in Hz."""
        return (MIN_RECORDING_FREQUENCY_HZ, MAX_RECORDING_FREQUENCY_HZ)

    def _record_status_locked(self) -> ArmAcquisitionStatus:
        running = (
            self._record_thread is not None
            and self._record_thread.is_alive()
            and not self._record_stop.is_set()
        )
        ok = True
        message = self._record_error_message
        if not running:
            ok = False
            if not message:
                message = "Recording is not active."
        else:
            reference_mark = (
                self._record_last_sample_monotonic
                if self._record_last_sample_monotonic is not None
                else self._record_started_monotonic
            )
            if reference_mark is not None and (
                time.monotonic() - reference_mark
            ) > self._stale_timeout_s():
                ok = False
                if not message:
                    message = (
                        "No arm samples have arrived within the stale-data timeout."
                    )
            elif message:
                ok = False

        return ArmAcquisitionStatus(
            ok=ok,
            running=running,
            message=message or ("ok" if ok else "Recording is not active."),
            sample_count=self._record_sample_count,
            last_sample_ts=self._record_last_sample_ts,
        )

    def get_state(self) -> ArmState:
        """Read one arm state from the xArm wrapper as a safe copied snapshot."""
        code, payload = self.arm.get_joint_states()
        if code != 0:
            raise RuntimeError(f"get_joint_states returned code {code}")

        q, qd, tau = payload
        state = ArmState(
            ts=datetime.datetime.now(datetime.timezone.utc).timestamp(),
            q=np.asarray(q, dtype=float),
            qd=np.asarray(qd, dtype=float),
            tau=np.asarray(tau, dtype=float),
        )
        with self._record_lock:
            self._latest_state = state
        return state.copy()

    def get_latest_state(self) -> ArmState:
        """Return a safe copy of the cached latest arm state."""
        with self._record_lock:
            latest_state = self._latest_state
        if latest_state is not None:
            return latest_state.copy()
        return self.get_state()

    def get_joint_positions(self) -> list[float]:
        """Return current joint positions in radians."""
        return self.get_state().q.tolist()

    def get_tcp_pose(self) -> list[float]:
        """Return the current TCP pose as ``[x, y, z, qx, qy, qz, qw]``."""
        code, pose = self.arm.get_position_aa()
        if code != 0:
            raise RuntimeError(f"get_position_aa returned code {code}")

        position, axang = pose[:3], pose[3:]
        position_m = [float(coord) / 1000.0 for coord in position]
        if not self.arm._is_radian:
            axang = [np.deg2rad(float(coord)) for coord in axang]
        quat = R.from_rotvec(np.asarray(axang, dtype=float)).as_quat().tolist()
        return [*position_m, *quat]

    def set_position_mode(self, settle_s: float = 1.0) -> None:
        """Set the arm into position mode."""
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(settle_s)

    def set_servo_mode(self, settle_s: float = 1.0) -> None:
        """Set the arm into servo mode."""
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(1)
        self.arm.set_state(0)
        time.sleep(settle_s)

    def move_home(self, settle_s: float = 1.0) -> None:
        """Move the arm to the wrapper's built-in home position."""
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(settle_s)

        result = self.arm.move_gohome(wait=True)
        code = result[0] if isinstance(result, tuple) else result
        if code != 0:
            raise RuntimeError(f"move_gohome(wait=True) returned code {code}")

        time.sleep(settle_s)

    def move_to_joint(self, target_joint: np.ndarray | list[float] | tuple[float, ...]) -> None:
        """Move the arm to one joint target using blocking position control."""
        self.set_position_mode()
        joint_target = np.asarray(target_joint, dtype=float)
        if not self.arm._is_radian:
            joint_target = np.rad2deg(joint_target)

        code = self.arm.set_servo_angle(
            angle=joint_target.tolist(),
            speed=None,
            mvacc=None,
            mvtime=None,
            wait=True,
        )
        if code != 0:
            raise RuntimeError(
                f"set_servo_angle(...) returned code {code} for target {joint_target.tolist()}"
            )

    def move_to_pose(self, target_quat: list[float], target_xyz: list[float]) -> None:
        """Move the arm to one Cartesian target using blocking position control."""
        self.set_position_mode()
        axang = R.from_quat(np.asarray(target_quat, dtype=float)).as_rotvec()
        if not self.arm._is_radian:
            axang = np.rad2deg(axang)
        position_mm = (1000.0 * np.asarray(target_xyz, dtype=float)).tolist()
        pose = [*position_mm, *np.asarray(axang, dtype=float).tolist()]

        code = self.arm.set_position_aa(
            axis_angle_pose=pose,
            speed=None,
            mvacc=None,
            mvtime=None,
            motion_type=1,
            wait=True,
        )
        if code != 0:
            raise RuntimeError(
                f"set_position_aa(...) returned code {code} for target xyz={target_xyz}, quat={target_quat}"
            )

    def command_position(
        self,
        q: np.ndarray | list[float],
        *,
        speed: float = 100.0,
        wait: bool = True,
    ) -> int:
        """Send one position command through the xArm wrapper."""
        return self.arm.set_servo_angle(
            angle=np.asarray(q, dtype=float).tolist(),
            speed=speed,
            mvacc=None,
            mvtime=None,
            wait=wait,
        )

    def command_servo(self, q: np.ndarray | list[float], wait: bool = False) -> int:
        """Send one servo-angle command through the xArm wrapper."""
        return self.arm.set_servo_angle_j(
            angles=np.asarray(q, dtype=float).tolist(),
            wait=wait,
        )

    def _record_loop(self) -> None:
        period_s = 1.0 / self._recording_data_frequency_hz
        next_tick_s = time.perf_counter()
        while not self._record_stop.is_set():
            try:
                state = self.get_state()
            except BaseException as exc:  # noqa: BLE001
                with self._record_lock:
                    if not self._record_error_message:
                        self._record_error_message = (
                            f"Arm state acquisition failed: {exc!r}"
                        )
                return

            now = time.monotonic()
            with self._record_lock:
                self._record_last_sample_monotonic = now
                self._record_last_sample_ts = float(state.ts)
                self._recording.append(state)
                self._record_sample_count += 1

            next_tick_s += period_s
            sleep_s = next_tick_s - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    def start_recording(self) -> None:
        """Start in-memory arm recording from the continuous wrapper poll loop."""
        if self._record_thread is not None and self._record_thread.is_alive():
            return

        self._record_stop.clear()
        with self._record_lock:
            self._recording = ArmStateTraj(maxlen=self._recording.states.maxlen or int(1e6))
            self._record_started_monotonic = time.monotonic()
            self._record_last_sample_monotonic = None
            self._record_last_sample_ts = None
            self._record_sample_count = 0
            self._record_error_message = ""

        self._record_thread = threading.Thread(
            target=self._record_loop,
            name="arm-state-recorder",
            daemon=True,
        )
        self._record_thread.start()

    def check_recording(self) -> ArmAcquisitionStatus:
        """Return current recording health/status."""
        with self._record_lock:
            return self._record_status_locked()

    def stop_recording(self) -> ArmAcquisitionStatus:
        """Stop recording and return the final recording status."""
        with self._record_lock:
            final_status = self._record_status_locked()
        self._record_stop.set()
        if self._record_thread is not None:
            self._record_thread.join(timeout=2.0)
            self._record_thread = None
        return ArmAcquisitionStatus(
            ok=final_status.ok,
            running=False,
            message=final_status.message,
            sample_count=final_status.sample_count,
            last_sample_ts=final_status.last_sample_ts,
        )

    def get_recording(self) -> ArmStateTraj:
        """Return a deep-copied trajectory safe for external callers."""
        with self._record_lock:
            return self._recording.copy()
