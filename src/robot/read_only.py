"""Read-only Yaskawa discovery and feedback-stream lifecycle.

The ACU-side bridge owns generated vendor bindings. This module owns the
Reforge-side lifecycle, validation, state cache, timestamp policy, and derived
velocity. It has no motion methods and cannot power servos or publish targets.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol

from .contract import (
    DEFAULT_GROUP_NO,
    DEFAULT_TOOL_NO,
    MAX_MONITOR_RATE_HZ,
    DiscoveryReport,
    ModeReport,
    YaskawaContractError,
    discover_controller,
    parse_feedback_sample,
    parse_mode_response,
)


class YaskawaReadOnlyError(RuntimeError):
    """Raised when the read-only client cannot provide trustworthy state."""


@dataclass(frozen=True, slots=True)
class FeedbackResponses:
    """Synchronized response set emitted by the thin ACU bridge.

    Args:
        joints: Feedback-axis-position response in vendor field shape.
        torques: Feedback-torque response requested in N-m.
        cartesian: Feedback-Cartesian response requested in the base frame.
    """

    joints: object
    torques: object
    cartesian: object


class YaskawaReadOnlyTransport(Protocol):
    """Transport boundary implemented by the local-only ACU bridge client."""

    def open(self) -> None:
        """Open the local bridge channel without changing controller state."""

    def close(self) -> None:
        """Close the bridge channel and release transport resources."""

    def get_firmware_version(self) -> object:
        """Return one ``GetFirmwareVersion``-shaped response."""

    def get_configuration(self) -> object:
        """Return one ``GetConfiguration``-shaped response."""

    def get_group_properties(self, group_no: int) -> object:
        """Return properties for the requested controller group."""

    def get_mode(self) -> object:
        """Return one read-only ``GetMode``-shaped response."""

    def stream_feedback(
        self,
        *,
        group_no: int,
        tool_no: int,
        rate_hz: float,
        stop_event: threading.Event,
    ) -> Iterable[FeedbackResponses]:
        """Yield synchronized joint, torque, and Cartesian response sets."""

    def stop_feedback(self) -> None:
        """Interrupt a blocking feedback stream without changing robot state."""


@dataclass(frozen=True, slots=True)
class ReadOnlyState:
    """One accepted Yaskawa state and its timing metadata.

    ``aligned_unix_s`` and ``joint_velocities_rad_s`` are unavailable until a
    controller tick period is supplied. The first sample after connection or a
    timing gap has zero derived velocity and a non-``None`` reset reason.

    Args:
        joint_positions_rad: Enabled joint positions [rad].
        joint_velocities_rad_s: Derived joint velocities [rad/s], if configured.
        joint_torques_nm: Enabled feedback torques [N-m].
        tcp_pose: Base-frame TCP pose ``[x, y, z, qx, qy, qz, qw]`` [m, -].
        controller_timestamp: ACU uptime tick.
        received_unix_s: Host Unix time when the sample reached this process [s].
        aligned_unix_s: Tick-derived Unix time anchored at the first sample [s].
        received_monotonic_s: Host monotonic receipt time used for staleness [s].
        velocity_reset_reason: Reason velocity was reset to zero, when applicable.
    """

    joint_positions_rad: tuple[float, ...]
    joint_velocities_rad_s: tuple[float, ...] | None
    joint_torques_nm: tuple[float, ...]
    tcp_pose: tuple[float, float, float, float, float, float, float]
    controller_timestamp: int
    received_unix_s: float
    aligned_unix_s: float | None
    received_monotonic_s: float
    velocity_reset_reason: str | None


@dataclass(frozen=True, slots=True)
class AcquisitionStatus:
    """Snapshot of read-only stream health and rejection counters."""

    connected: bool
    streaming: bool
    accepted_samples: int
    reordered_samples: int
    last_controller_timestamp: int | None
    error: str | None


class YaskawaReadOnlyClient:
    """Own discovery, stream validation, bounded caching, and clean shutdown.

    Args:
        transport: Injected local bridge transport.
        group_no: Controller group number to validate and monitor.
        tool_no: Tool number used for base-frame Cartesian feedback.
        rate_hz: Requested feedback rate [Hz], capped by the vendor contract.
        controller_tick_period_s: Measured duration of one ACU timestamp tick
            [s]. Leave unset until the no-motion preflight establishes it.
        stale_after_s: Maximum host receipt age before cached state is rejected
            [s]. Defaults to five requested sample periods with a 100 ms floor.
        cache_size: Maximum accepted samples retained in memory.

    Raises:
        ValueError: If configuration values are outside the safe contract.
    """

    def __init__(
        self,
        transport: YaskawaReadOnlyTransport,
        *,
        group_no: int = DEFAULT_GROUP_NO,
        tool_no: int = DEFAULT_TOOL_NO,
        rate_hz: float = 25.0,
        controller_tick_period_s: float | None = None,
        stale_after_s: float | None = None,
        cache_size: int = 1024,
    ) -> None:
        """Initialize an inert client; call :meth:`connect` to open transport."""

        if group_no < 0 or tool_no < 0:
            raise ValueError("group_no and tool_no must be non-negative.")
        if not math.isfinite(rate_hz) or not 0.0 < rate_hz <= MAX_MONITOR_RATE_HZ:
            raise ValueError(
                f"rate_hz must be finite and in (0, {MAX_MONITOR_RATE_HZ}]."
            )
        if controller_tick_period_s is not None and (
            not math.isfinite(controller_tick_period_s)
            or controller_tick_period_s <= 0.0
        ):
            raise ValueError("controller_tick_period_s must be finite and positive.")
        resolved_stale_after_s = (
            max(0.1, 5.0 / rate_hz) if stale_after_s is None else stale_after_s
        )
        if not math.isfinite(resolved_stale_after_s) or resolved_stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be finite and positive.")
        if cache_size <= 0:
            raise ValueError("cache_size must be positive.")

        self._transport = transport
        self._group_no = group_no
        self._tool_no = tool_no
        self._rate_hz = float(rate_hz)
        self._controller_tick_period_s = controller_tick_period_s
        self._stale_after_s = float(resolved_stale_after_s)
        self._states: deque[ReadOnlyState] = deque(maxlen=cache_size)
        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._connected = False
        self._error: BaseException | None = None
        self._reordered_samples = 0
        self._discovery: DiscoveryReport | None = None
        self._mode: ModeReport | None = None
        self._clock_anchor: tuple[int, float] | None = None

    @property
    def discovery(self) -> DiscoveryReport:
        """Return validated discovery data after connection.

        Raises:
            YaskawaReadOnlyError: If the client has not connected successfully.
        """

        if self._discovery is None:
            raise YaskawaReadOnlyError(
                "Yaskawa discovery is unavailable before connect()."
            )
        return self._discovery

    @property
    def mode(self) -> ModeReport:
        """Return the mode observed during discovery.

        Raises:
            YaskawaReadOnlyError: If the client has not connected successfully.
        """

        if self._mode is None:
            raise YaskawaReadOnlyError("Yaskawa mode is unavailable before connect().")
        return self._mode

    def refresh_mode(self) -> ModeReport:
        """Read and cache current Teach/Play/Remote state without motion.

        Returns:
            Fresh mode observation from ``ModeGetService``.

        Raises:
            YaskawaReadOnlyError: If the client is not connected.
            YaskawaContractError: If the response is malformed or unsuccessful.
        """

        if not self._connected:
            raise YaskawaReadOnlyError("connect() must succeed before refresh_mode().")
        mode = parse_mode_response(self._transport.get_mode())
        with self._condition:
            self._mode = mode
        return mode

    def connect(self) -> DiscoveryReport:
        """Open the bridge and perform fail-closed system discovery.

        Returns:
            Validated controller/group discovery report.

        Raises:
            YaskawaContractError: If the bridge returns an unsupported contract.
            RuntimeError: If transport setup or an RPC fails.
        """

        if self._connected:
            return self.discovery
        self._transport.open()
        try:
            discovery = discover_controller(
                self._transport.get_firmware_version(),
                self._transport.get_configuration(),
                self._transport.get_group_properties(self._group_no),
                group_no=self._group_no,
            )
            mode = parse_mode_response(self._transport.get_mode())
        except BaseException:
            self._transport.close()
            raise
        self._discovery = discovery
        self._mode = mode
        self._connected = True
        return discovery

    def start(self) -> None:
        """Start the background feedback stream after successful discovery.

        Raises:
            YaskawaReadOnlyError: If not connected or already streaming.
        """

        if not self._connected:
            raise YaskawaReadOnlyError("connect() must succeed before start().")
        if self._thread is not None and self._thread.is_alive():
            raise YaskawaReadOnlyError("Yaskawa feedback stream is already running.")
        with self._condition:
            self._states.clear()
            self._error = None
            self._reordered_samples = 0
            self._clock_anchor = None
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._stream_loop,
            name="yaskawa-read-only-feedback",
            daemon=True,
        )
        self._thread.start()

    def _stream_loop(self) -> None:
        """Consume, validate, and cache bridge samples until stopped or failed."""

        try:
            responses = self._transport.stream_feedback(
                group_no=self._group_no,
                tool_no=self._tool_no,
                rate_hz=self._rate_hz,
                stop_event=self._stop_event,
            )
            for response_set in responses:
                if self._stop_event.is_set():
                    break
                self._accept(response_set)
            if not self._stop_event.is_set():
                raise YaskawaReadOnlyError(
                    "Yaskawa feedback stream ended unexpectedly."
                )
        except BaseException as exc:  # noqa: BLE001
            if not self._stop_event.is_set():
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()

    def _accept(self, response_set: FeedbackResponses) -> None:
        """Validate one synchronized response set and append accepted state."""

        feedback = parse_feedback_sample(
            response_set.joints,
            response_set.torques,
            response_set.cartesian,
            self.discovery,
        )
        received_monotonic_s = time.monotonic()
        received_unix_s = time.time()
        with self._condition:
            previous = self._states[-1] if self._states else None
            if (
                previous is not None
                and feedback.controller_timestamp <= previous.controller_timestamp
            ):
                self._reordered_samples += 1
                return
            velocities, reset_reason = self._derive_velocities(
                feedback.joint_positions_rad,
                feedback.controller_timestamp,
                previous,
            )
            if self._clock_anchor is None:
                self._clock_anchor = (feedback.controller_timestamp, received_unix_s)
            aligned_unix_s = None
            if self._controller_tick_period_s is not None:
                anchor_tick, anchor_unix_s = self._clock_anchor
                aligned_unix_s = (
                    anchor_unix_s
                    + (feedback.controller_timestamp - anchor_tick)
                    * self._controller_tick_period_s
                )
            self._states.append(
                ReadOnlyState(
                    joint_positions_rad=feedback.joint_positions_rad,
                    joint_velocities_rad_s=velocities,
                    joint_torques_nm=feedback.joint_torques_nm,
                    tcp_pose=feedback.tcp_pose,
                    controller_timestamp=feedback.controller_timestamp,
                    received_unix_s=received_unix_s,
                    aligned_unix_s=aligned_unix_s,
                    received_monotonic_s=received_monotonic_s,
                    velocity_reset_reason=reset_reason,
                )
            )
            self._condition.notify_all()

    def _derive_velocities(
        self,
        positions_rad: tuple[float, ...],
        controller_timestamp: int,
        previous: ReadOnlyState | None,
    ) -> tuple[tuple[float, ...] | None, str | None]:
        """Derive bounded velocity from controller ticks with reset semantics."""

        if self._controller_tick_period_s is None:
            return None, "controller_tick_period_unavailable"
        zeros = tuple(0.0 for _ in positions_rad)
        if previous is None:
            return zeros, "first_sample"
        elapsed_s = (
            controller_timestamp - previous.controller_timestamp
        ) * self._controller_tick_period_s
        if elapsed_s > self._stale_after_s:
            return zeros, "controller_timestamp_gap"
        deltas = tuple(
            math.remainder(current - prior, 2.0 * math.pi)
            for current, prior in zip(positions_rad, previous.joint_positions_rad)
        )
        velocities = tuple(delta / elapsed_s for delta in deltas)
        for velocity, axis in zip(velocities, self.discovery.axis_specs):
            if abs(velocity) > axis.max_speed_rad_s * 1.05:
                raise YaskawaContractError(
                    "Derived feedback velocity exceeds the discovered axis limit."
                )
        return velocities, None

    def latest(self, *, timeout_s: float = 0.0) -> ReadOnlyState:
        """Return the newest non-stale state, optionally waiting for the first.

        Args:
            timeout_s: Maximum wait for an initial sample [s]. Zero does not wait.

        Returns:
            Latest immutable accepted state.

        Raises:
            ValueError: If ``timeout_s`` is negative or non-finite.
            YaskawaReadOnlyError: If acquisition failed, timed out, or is stale.
        """

        if not math.isfinite(timeout_s) or timeout_s < 0.0:
            raise ValueError("timeout_s must be finite and non-negative.")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not self._states and self._error is None:
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    break
                self._condition.wait(remaining_s)
            if self._error is not None:
                raise YaskawaReadOnlyError(
                    f"Yaskawa feedback acquisition failed: {self._error}"
                ) from self._error
            if not self._states:
                raise YaskawaReadOnlyError("No Yaskawa feedback sample is available.")
            state = self._states[-1]
        age_s = time.monotonic() - state.received_monotonic_s
        if age_s > self._stale_after_s:
            raise YaskawaReadOnlyError(
                f"Latest Yaskawa feedback is stale ({age_s:.3f} s old; "
                f"limit {self._stale_after_s:.3f} s)."
            )
        return state

    def cached_states(self) -> tuple[ReadOnlyState, ...]:
        """Return an immutable snapshot of the bounded accepted-state cache."""

        with self._condition:
            return tuple(self._states)

    def wait_for_samples(self, sample_count: int, *, timeout_s: float) -> None:
        """Wait until the cache contains a requested number of accepted samples.

        Args:
            sample_count: Required accepted sample count.
            timeout_s: Maximum wait [s].

        Raises:
            ValueError: If bounds are invalid or exceed cache capacity.
            YaskawaReadOnlyError: If acquisition fails or the wait times out.
        """

        if sample_count <= 0:
            raise ValueError("sample_count must be positive.")
        if self._states.maxlen is not None and sample_count > self._states.maxlen:
            raise ValueError("sample_count exceeds the configured cache size.")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive.")
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while len(self._states) < sample_count and self._error is None:
                remaining_s = deadline - time.monotonic()
                if remaining_s <= 0.0:
                    break
                self._condition.wait(remaining_s)
            if self._error is not None:
                raise YaskawaReadOnlyError(
                    f"Yaskawa feedback acquisition failed: {self._error}"
                ) from self._error
            if len(self._states) < sample_count:
                raise YaskawaReadOnlyError(
                    f"Timed out after {timeout_s:.3f} s waiting for "
                    f"{sample_count} feedback samples."
                )

    def status(self) -> AcquisitionStatus:
        """Return current lifecycle, sample, rejection, and error status."""

        with self._condition:
            streaming = self._thread is not None and self._thread.is_alive()
            last_timestamp = (
                self._states[-1].controller_timestamp if self._states else None
            )
            return AcquisitionStatus(
                connected=self._connected,
                streaming=streaming,
                accepted_samples=len(self._states),
                reordered_samples=self._reordered_samples,
                last_controller_timestamp=last_timestamp,
                error=str(self._error) if self._error is not None else None,
            )

    def close(self) -> None:
        """Stop acquisition and close the bridge without issuing controller RPCs."""

        self._stop_event.set()
        thread = self._thread
        shutdown_error: YaskawaReadOnlyError | None = None
        if thread is not None and thread.is_alive():
            try:
                self._transport.stop_feedback()
            except BaseException as exc:  # noqa: BLE001
                shutdown_error = YaskawaReadOnlyError(
                    f"Failed to interrupt the Yaskawa feedback stream: {exc}"
                )
            else:
                thread.join(timeout=2.0)
                if thread.is_alive():
                    shutdown_error = YaskawaReadOnlyError(
                        "Yaskawa feedback stream did not stop within 2 seconds."
                    )
        self._thread = None
        try:
            if self._connected:
                self._transport.close()
        finally:
            self._connected = False
        if shutdown_error is not None:
            raise shutdown_error

    def __enter__(self) -> YaskawaReadOnlyClient:
        """Connect and return this client for context-managed use."""

        self.connect()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        """Close the client on context exit."""

        self.close()
