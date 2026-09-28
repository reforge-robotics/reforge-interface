"""Software-qualified Yaskawa motion lifecycle and command validation.

The ACU SDK bindings remain behind an injected transport.  This module owns
the Reforge-side safety boundary and uses the names and shapes of the supplied
Robot Control Service contract: ``SendJointMotionTarget``/
``ReceiveTarget``, the empty-request motion controls, and
``StartIncrementMove``/``SetIncrementMove``/``StopIncrementMove``.  The SDK
does not expose a servo-state getter.  Live callers therefore provide an
operator-controlled servo-power confirmation; no servo-power RPC is present
in this boundary and no constructor changes controller state.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Protocol

from .contract import ControllerMode, DiscoveryReport, require_success
from .read_only import ReadOnlyState, YaskawaReadOnlyClient, YaskawaReadOnlyError


class YaskawaMotionError(RuntimeError):
    """Raised when the controller cannot safely execute a motion command."""


class YaskawaMotionPrerequisiteError(YaskawaMotionError):
    """Raised when mode, Remote, or operator-confirmed servo readiness is absent."""


class YaskawaMotionTimeout(YaskawaMotionError):
    """Raised when queued motion does not complete before its deadline."""


class MotionState(StrEnum):
    """Normalized target completion states exposed by the motion client."""

    COMPLETE = "complete"
    INTERRUPTED = "interrupted"
    UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True)
class IncrementMoveRequest:
    """Describe one exact ``IncrementMoveGroupRequest`` angle command.

    ``angle_degrees`` contains one per-axis increment in controller degrees,
    not an absolute position.  The ACU bridge is responsible for placing it
    in the protobuf ``angle.pos`` field.

    Args:
        group_no: ACU control-group number.
        tool_no: ACU tool number.  Joint-angle increments still carry the tool
            field because it is part of the vendor request message.
        angle_degrees: Per-axis incremental angles [deg].
    """

    group_no: int
    tool_no: int
    angle_degrees: tuple[float, ...]

    @property
    def angle(self) -> tuple[float, ...]:
        """Return the degree values that map to protobuf ``angle.pos``."""

        return self.angle_degrees


class YaskawaMotionTransport(Protocol):
    """Document the exact motion calls supplied by the ACU bridge.

    This structural protocol is implemented by the generated client for the
    versioned, Reforge-owned ACU bridge contract. Implementations must preserve
    vendor response ``status`` fields, controller timestamps, and nested
    ``result.target_id``/``task_no`` values.
    """

    def send_joint_motion_target(
        self,
        *,
        group_no: int,
        target_degrees: Sequence[float],
        tool_no: int,
        speed_percent: float,
        acceleration_percent: float,
        deceleration_percent: float,
        timeout: int,
    ) -> object:
        """Send one absolute ANGLE target and return ``result.target_id``."""

        raise NotImplementedError

    def receive_target(self, *, group_no: int, target_id: int, timeout: int) -> object:
        """Wait for one target and return ``received_target_status``."""

        raise NotImplementedError

    def clear_target(self, *, control_group_bit: int) -> object:
        """Clear every queued target for the exclusively owned control group."""

        raise NotImplementedError

    def start_motion(self) -> object:
        """Call ``StartMotion`` with its empty request."""

        raise NotImplementedError

    def stop_motion(self) -> object:
        """Call ``StopMotion`` with its empty request."""

        raise NotImplementedError

    def abort_motion(self) -> object:
        """Call ``AbortMotion`` with its empty request.

        The vendor documents that this also turns servo power off.  The
        adapter calls it only as an explicit caller-requested or safety-fallback
        stop; it never calls a power-on operation.
        """

        raise NotImplementedError

    def start_increment_move(self, *, control_group_bit: int) -> object:
        """Start increment control and return the allocated ``task_no``."""

        raise NotImplementedError

    def set_increment_move(
        self,
        *,
        task_no: int,
        timeout: int,
        requests: Sequence[IncrementMoveRequest],
    ) -> object:
        """Set one cycle of angle increments for the allocated task."""

        raise NotImplementedError

    def stop_increment_move(self, *, task_no: int) -> object:
        """Stop the incremental task identified by ``task_no``."""

        raise NotImplementedError


@dataclass(frozen=True, slots=True)
class MotionPrerequisites:
    """Snapshot of state required before a motion call.

    Servo power is not part of the supplied read-only ACU contract.  The
    optional value records an explicit operator confirmation when one was
    supplied; ``None`` means that the state is unknown and motion must remain
    blocked.

    Args:
        mode: Current Teach/Play mode.
        remote_enabled: Whether Remote control is enabled.
        servo_power_confirmed: Operator-provided servo-power confirmation, or
            ``None`` when no confirmation source was configured.
    """

    mode: ControllerMode
    remote_enabled: bool
    servo_power_confirmed: bool | None


@dataclass(frozen=True, slots=True)
class QueuedJointCommand:
    """Exact validated joint command sent to the controller.

    Args:
        target_joints_rad: Reforge target positions [rad].
        target_degrees: ACU target positions [deg].
        speed_percent: Controller target speed [%].
        acceleration_percent: Controller acceleration ratio [%].
        deceleration_percent: Controller deceleration ratio [%].
    """

    target_joints_rad: tuple[float, ...]
    target_degrees: tuple[float, ...]
    speed_percent: float
    acceleration_percent: float
    deceleration_percent: float


def _field(response: object, name: str, *, default: object = None) -> object:
    """Read one field from a protobuf-like object or mapping.

    Args:
        response: Mapping or generated-message-like object.
        name: Field name to read.
        default: Value returned when the field is absent.

    Returns:
        The field value or ``default``.
    """

    if isinstance(response, Mapping):
        return response.get(name, default)
    return getattr(response, name, default)


def _raw_status(response: object) -> object:
    """Return a response status or a sentinel for malformed responses."""

    return _field(response, "status", default="<missing>")


def _status_text(response: object, *, operation: str = "") -> str:
    """Normalize a vendor status into a readable lower-case label.

    Args:
        response: ACU response object or mapping.
        operation: RPC operation used to select numeric enum labels.

    Returns:
        A stable status label suitable for error messages.
    """

    status = _raw_status(response)
    if isinstance(status, str):
        text = status.rsplit(".", 1)[-1].lower()
    else:
        name = getattr(status, "name", None)
        text = name.lower() if isinstance(name, str) else str(status).lower()
    if isinstance(status, int) and not isinstance(status, bool):
        if operation == "SendJointMotionTarget":
            labels = {
                0: "unspecified",
                1: "success",
                2: "unknown_error",
                3: "number_of_targets_exceeded",
                4: "group_does_not_exist",
            }
            text = labels.get(status, text)
        elif operation == "ReceiveTarget":
            text = {
                0: "unspecified",
                1: "success",
                2: "unknown_error",
                3: "group_does_not_exist",
                4: "target_receive_timeout",
            }.get(status, text)
        elif operation == "SetIncrementMove":
            text = {
                0: "unspecified",
                1: "success",
                2: "unknown_error",
                8: "group_does_not_ready",
                16: "servo_power_not_enabled",
                23: "timeout",
            }.get(status, text)
    return text


def _require_motion_success(response: object, operation: str) -> None:
    """Require explicit ACU success and preserve actionable status details.

    Args:
        response: ACU response object or mapping.
        operation: RPC operation name.

    Raises:
        YaskawaMotionError: If the response is malformed or not successful.
    """

    try:
        require_success(response, operation)
    except Exception as exc:
        status = _status_text(response, operation=operation)
        if any(
            token in status
            for token in (
                "number_of_targets_exceeded",
                "queue",
                "buffer_no_space",
            )
        ):
            raise YaskawaMotionError(
                f"Yaskawa {operation} rejected because the motion target queue is "
                f"full (status={status})."
            ) from exc
        if "servo_power_not_enabled" in status:
            raise YaskawaMotionPrerequisiteError(
                "Yaskawa controller rejected the command because servo power is "
                "not enabled; enable it manually and start a new session."
            ) from exc
        raise YaskawaMotionError(
            f"Yaskawa {operation} failed with status={status}."
        ) from exc


def _finite_values(
    values: Sequence[float], expected: int, name: str
) -> tuple[float, ...]:
    """Validate a finite fixed-length numeric vector.

    Args:
        values: Candidate numeric sequence.
        expected: Required sequence length.
        name: Value name used in errors.

    Returns:
        A tuple of finite floats.

    Raises:
        ValueError: If the sequence is malformed, wrong-sized, or non-finite.
    """

    if isinstance(values, (str, bytes, bytearray)):
        raise ValueError(f"{name} must be a sequence of {expected} finite values.")
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain numeric values.") from exc
    if len(result) != expected:
        raise ValueError(f"{name} must contain exactly {expected} values.")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain only finite values.")
    return result


def _target_id(response: object) -> int:
    """Extract the required nested ``SendTargetResult.target_id``.

    Args:
        response: ``SendJointMotionTargetResponse``-shaped object.

    Returns:
        Non-negative target identifier.

    Raises:
        YaskawaMotionError: If the successful response omits the identifier.
    """

    result = _field(response, "result", default=None)
    value = _field(result, "target_id", default=None) if result is not None else None
    # A bridge may flatten this one field while adapting generated messages;
    # accept that representation only as a compatibility detail.
    if value is None:
        value = _field(response, "target_id", default=None)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise YaskawaMotionError(
            "Yaskawa SendJointMotionTarget succeeded without a valid result.target_id."
        )
    return value


def _task_no(response: object) -> int:
    """Extract the allocated ``StartIncrementMoveResponse.task_no``.

    Args:
        response: ``StartIncrementMoveResponse``-shaped object.

    Returns:
        Non-negative task number.

    Raises:
        YaskawaMotionError: If the successful response omits the task number.
    """

    value = _field(response, "task_no", default=None)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise YaskawaMotionError(
            "Yaskawa StartIncrementMove succeeded without a valid task_no; "
            "no task_no was returned."
        )
    return value


def _target_state(response: object) -> MotionState:
    """Normalize ``received_target_status`` into a finite state vocabulary.

    Args:
        response: ``ReceiveTargetResponse``-shaped object.

    Returns:
        ``COMPLETE``, ``INTERRUPTED``, or ``UNKNOWN``.
    """

    raw = _field(response, "received_target_status", default=None)
    if isinstance(raw, int) and not isinstance(raw, bool):
        return {1: MotionState.COMPLETE, 2: MotionState.INTERRUPTED}.get(
            raw, MotionState.UNKNOWN
        )
    text = str(getattr(raw, "name", raw)).rsplit(".", 1)[-1].lower()
    if "complete" in text:
        return MotionState.COMPLETE
    if "interrupt" in text:
        return MotionState.INTERRUPTED
    return MotionState.UNKNOWN


def _is_target_timeout(response: object) -> bool:
    """Return whether a ReceiveTarget response reports its documented timeout."""

    raw = _raw_status(response)
    if isinstance(raw, int) and not isinstance(raw, bool):
        return raw == 4
    return "target_receive_timeout" in _status_text(response, operation="ReceiveTarget")


def _timeout_milliseconds(value_s: float, name: str) -> int:
    """Convert a positive duration to a bounded ACU timeout in milliseconds.

    Args:
        value_s: Duration in seconds.
        name: Configuration name used in errors.

    Returns:
        A positive integer in the vendor's documented timeout range.

    Raises:
        ValueError: If the value is non-finite, non-positive, or out of range.
    """

    if not math.isfinite(value_s) or value_s <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")
    value_ms = math.ceil(value_s * 1000.0)
    if value_ms > 268_435_455:
        raise ValueError(f"{name} exceeds the ACU timeout range.")
    return max(1, value_ms)


class YaskawaMotionClient:
    """Own queued and incremental Yaskawa motion over an exact ACU boundary.

    The client never changes controller mode or servo power.  The supplied
    SDK exposes no servo-state getter, so actual commands require a callable
    operator/application confirmation that servo power is already enabled.
    Any missing or false confirmation, mode change, feedback fault, timing
    overrun, or transport failure fails closed and invalidates incremental
    ownership.

    Args:
        state_client: Connected and streaming read-only state client.
        transport: Injected exact ACU motion transport.
        cycle_period_s: Incremental command period [s].
        max_command_interval_s: Maximum host interval between commands [s].
        max_motion_timeout_s: Upper bound for target receive/send waits [s].
        servo_power_confirmation: Operator-controlled callable returning
            ``True`` only when servo power is known to be enabled.  ``None``
            deliberately blocks live commands because the vendor has no getter.
        increment_timeout_s: ACU SetIncrementMove timeout [s]. Defaults to one
            configured incremental cycle.

    Raises:
        ValueError: If timing configuration is invalid.
    """

    def __init__(
        self,
        state_client: YaskawaReadOnlyClient,
        transport: YaskawaMotionTransport,
        *,
        cycle_period_s: float = 0.01,
        max_command_interval_s: float | None = None,
        max_motion_timeout_s: float = 30.0,
        servo_power_confirmation: Callable[[], bool] | None = None,
        increment_timeout_s: float | None = None,
    ) -> None:
        """Configure an inert motion client around an existing state client."""

        for value, name in (
            (cycle_period_s, "cycle_period_s"),
            (max_motion_timeout_s, "max_motion_timeout_s"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
        interval = (
            2.0 * cycle_period_s
            if max_command_interval_s is None
            else max_command_interval_s
        )
        if not math.isfinite(interval) or interval <= 0.0:
            raise ValueError("max_command_interval_s must be finite and positive.")
        if servo_power_confirmation is not None and not callable(
            servo_power_confirmation
        ):
            raise TypeError("servo_power_confirmation must be callable or None.")
        increment_timeout = (
            cycle_period_s if increment_timeout_s is None else increment_timeout_s
        )
        increment_timeout_ms = _timeout_milliseconds(
            increment_timeout, "increment_timeout_s"
        )
        self.state_client = state_client
        self._transport = transport
        self._cycle_period_s = float(cycle_period_s)
        self._max_command_interval_s = float(interval)
        self._max_motion_timeout_s = float(max_motion_timeout_s)
        self._motion_timeout_ms = _timeout_milliseconds(
            self._max_motion_timeout_s, "max_motion_timeout_s"
        )
        self._increment_timeout_ms = increment_timeout_ms
        self._servo_power_confirmation = servo_power_confirmation
        self._increment_task_no: int | None = None
        self._last_target_rad: tuple[float, ...] | None = None
        self._last_command_monotonic_s: float | None = None
        self._active_motion = False
        self._active_target_id: int | None = None

    @property
    def discovery(self) -> DiscoveryReport:
        """Return validated controller discovery from the state client."""

        return self.state_client.discovery

    @property
    def session_active(self) -> bool:
        """Return whether an incremental task currently owns a handle."""

        return self._increment_task_no is not None

    def _servo_power_status(self, *, required: bool) -> bool | None:
        """Read the explicit operator confirmation without contacting a getter.

        Args:
            required: Whether an absent/false confirmation should raise.

        Returns:
            ``True``/``False`` from the configured confirmation callback, or
            ``None`` when no callback exists and ``required`` is false.

        Raises:
            YaskawaMotionPrerequisiteError: If required confirmation is absent
                or false.
        """

        if self._servo_power_confirmation is None:
            if required:
                raise YaskawaMotionPrerequisiteError(
                    "Yaskawa servo power state is not exposed by the ACU SDK; "
                    "provide explicit operator confirmation before motion."
                )
            return None
        try:
            confirmed = bool(self._servo_power_confirmation())
        except Exception as exc:
            raise YaskawaMotionPrerequisiteError(
                "Yaskawa servo-power confirmation could not be read."
            ) from exc
        if required and not confirmed:
            raise YaskawaMotionPrerequisiteError(
                "Yaskawa motion requires servo power to be enabled by the operator."
            )
        return confirmed

    def _prerequisites(
        self, *, require_servo_power: bool = True
    ) -> MotionPrerequisites:
        """Read mode/Remote and explicit servo readiness, failing closed.

        Args:
            require_servo_power: Whether this operation will publish motion.

        Returns:
            Immutable prerequisite snapshot.

        Raises:
            YaskawaMotionPrerequisiteError: If a required prerequisite is absent.
        """

        try:
            mode = self.state_client.refresh_mode()
        except Exception as exc:
            raise YaskawaMotionPrerequisiteError(
                "Yaskawa motion could not refresh Teach/Play/Remote state."
            ) from exc
        if mode.mode is not ControllerMode.PLAY:
            raise YaskawaMotionPrerequisiteError(
                f"Yaskawa motion requires Play mode; observed {mode.mode.value}."
            )
        if not mode.remote_enabled:
            raise YaskawaMotionPrerequisiteError(
                "Yaskawa motion requires Remote control to be enabled."
            )
        servo_power = self._servo_power_status(required=require_servo_power)
        prerequisites = MotionPrerequisites(
            mode=mode.mode,
            remote_enabled=mode.remote_enabled,
            servo_power_confirmed=servo_power,
        )
        return prerequisites

    def _invalidate_increment_session(self) -> None:
        """Forget a task so a changed prerequisite cannot reuse it."""

        self._increment_task_no = None
        self._last_target_rad = None
        self._last_command_monotonic_s = None

    def _stop_increment_best_effort(self, task_no: int) -> None:
        """Attempt task cleanup while preserving the original failure path.

        Args:
            task_no: Allocated ACU incremental task number.
        """

        try:
            response = self._transport.stop_increment_move(task_no=task_no)
            _require_motion_success(response, "StopIncrementMove")
        except Exception:
            # The vendor invalidates a task automatically on several faults.
            # Cleanup is best effort here; the public operation still fails
            # closed by forgetting the local task number below.
            pass

    def _invalidate_increment_session_with_cleanup(self) -> None:
        """Stop an owned task when possible, then always clear local state."""

        task_no = self._increment_task_no
        if task_no is not None:
            self._stop_increment_best_effort(task_no)
        self._invalidate_increment_session()

    def _require_increment_session(self) -> int:
        """Return an active task number or fail closed."""

        if self._increment_task_no is None:
            raise YaskawaMotionPrerequisiteError(
                "No active Yaskawa incremental-move session; call start_servo_session()."
            )
        return self._increment_task_no

    def _latest_fresh_state(self) -> ReadOnlyState:
        """Return fresh feedback or convert read-only failure to a motion error."""

        try:
            return self.state_client.latest()
        except YaskawaReadOnlyError as exc:
            raise YaskawaMotionError(
                "Yaskawa motion requires fresh feedback; the cached state is stale "
                "or unavailable."
            ) from exc

    def start_servo_session(self) -> None:
        """Validate prerequisites and acquire one fresh incremental task."""

        if self.session_active:
            raise YaskawaMotionError(
                "Yaskawa incremental-move session is already active."
            )
        try:
            self._prerequisites()
            state = self._latest_fresh_state()
            response = self._transport.start_increment_move(
                control_group_bit=1 << self.discovery.group_no
            )
            _require_motion_success(response, "StartIncrementMove")
            task_no = _task_no(response)
            self._increment_task_no = task_no
            self._last_target_rad = tuple(state.joint_positions_rad)
            self._last_command_monotonic_s = None
        except BaseException:
            self._invalidate_increment_session_with_cleanup()
            raise

    def command_servo_j(
        self, target_joints_rad: Sequence[float], *, wait: bool = False
    ) -> int:
        """Convert and publish one bounded absolute joint target.

        Args:
            target_joints_rad: Absolute target positions [rad].
            wait: Accepted for the ArmClient contract; one increment is
                acknowledged by SetIncrementMove and never waits for a path.

        Returns:
            Zero after an explicit successful SetIncrementMove response.

        Raises:
            YaskawaMotionError: If the task, state, timing, limits, or bridge
                response is unsafe.
        """

        del wait
        task_no = self._require_increment_session()
        try:
            self._prerequisites()
            state = self._latest_fresh_state()
            target = _finite_values(
                target_joints_rad, self.discovery.axis_count, "target_joints_rad"
            )
            now = time.monotonic()
            if (
                self._last_command_monotonic_s is not None
                and now - self._last_command_monotonic_s > self._max_command_interval_s
            ):
                raise YaskawaMotionError(
                    "Yaskawa incremental command timing overrun invalidated the session."
                )
            for value, axis in zip(target, self.discovery.axis_specs):
                if not axis.min_position_rad <= value <= axis.max_position_rad:
                    raise YaskawaMotionError(
                        "Yaskawa joint target exceeds the discovered position limits."
                    )
            if self._last_target_rad is None:
                raise YaskawaMotionError(
                    "Yaskawa incremental session has no accepted target state."
                )
            deltas = tuple(
                value - previous
                for value, previous in zip(target, self._last_target_rad)
            )
            feedback_error = tuple(
                value - feedback
                for value, feedback in zip(target, state.joint_positions_rad)
            )
            for delta, error, axis in zip(
                deltas, feedback_error, self.discovery.axis_specs
            ):
                max_delta = axis.max_speed_rad_s * self._cycle_period_s
                if abs(delta) > max_delta + 1.0e-9:
                    raise YaskawaMotionError(
                        "Yaskawa incremental target discontinuity exceeds the "
                        "per-cycle axis speed bound."
                    )
                if abs(error) > max_delta + 1.0e-9:
                    raise YaskawaMotionError(
                        "Yaskawa incremental target is discontinuous from fresh feedback."
                    )
            request = IncrementMoveRequest(
                group_no=self.discovery.group_no,
                tool_no=self.discovery.default_tool_no,
                angle_degrees=tuple(math.degrees(value) for value in deltas),
            )
            response = self._transport.set_increment_move(
                task_no=task_no,
                timeout=self._increment_timeout_ms,
                requests=(request,),
            )
            _require_motion_success(response, "SetIncrementMove")
            self._last_target_rad = target
            self._last_command_monotonic_s = now
            return 0
        except BaseException:
            self._invalidate_increment_session_with_cleanup()
            raise

    def stop_servo_session(self) -> None:
        """Stop the exact incremental task and invalidate it even on errors."""

        task_no = self._increment_task_no
        if task_no is None:
            return
        try:
            response = self._transport.stop_increment_move(task_no=task_no)
            _require_motion_success(response, "StopIncrementMove")
        finally:
            self._invalidate_increment_session()

    def close(self) -> None:
        """Stop active motion and release any incremental task."""

        errors: list[BaseException] = []
        if self._active_motion:
            try:
                self.stop_motion()
            except BaseException as exc:  # noqa: BLE001
                errors.append(exc)
        try:
            self.stop_servo_session()
        except BaseException as exc:  # noqa: BLE001
            errors.append(exc)
        if errors:
            raise YaskawaMotionError(
                "Yaskawa motion cleanup failed: "
                + "; ".join(str(error) for error in errors)
            ) from errors[0]

    def move_j(
        self,
        target_joints_rad: Sequence[float],
        *,
        speed_percent: float = 50.0,
        acceleration_percent: float = 100.0,
        deceleration_percent: float = 100.0,
        wait: bool = True,
        timeout_s: float | None = None,
    ) -> int:
        """Send one absolute ANGLE target and optionally await completion.

        Args:
            target_joints_rad: Absolute target positions [rad].
            speed_percent: Controller joint speed percentage in (0, 100].
            acceleration_percent: Acceleration ratio in [20, 100] [%].
            deceleration_percent: Deceleration ratio in [20, 100] [%].
            wait: Whether to call ``ReceiveTarget`` before returning.
            timeout_s: Send/receive timeout [s], bounded by configuration.

        Returns:
            Zero after target acceptance, or completion when ``wait`` is true.

        Raises:
            YaskawaMotionError: If validation or an ACU operation fails.
            YaskawaMotionTimeout: If target completion times out.
        """

        self._prerequisites()
        self._latest_fresh_state()
        target = _finite_values(
            target_joints_rad, self.discovery.axis_count, "target_joints_rad"
        )
        speed = self._percentage(speed_percent, "speed_percent", minimum=0.0)
        acceleration = self._percentage(
            acceleration_percent, "acceleration_percent", minimum=20.0
        )
        deceleration = self._percentage(
            deceleration_percent, "deceleration_percent", minimum=20.0
        )
        for value, axis in zip(target, self.discovery.axis_specs):
            if not axis.min_position_rad <= value <= axis.max_position_rad:
                raise YaskawaMotionError(
                    "Yaskawa joint target exceeds the discovered position limits."
                )
        command = QueuedJointCommand(
            target_joints_rad=target,
            target_degrees=tuple(math.degrees(value) for value in target),
            speed_percent=speed,
            acceleration_percent=acceleration,
            deceleration_percent=deceleration,
        )
        if self._active_motion:
            raise YaskawaMotionError(
                "A Yaskawa queued motion is already active; stop it before queuing another."
            )
        timeout = self._resolve_motion_timeout(timeout_s)
        self._clear_target_queue()
        target_may_be_queued = False
        try:
            # A transport failure can hide a controller-accepted target, so
            # cleanup must treat the queue as ambiguous before publishing.
            target_may_be_queued = True
            response = self._transport.send_joint_motion_target(
                group_no=self.discovery.group_no,
                target_degrees=command.target_degrees,
                tool_no=self.discovery.default_tool_no,
                speed_percent=command.speed_percent,
                acceleration_percent=command.acceleration_percent,
                deceleration_percent=command.deceleration_percent,
                timeout=timeout,
            )
            _require_motion_success(response, "SendJointMotionTarget")
            # Mark the target active before extracting the id: a malformed
            # successful response may still have queued a target and must be
            # stopped rather than silently abandoned.
            self._active_motion = True
            target_may_be_queued = False
            target_id = _target_id(response)
            self._active_target_id = target_id
            start_response = self._transport.start_motion()
            _require_motion_success(start_response, "StartMotion")
            if wait:
                self._wait_for_completion(target_id, timeout_s)
            return 0
        except BaseException:
            if self._active_motion:
                self._stop_or_abort()
            elif target_may_be_queued:
                self._clear_target_queue()
            raise

    def _resolve_motion_timeout(self, timeout_s: float | None) -> int:
        """Validate and convert one optional motion timeout to ACU milliseconds."""

        timeout = self._max_motion_timeout_s if timeout_s is None else timeout_s
        if (
            not math.isfinite(timeout)
            or timeout <= 0.0
            or timeout > self._max_motion_timeout_s
        ):
            raise ValueError(
                f"timeout_s must be in (0, {self._max_motion_timeout_s}] seconds."
            )
        return _timeout_milliseconds(timeout, "timeout_s")

    def _wait_for_completion(self, target_id: int, timeout_s: float | None) -> None:
        """Receive one target completion and stop safely on any bad result.

        Args:
            target_id: Target identifier returned by SendJointMotionTarget.
            timeout_s: Completion timeout [s].

        Raises:
            YaskawaMotionTimeout: If ReceiveTarget reports its timeout status.
            YaskawaMotionError: If completion is interrupted or malformed.
        """

        timeout = self._resolve_motion_timeout(timeout_s)
        response = self._transport.receive_target(
            group_no=self.discovery.group_no,
            target_id=target_id,
            timeout=timeout,
        )
        if _is_target_timeout(response):
            self._stop_or_abort()
            raise YaskawaMotionTimeout(
                f"Yaskawa queued joint motion timed out after {timeout / 1000.0:.3f} s."
            )
        _require_motion_success(response, "ReceiveTarget")
        state = _target_state(response)
        if state is MotionState.COMPLETE:
            self._active_motion = False
            self._active_target_id = None
            return
        if state is MotionState.INTERRUPTED:
            self._stop_or_abort()
            raise YaskawaMotionError(
                "Yaskawa queued motion was interrupted before target completion."
            )
        raise YaskawaMotionError(
            "Yaskawa ReceiveTarget returned an unknown received_target_status."
        )

    def _stop_or_abort(self) -> None:
        """Stop queued motion, then clear targets so none can execute later."""

        try:
            response = self._transport.stop_motion()
            _require_motion_success(response, "StopMotion")
        except BaseException:
            response = self._transport.abort_motion()
            _require_motion_success(response, "AbortMotion")
        finally:
            self._active_motion = False
            self._active_target_id = None
        self._clear_target_queue()

    def _clear_target_queue(self) -> None:
        """Clear all queued targets for the exclusively owned robot group."""

        response = self._transport.clear_target(
            control_group_bit=1 << self.discovery.group_no
        )
        _require_motion_success(response, "ClearTarget")

    def stop_motion(self, *, abort: bool = False) -> None:
        """Stop or abort queued motion and clear local active state.

        Args:
            abort: Call the vendor AbortMotion RPC directly when true.  The
                vendor documents that AbortMotion also turns servo power off.
        """

        if not self._active_motion:
            return
        try:
            if abort:
                response = self._transport.abort_motion()
                _require_motion_success(response, "AbortMotion")
            else:
                try:
                    response = self._transport.stop_motion()
                    _require_motion_success(response, "StopMotion")
                except BaseException:
                    response = self._transport.abort_motion()
                    _require_motion_success(response, "AbortMotion")
            self._clear_target_queue()
        finally:
            self._active_motion = False
            self._active_target_id = None

    def dry_run_joint_move(
        self,
        target_joints_rad: Sequence[float],
        *,
        speed_percent: float = 50.0,
        acceleration_percent: float = 100.0,
        deceleration_percent: float = 100.0,
    ) -> dict[str, Any]:
        """Validate a queued command and return a no-publication report.

        The report reads only mode/Remote state.  Servo readiness is explicitly
        ``None`` when no operator confirmation callback exists because the ACU
        SDK has no documented getter.

        Args:
            target_joints_rad: Absolute target positions [rad].
            speed_percent: Controller joint speed percentage in (0, 100].
            acceleration_percent: Acceleration ratio in [20, 100] [%].
            deceleration_percent: Deceleration ratio in [20, 100] [%].

        Returns:
            JSON-compatible command, prerequisite, and limit report.
        """

        target = _finite_values(
            target_joints_rad, self.discovery.axis_count, "target_joints_rad"
        )
        speed = self._percentage(speed_percent, "speed_percent", minimum=0.0)
        acceleration = self._percentage(
            acceleration_percent, "acceleration_percent", minimum=20.0
        )
        deceleration = self._percentage(
            deceleration_percent, "deceleration_percent", minimum=20.0
        )
        for value, axis in zip(target, self.discovery.axis_specs):
            if not axis.min_position_rad <= value <= axis.max_position_rad:
                raise YaskawaMotionError(
                    "Yaskawa joint target exceeds the discovered position limits."
                )
        prerequisites = self._prerequisites(require_servo_power=False)
        return {
            "schema_version": "yaskawa-motion-preflight-2",
            "safety": {
                "dry_run": True,
                "motion_published": False,
                "servo_power_changed": False,
            },
            "prerequisites": {
                "mode": prerequisites.mode.value,
                "remote_enabled": prerequisites.remote_enabled,
                "servo_power_confirmed": prerequisites.servo_power_confirmed,
                "servo_power_state_source": "operator_confirmation_or_unknown",
            },
            "command": {
                "kind": "joint_absolute_angle",
                "group_no": self.discovery.group_no,
                "tool_no": self.discovery.default_tool_no,
                "target_joints_rad": list(target),
                "target_degrees": [math.degrees(value) for value in target],
                "speed_percent": speed,
                "acceleration_percent": acceleration,
                "deceleration_percent": deceleration,
            },
            "limits": self._limits_dict(),
        }

    def _limits_dict(self) -> dict[str, object]:
        """Return exact discovered position and speed limits for reports."""

        return {
            "joint_position_rad": [
                [axis.min_position_rad, axis.max_position_rad]
                for axis in self.discovery.axis_specs
            ],
            "joint_speed_rad_s": [
                axis.max_speed_rad_s for axis in self.discovery.axis_specs
            ],
            "increment_cycle_period_s": self._cycle_period_s,
            "increment_timeout_ms": self._increment_timeout_ms,
        }

    @staticmethod
    def _percentage(value: float, name: str, *, minimum: float) -> float:
        """Validate one controller percentage parameter.

        Args:
            value: Candidate percentage.
            name: Parameter name used in errors.
            minimum: Inclusive minimum, except speed's zero sentinel.

        Returns:
            Finite percentage as a float.

        Raises:
            ValueError: If the percentage is outside the vendor range.
        """

        if not math.isfinite(value) or not minimum <= value <= 100.0:
            raise ValueError(f"{name} must be in [{minimum}, 100] percent.")
        if minimum == 0.0 and value == 0.0:
            raise ValueError(f"{name} must be greater than zero percent.")
        return float(value)


def motion_limits_report(discovery: DiscoveryReport) -> dict[str, object]:
    """Return a serializable no-motion summary of discovered joint limits.

    Args:
        discovery: Validated YNX1000 group discovery.

    Returns:
        JSON-compatible limits and blocked Cartesian-motion explanation.
    """

    return {
        "group_no": discovery.group_no,
        "axis_count": discovery.axis_count,
        "axis_indices": list(discovery.sdk_axis_indices),
        "joint_position_rad": [
            [axis.min_position_rad, axis.max_position_rad]
            for axis in discovery.axis_specs
        ],
        "joint_speed_rad_s": [axis.max_speed_rad_s for axis in discovery.axis_specs],
        "cartesian_motion": {
            "available": False,
            "reason": (
                "Blocked until Yaskawa confirms base/tool/configuration semantics "
                "and the provisional Euler convention with a known-pose sample."
            ),
        },
    }
