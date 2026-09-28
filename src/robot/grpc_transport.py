"""Generated-client transport for the Reforge Yaskawa ACU bridge."""

# mypy: disable-error-code="attr-defined"

from __future__ import annotations

import queue
import threading
import time
from collections.abc import Iterable, Sequence
from typing import Any

import grpc

from .motion import IncrementMoveRequest
from .read_only import FeedbackResponses
from .vendor import reforge_yaskawa_bridge_pb2 as bridge_pb2
from .vendor import reforge_yaskawa_bridge_pb2_grpc as bridge_pb2_grpc

BRIDGE_CONTRACT_MAJOR = 1
BRIDGE_SDK_VERSION = "1.10.0"
DEFAULT_BRIDGE_PORT = 50051
DEFAULT_COMMAND_LEASE_MS = 60_000
DEFAULT_RPC_TIMEOUT_S = 10.0
MAX_PENDING_TIMESTAMPS = 128


class YaskawaGrpcTransportError(RuntimeError):
    """Raised when the local ACU bridge cannot satisfy its frozen contract."""


class YaskawaGrpcTransport:
    """Adapt the generated bridge stub to the state and motion protocols.

    The endpoint must be the locally forwarded bridge address. The ACU bridge
    itself accepts loopback connections only; establishing an approved tunnel
    or controller-network forward remains a deployment prerequisite.

    Args:
        endpoint: Local bridge endpoint as ``host`` or ``host:port``.
        connect_timeout_s: Channel readiness and ordinary RPC timeout [s].
        command_lease_ms: Exclusive motion-session lease [ms]. The session is
            opened lazily by the first mutating call, never by construction or
            read-only connection checks.

    Raises:
        ValueError: If endpoint or timing configuration is invalid.
    """

    def __init__(
        self,
        endpoint: str,
        *,
        connect_timeout_s: float = DEFAULT_RPC_TIMEOUT_S,
        command_lease_ms: int = DEFAULT_COMMAND_LEASE_MS,
    ) -> None:
        """Configure an inert generated client without opening a channel."""

        endpoint = endpoint.strip()
        if not endpoint:
            raise ValueError("Yaskawa bridge endpoint must not be empty.")
        if ":" not in endpoint.rsplit("]", 1)[-1]:
            endpoint = f"{endpoint}:{DEFAULT_BRIDGE_PORT}"
        if connect_timeout_s <= 0.0:
            raise ValueError("connect_timeout_s must be positive.")
        if not 5_000 <= command_lease_ms <= 60_000:
            raise ValueError("command_lease_ms must be in [5000, 60000].")

        self.endpoint = endpoint
        self._connect_timeout_s = float(connect_timeout_s)
        self._command_lease_ms = command_lease_ms
        self._channel: grpc.Channel | None = None
        self._stub: bridge_pb2_grpc.YaskawaBridgeStub | None = None
        self._discovery: Any = None
        self._session_token: str | None = None
        self._session_expiry_monotonic_s = 0.0
        self._active_stream_calls: list[Any] = []
        self._stream_lock = threading.Lock()

    def open(self) -> None:
        """Open the channel and validate the bridge version and safety surface."""

        if self._channel is not None:
            return
        channel = grpc.insecure_channel(self.endpoint)
        try:
            grpc.channel_ready_future(channel).result(timeout=self._connect_timeout_s)
            stub = bridge_pb2_grpc.YaskawaBridgeStub(channel)
            contract = stub.GetContract(
                bridge_pb2.GetContractRequest(), timeout=self._connect_timeout_s
            )
            if contract.contract_major != BRIDGE_CONTRACT_MAJOR:
                raise YaskawaGrpcTransportError(
                    "Unsupported Yaskawa bridge contract major "
                    f"{contract.contract_major}; expected {BRIDGE_CONTRACT_MAJOR}."
                )
            if contract.sdk_version != BRIDGE_SDK_VERSION:
                raise YaskawaGrpcTransportError(
                    f"Yaskawa bridge reports SDK {contract.sdk_version!r}; "
                    f"expected {BRIDGE_SDK_VERSION}."
                )
            if (
                contract.servo_power_operations_exposed
                or contract.cartesian_motion_exposed
            ):
                raise YaskawaGrpcTransportError(
                    "Yaskawa bridge exposes operations outside the reviewed v1 surface."
                )
        except BaseException:
            channel.close()
            raise
        self._channel = channel
        self._stub = stub

    def close(self) -> None:
        """Cancel feedback, close any command lease, and release the channel."""

        self.stop_feedback()
        stub = self._stub
        token = self._session_token
        self._session_token = None
        self._session_expiry_monotonic_s = 0.0
        try:
            if stub is not None and token is not None:
                stub.CloseCommandSession(
                    bridge_pb2.CommandSessionRequest(session_token=token),
                    timeout=self._connect_timeout_s,
                )
        finally:
            if self._channel is not None:
                self._channel.close()
            self._channel = None
            self._stub = None
            self._discovery = None

    def _require_stub(self) -> bridge_pb2_grpc.YaskawaBridgeStub:
        """Return the open generated stub or fail with an actionable error."""

        if self._stub is None:
            raise YaskawaGrpcTransportError(
                "Yaskawa bridge transport must be opened before an RPC."
            )
        return self._stub

    def _discover(self, group_no: int) -> Any:
        """Return one cached bridge discovery response for the selected group."""

        if group_no != 0:
            raise YaskawaGrpcTransportError(
                "Yaskawa bridge v1 supports discovery for group 0 only."
            )
        if self._discovery is None:
            self._discovery = self._require_stub().Discover(
                bridge_pb2.DiscoverRequest(group_no=group_no),
                timeout=self._connect_timeout_s,
            )
        return self._discovery

    @staticmethod
    def _timestamp(value: int) -> dict[str, int]:
        """Shape a bridge integer timestamp like the vendor response field."""

        return {"time": value}

    def get_firmware_version(self) -> object:
        """Return bridge discovery fields in vendor firmware-response shape."""

        response = self._discover(0)
        return {
            "status": response.firmware_status,
            "timestamp": self._timestamp(response.firmware_timestamp),
            "string_version_firmware": response.firmware_version,
            "controller_type": response.controller_type,
        }

    def get_configuration(self) -> object:
        """Return bridge discovery fields in vendor configuration shape."""

        response = self._discover(0)
        return {
            "status": response.configuration_status,
            "timestamp": self._timestamp(response.configuration_timestamp),
            "group_bit": response.group_bit,
            "group_num": response.group_num,
            "robot_num": response.robot_num,
        }

    def get_group_properties(self, group_no: int) -> object:
        """Return bridge discovery fields in vendor group-properties shape."""

        response = self._discover(group_no)
        return {
            "status": response.group_status,
            "timestamp": self._timestamp(response.group_timestamp),
            "properties": {
                "group_name": response.group_name,
                "axis_num": response.axis_num,
                "axis_bit": response.axis_bit,
                "axis_types": list(response.axis_types),
                "max_joint_speeds": list(response.max_joint_speeds),
                "min_axis_limits": list(response.min_axis_limits),
                "max_axis_limits": list(response.max_axis_limits),
            },
        }

    def get_mode(self) -> object:
        """Return one read-only mode observation in vendor response shape."""

        response = self._require_stub().GetMode(
            bridge_pb2.GetModeRequest(), timeout=self._connect_timeout_s
        )
        return {
            "status": response.status,
            "timestamp": self._timestamp(response.controller_timestamp),
            "mode": response.mode,
            "remote": response.remote,
        }

    def stream_feedback(
        self,
        *,
        group_no: int,
        tool_no: int,
        rate_hz: float,
        stop_event: threading.Event,
    ) -> Iterable[FeedbackResponses]:
        """Merge the three raw streams only at equal controller timestamps."""

        stub = self._require_stub()
        request = bridge_pb2.FeedbackStreamRequest(
            group_no=group_no, tool_no=tool_no, rate_hz=rate_hz
        )
        events: queue.Queue[tuple[str, Any]] = queue.Queue()
        calls = {
            "joints": stub.StreamFeedbackAxes(request),
            "torques": stub.StreamFeedbackTorque(request),
            "cartesian": stub.StreamFeedbackCartesian(request),
        }
        with self._stream_lock:
            self._active_stream_calls = list(calls.values())

        def consume(kind: str, call: Any) -> None:
            """Forward one blocking generated stream into the merge queue."""

            try:
                for response in call:
                    events.put((kind, response))
                events.put(("ended", kind))
            except BaseException as exc:  # noqa: BLE001
                events.put(("error", exc))

        threads = [
            threading.Thread(
                target=consume,
                args=(kind, call),
                name=f"yaskawa-{kind}-grpc",
                daemon=True,
            )
            for kind, call in calls.items()
        ]
        for thread in threads:
            thread.start()

        pending: dict[int, dict[str, Any]] = {}
        try:
            while not stop_event.is_set():
                try:
                    kind, payload = events.get(timeout=0.1)
                except queue.Empty:
                    continue
                if kind == "error":
                    raise YaskawaGrpcTransportError(
                        f"Yaskawa feedback stream failed: {payload}"
                    ) from payload
                if kind == "ended":
                    raise YaskawaGrpcTransportError(
                        f"Yaskawa {payload} feedback stream ended unexpectedly."
                    )
                timestamp = int(payload.controller_timestamp)
                timestamp_set = pending.setdefault(timestamp, {})
                timestamp_set[kind] = payload
                if set(timestamp_set) == {"joints", "torques", "cartesian"}:
                    yield self._feedback_responses(timestamp_set)
                    del pending[timestamp]
                while len(pending) > MAX_PENDING_TIMESTAMPS:
                    del pending[min(pending)]
        finally:
            for call in calls.values():
                call.cancel()
            for thread in threads:
                thread.join(timeout=1.0)
            with self._stream_lock:
                self._active_stream_calls = []

    @classmethod
    def _feedback_responses(cls, values: dict[str, Any]) -> FeedbackResponses:
        """Shape one timestamp-aligned bridge sample like vendor responses."""

        joints = values["joints"]
        torques = values["torques"]
        cartesian = values["cartesian"]
        tcp = list(cartesian.tcp_mm_euler_degrees)
        if len(tcp) != 6:
            raise YaskawaGrpcTransportError(
                "Yaskawa bridge Cartesian feedback must contain six values."
            )
        timestamp = cls._timestamp(joints.controller_timestamp)
        return FeedbackResponses(
            joints={
                "status": joints.status,
                "timestamp": timestamp,
                "axes_pos": {"pos": list(joints.position_degrees)},
            },
            torques={
                "status": torques.status,
                "timestamp": cls._timestamp(torques.controller_timestamp),
                "trq": list(torques.torque_nm),
            },
            cartesian={
                "status": cartesian.status,
                "timestamp": cls._timestamp(cartesian.controller_timestamp),
                "tool_no": cartesian.returned_tool_no,
                "cartesian_pos": {
                    "pos": {
                        "point": {"x": tcp[0], "y": tcp[1], "z": tcp[2]},
                        "orient": {"rx": tcp[3], "ry": tcp[4], "rz": tcp[5]},
                    }
                },
            },
        )

    def stop_feedback(self) -> None:
        """Cancel every active generated feedback reader."""

        with self._stream_lock:
            calls = tuple(self._active_stream_calls)
        for call in calls:
            call.cancel()

    def _session(self) -> str:
        """Return a fresh exclusive command token, opening it lazily if needed."""

        now = time.monotonic()
        if self._session_token is not None and now < self._session_expiry_monotonic_s:
            return self._session_token
        self._session_token = None
        response = self._require_stub().OpenCommandSession(
            bridge_pb2.OpenCommandSessionRequest(
                lease_duration_ms=self._command_lease_ms
            ),
            timeout=self._connect_timeout_s,
        )
        if not response.session_token:
            raise YaskawaGrpcTransportError(
                "Yaskawa bridge opened a command session without a token."
            )
        self._session_token = response.session_token
        self._renew_session(response.lease_duration_ms)
        return response.session_token

    def _renew_session(self, lease_duration_ms: int | None = None) -> None:
        """Advance the local fail-closed command-session expiry."""

        duration_ms = (
            self._command_lease_ms if lease_duration_ms is None else lease_duration_ms
        )
        self._session_expiry_monotonic_s = time.monotonic() + duration_ms / 1000.0

    def _motion_call(self, method_name: str, request: Any, timeout_s: float) -> Any:
        """Execute one generated motion RPC and renew the locally tracked lease."""

        method = getattr(self._require_stub(), method_name)
        try:
            response = method(request, timeout=timeout_s)
        except grpc.RpcError:
            # Transport ambiguity or an invalid/expired lease must never leave
            # a token eligible for silent reuse. The ACU watchdog owns cleanup.
            self._session_token = None
            self._session_expiry_monotonic_s = 0.0
            raise
        self._renew_session()
        return response

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
        """Publish one absolute joint target through the exclusive session."""

        token = self._session()
        request = bridge_pb2.JointTargetRequest(
            group_no=group_no,
            tool_no=tool_no,
            target_degrees=target_degrees,
            speed_percent=speed_percent,
            acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent,
            timeout_ms=timeout,
            session_token=token,
        )
        return self._motion_call(
            "SendJointMotionTarget", request, timeout / 1000.0 + 2.0
        )

    def receive_target(self, *, group_no: int, target_id: int, timeout: int) -> object:
        """Wait for one queued target result through the exclusive session."""

        request = bridge_pb2.ReceiveTargetRequest(
            group_no=group_no,
            target_id=target_id,
            timeout_ms=timeout,
            session_token=self._session(),
        )
        return self._motion_call("ReceiveTarget", request, timeout / 1000.0 + 2.0)

    def clear_target(self, *, control_group_bit: int) -> object:
        """Clear queued targets owned by the exclusive session."""

        request = bridge_pb2.ClearTargetRequest(
            control_group_bit=control_group_bit, session_token=self._session()
        )
        return self._motion_call("ClearTarget", request, self._connect_timeout_s)

    def _command_request(self) -> Any:
        """Build a session-bearing request for an empty vendor motion call."""

        return bridge_pb2.CommandSessionRequest(session_token=self._session())

    def start_motion(self) -> object:
        """Start the queued target owned by the exclusive session."""

        return self._motion_call(
            "StartMotion", self._command_request(), self._connect_timeout_s
        )

    def stop_motion(self) -> object:
        """Stop queued motion owned by the exclusive session."""

        return self._motion_call(
            "StopMotion", self._command_request(), self._connect_timeout_s
        )

    def abort_motion(self) -> object:
        """Abort motion owned by the exclusive session."""

        return self._motion_call(
            "AbortMotion", self._command_request(), self._connect_timeout_s
        )

    def start_increment_move(self, *, control_group_bit: int) -> object:
        """Acquire one incremental task through the exclusive session."""

        request = bridge_pb2.StartIncrementRequest(
            control_group_bit=control_group_bit, session_token=self._session()
        )
        return self._motion_call("StartIncrementMove", request, self._connect_timeout_s)

    def set_increment_move(
        self,
        *,
        task_no: int,
        timeout: int,
        requests: Sequence[IncrementMoveRequest],
    ) -> object:
        """Publish one incremental cycle through the exclusive session."""

        if len(requests) != 1:
            raise YaskawaGrpcTransportError(
                "Yaskawa bridge v1 requires exactly one increment group request."
            )
        increment = requests[0]
        request = bridge_pb2.SetIncrementRequest(
            task_no=task_no,
            timeout_ms=timeout,
            group_no=increment.group_no,
            tool_no=increment.tool_no,
            increment_degrees=increment.angle_degrees,
            session_token=self._session(),
        )
        return self._motion_call("SetIncrementMove", request, timeout / 1000.0 + 2.0)

    def stop_increment_move(self, *, task_no: int) -> object:
        """Stop one incremental task through the exclusive session."""

        request = bridge_pb2.StopIncrementRequest(
            task_no=task_no, session_token=self._session()
        )
        return self._motion_call("StopIncrementMove", request, self._connect_timeout_s)
