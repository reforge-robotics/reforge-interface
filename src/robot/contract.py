"""Hardware-free Yaskawa ACU contract and discovery helpers.

The ACU SDK is supplied out of band and its generated bindings are not part of
this repository.  This module therefore accepts protobuf-like response objects
(attributes or mappings) at one narrow boundary.  The same boundary can be
fed by generated Python bindings or by the Phase 1 C++ bridge without copying
vendor message definitions into ``reforge-core``.

No function in this module opens a socket, powers a servo, or publishes motion.
It validates the read-only system-information contract and converts documented
Yaskawa units into the units used by Reforge.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Any


DEFAULT_GROUP_NO = 0
DEFAULT_TOOL_NO = 0
DEFAULT_FRAME = "base"
EXPECTED_NEX7_AXIS_COUNT = 6
MAX_MONITOR_RATE_HZ = 250.0
DEGREES_TO_RADIANS = math.pi / 180.0
MILLIMETERS_TO_METERS = 1.0e-3


class YaskawaContractError(ValueError):
    """Raised when an ACU response cannot satisfy the integration contract."""


class YaskawaResponseError(YaskawaContractError):
    """Raised when an ACU operation reports a non-success status."""


class ResponseStatus(StrEnum):
    """Normalized response status values shared by ACU services."""

    SUCCESS = "success"
    UNSPECIFIED = "unspecified"
    UNKNOWN_ERROR = "unknown_error"
    REJECTED = "rejected"


class AxisType(StrEnum):
    """Axis types reported by ``SystemInfoService``."""

    ROTATE = "rotate"
    LINEAR = "linear"
    UNKNOWN = "unknown"


class ControllerType(StrEnum):
    """Controller types supported by the Phase 1 target contract."""

    YNX1000 = "ynx1000"
    UNKNOWN = "unknown"


class ControllerMode(StrEnum):
    """Operator mode reported by ``ModeGetService``."""

    TEACH = "teach"
    PLAY = "play"
    UNKNOWN = "unknown"


@dataclass(frozen=True, slots=True)
class AxisSpec:
    """Describe one enabled controller axis in SDK and Reforge units.

    Args:
        sdk_index: Index in the fixed-width ACU arrays.
        axis_type: Validated ACU axis type.
        min_position_rad: Lower joint limit [rad].
        max_position_rad: Upper joint limit [rad].
        max_speed_rad_s: Maximum rotational speed [rad/s].
    """

    sdk_index: int
    axis_type: AxisType
    min_position_rad: float
    max_position_rad: float
    max_speed_rad_s: float


@dataclass(frozen=True, slots=True)
class DiscoveryReport:
    """Validated read-only controller and group discovery result.

    The controller timestamp is an uptime tick/count from the ACU, not a Unix
    timestamp.  Phase 3 must measure the host clock offset before recordings
    can align it with Reforge wall-clock data.
    """

    controller_type: ControllerType
    firmware_version: str
    group_no: int
    group_name: str
    axis_specs: tuple[AxisSpec, ...]
    controller_timestamp: int | None
    default_tool_no: int = DEFAULT_TOOL_NO
    default_frame: str = DEFAULT_FRAME
    euler_convention: str = "extrinsic_xyz_equivalent_to_intrinsic_zyx"

    @property
    def axis_count(self) -> int:
        """Return the number of enabled axes in controller order."""

        return len(self.axis_specs)

    @property
    def sdk_axis_indices(self) -> tuple[int, ...]:
        """Return enabled fixed-array indices in controller order."""

        return tuple(spec.sdk_index for spec in self.axis_specs)

    def diagnostic_dict(self) -> dict[str, Any]:
        """Return a JSON-compatible, no-credential diagnostic summary."""

        return {
            "controller_type": self.controller_type.value,
            "firmware_version": self.firmware_version,
            "group_no": self.group_no,
            "group_name": self.group_name,
            "axis_count": self.axis_count,
            "sdk_axis_indices": list(self.sdk_axis_indices),
            "axis_limits_rad": [
                [spec.min_position_rad, spec.max_position_rad]
                for spec in self.axis_specs
            ],
            "axis_speed_limits_rad_s": [
                spec.max_speed_rad_s for spec in self.axis_specs
            ],
            "controller_timestamp": self.controller_timestamp,
            "controller_timestamp_kind": "controller_uptime_ticks",
            "default_tool_no": self.default_tool_no,
            "default_frame": self.default_frame,
            "euler_convention": self.euler_convention,
        }


@dataclass(frozen=True, slots=True)
class ModeReport:
    """Read-only controller mode and Remote-state observation."""

    mode: ControllerMode
    remote_enabled: bool
    controller_timestamp: int


@dataclass(frozen=True, slots=True)
class FeedbackSample:
    """One synchronized, validated read-only feedback sample.

    Args:
        joint_positions_rad: Enabled joint positions in controller order [rad].
        joint_torques_nm: Enabled feedback torques in controller order [N-m].
        tcp_pose: Base-frame TCP pose ``[x, y, z, qx, qy, qz, qw]`` [m, -].
        controller_timestamp: ACU uptime tick shared by all source responses.
    """

    joint_positions_rad: tuple[float, ...]
    joint_torques_nm: tuple[float, ...]
    tcp_pose: tuple[float, float, float, float, float, float, float]
    controller_timestamp: int


def _field(message: object, name: str, *, required: bool = True) -> Any:
    """Read one field from a protobuf-like object or mapping.

    Args:
        message: Response object or mapping supplied by a transport boundary.
        name: Field name to read.
        required: Whether a missing field is a contract error.

    Returns:
        The field value, or ``None`` for an optional missing field.

    Raises:
        YaskawaContractError: If a required field is missing.
    """

    if isinstance(message, Mapping):
        if name in message:
            return message[name]
    elif hasattr(message, name):
        return getattr(message, name)
    if required:
        raise YaskawaContractError(f"ACU response is missing required field {name!r}.")
    return None


def _enum_name(value: object) -> str:
    """Normalize a protobuf enum, string enum, or integer-like value name."""

    if isinstance(value, StrEnum):
        return value.value.upper()
    if isinstance(value, str):
        return value.rsplit(".", 1)[-1].upper()
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name.upper()
    return str(value).upper()


def normalize_status(value: object) -> ResponseStatus:
    """Normalize a vendor status while preserving fail-closed semantics.

    ACU generated bindings expose enum values as integers; test doubles and a
    future C++ bridge may expose symbolic names.  Only an explicit success
    value is considered successful.  Unknown numeric values remain rejected.

    Args:
        value: Raw ACU status field.

    Returns:
        Normalized status category.
    """

    if isinstance(value, int) and not isinstance(value, bool):
        return ResponseStatus.SUCCESS if value == 1 else ResponseStatus.REJECTED
    name = _enum_name(value)
    if name.endswith("STATUS_SUCCESS") or name in {"SUCCESS", "OK"}:
        return ResponseStatus.SUCCESS
    if name.endswith("STATUS_UNSPECIFIED") or name in {"UNSPECIFIED", "0"}:
        return ResponseStatus.UNSPECIFIED
    if name.endswith("STATUS_UNKNOWN_ERROR") or "UNKNOWN_ERROR" in name:
        return ResponseStatus.UNKNOWN_ERROR
    return ResponseStatus.REJECTED


def require_success(response: object, operation: str) -> None:
    """Raise a descriptive error unless a response reports explicit success.

    Args:
        response: ACU response object or mapping.
        operation: Human-readable RPC operation name.

    Raises:
        YaskawaResponseError: If ``status`` is missing or not successful.
    """

    raw_status = _field(response, "status")
    status = normalize_status(raw_status)
    if status is not ResponseStatus.SUCCESS:
        raise YaskawaResponseError(
            f"Yaskawa {operation} failed with status {raw_status!r} "
            f"({status.value})."
        )


def _finite(value: object, description: str) -> float:
    """Return a finite float or reject malformed vendor data."""

    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise YaskawaContractError(
            f"{description} must be numeric; received {value!r}."
        ) from exc
    if not math.isfinite(result):
        raise YaskawaContractError(f"{description} must be finite.")
    return result


def enabled_axis_indices(axis_bit: int, axis_count: int) -> tuple[int, ...]:
    """Return fixed-array indices enabled by an ACU axis bitmap.

    Args:
        axis_bit: ACU bitmap where bit zero is the first fixed array slot.
        axis_count: Controller-reported enabled-axis count.

    Returns:
        Enabled fixed-array indices in ascending controller order.

    Raises:
        YaskawaContractError: If the bitmap is malformed or disagrees with the
            reported axis count.
    """

    if not isinstance(axis_bit, int) or isinstance(axis_bit, bool) or axis_bit < 0:
        raise YaskawaContractError("axis_bit must be a non-negative integer.")
    if (
        not isinstance(axis_count, int)
        or isinstance(axis_count, bool)
        or axis_count <= 0
    ):
        raise YaskawaContractError("axis_num must be a positive integer.")
    indices = tuple(
        index for index in range(axis_bit.bit_length()) if axis_bit & (1 << index)
    )
    if len(indices) != axis_count:
        raise YaskawaContractError(
            "ACU axis bitmap/count mismatch: "
            f"axis_bit={axis_bit:#x} enables {len(indices)} axes, "
            f"axis_num={axis_count}."
        )
    return indices


def _enabled_values(
    values: object, axis_bit: int, axis_count: int, description: str
) -> tuple[object, ...]:
    """Select enabled values from an ACU fixed-width array.

    Args:
        values: Candidate repeated protobuf field or equivalent sequence.
        axis_bit: ACU bitmap where bit zero is the first fixed array slot.
        axis_count: Controller-reported enabled-axis count.
        description: Field name used in validation errors.

    Returns:
        Values for enabled axes in ascending controller order.

    Raises:
        YaskawaContractError: If the field is not an indexable sequence or is
            too short for the enabled bitmap.
    """

    if not isinstance(values, Sequence) or isinstance(values, (str, bytes, bytearray)):
        raise YaskawaContractError(f"{description} must be an array.")

    indices = enabled_axis_indices(axis_bit, axis_count)
    if len(values) <= indices[-1]:
        raise YaskawaContractError(
            f"{description} has {len(values)} entries but requires SDK index "
            f"{indices[-1]}."
        )
    return tuple(values[index] for index in indices)


def _controller_type(value: object) -> ControllerType:
    """Normalize and validate the firmware controller type."""

    if isinstance(value, int) and not isinstance(value, bool):
        return ControllerType.YNX1000 if value == 1 else ControllerType.UNKNOWN
    name = _enum_name(value)
    if name.endswith("CONTROLLER_TYPE_YNX1000") or name == "YNX1000":
        return ControllerType.YNX1000
    return ControllerType.UNKNOWN


def _axis_type(value: object) -> AxisType:
    """Normalize a ``GroupProperties.AxisType`` value."""

    if isinstance(value, int) and not isinstance(value, bool):
        if value == 2:
            return AxisType.ROTATE
        if value == 3:
            return AxisType.LINEAR
        return AxisType.UNKNOWN
    name = _enum_name(value)
    if name.endswith("AXIS_TYPE_ROTATE") or name == "ROTATE":
        return AxisType.ROTATE
    if name.endswith("AXIS_TYPE_LINEAR") or name == "LINEAR":
        return AxisType.LINEAR
    return AxisType.UNKNOWN


def _timestamp(response: object) -> int | None:
    """Extract an optional controller uptime timestamp without conversion."""

    timestamp = _field(response, "timestamp", required=False)
    if timestamp is None:
        return None
    raw_time = _field(timestamp, "time", required=False)
    if raw_time is None:
        return None
    try:
        value = int(raw_time)
    except (TypeError, ValueError) as exc:
        raise YaskawaContractError("controller timestamp must be an integer.") from exc
    if value < 0:
        raise YaskawaContractError("controller timestamp must be non-negative.")
    return value


def discover_controller(
    firmware_response: object,
    configuration_response: object,
    group_response: object,
    *,
    group_no: int = DEFAULT_GROUP_NO,
    expected_axis_count: int = EXPECTED_NEX7_AXIS_COUNT,
) -> DiscoveryReport:
    """Validate read-only ACU discovery responses for the v1 NEX7 contract.

    The function validates firmware/controller type, enabled group, fixed-width
    axis ordering, rotary axis types, finite limits, and maximum speeds.  It
    deliberately does not infer an exact C00-C03 variant from firmware text;
    that remains a hardware-derived fact for the no-motion preflight.

    Args:
        firmware_response: ``GetFirmwareVersionResponse``-like object.
        configuration_response: ``GetConfigurationResponse``-like object.
        group_response: ``GetGroupPropertiesResponse``-like object.
        group_no: Expected ACU group number.
        expected_axis_count: Expected enabled axis count for the target model.

    Returns:
        A validated, immutable discovery report.

    Raises:
        YaskawaContractError: If any response is malformed or outside the v1
            NEX7/YNX1000 contract.
        YaskawaResponseError: If an RPC reports a non-success status.
    """

    if group_no < 0:
        raise YaskawaContractError("group_no must be non-negative.")
    if expected_axis_count <= 0:
        raise YaskawaContractError("expected_axis_count must be positive.")

    require_success(firmware_response, "GetFirmwareVersion")
    require_success(configuration_response, "GetConfiguration")
    require_success(group_response, "GetGroupProperties")

    controller_type = _controller_type(_field(firmware_response, "controller_type"))
    if controller_type is not ControllerType.YNX1000:
        raise YaskawaContractError(
            "Unsupported Yaskawa controller type: "
            f"{_field(firmware_response, 'controller_type')!r}. Expected YNX1000."
        )

    group_bit = _field(configuration_response, "group_bit")
    if not isinstance(group_bit, int) or isinstance(group_bit, bool) or group_bit < 0:
        raise YaskawaContractError("group_bit must be a non-negative integer.")
    if not group_bit & (1 << group_no):
        raise YaskawaContractError(
            f"Configured group {group_no} is not enabled by group_bit={group_bit:#x}."
        )

    properties = _field(group_response, "properties")
    axis_count = _field(properties, "axis_num")
    if axis_count != expected_axis_count:
        raise YaskawaContractError(
            f"Expected {expected_axis_count} enabled axes for NEX7 group {group_no}; "
            f"controller reports {axis_count}."
        )
    axis_bit = _field(properties, "axis_bit")
    indices = enabled_axis_indices(axis_bit, axis_count)
    axis_types = _field(properties, "axis_types")
    max_speeds = _field(properties, "max_joint_speeds")
    max_limits = _field(properties, "max_axis_limits")
    min_limits = _field(properties, "min_axis_limits")
    enabled_types = _enabled_values(axis_types, axis_bit, axis_count, "axis_types")
    enabled_speeds = _enabled_values(
        max_speeds, axis_bit, axis_count, "max_joint_speeds"
    )
    enabled_max_limits = _enabled_values(
        max_limits, axis_bit, axis_count, "max_axis_limits"
    )
    enabled_min_limits = _enabled_values(
        min_limits, axis_bit, axis_count, "min_axis_limits"
    )

    axis_specs: list[AxisSpec] = []
    for sdk_index, raw_type, raw_speed, raw_min, raw_max in zip(
        indices,
        enabled_types,
        enabled_speeds,
        enabled_min_limits,
        enabled_max_limits,
    ):
        axis_type = _axis_type(raw_type)
        if axis_type is not AxisType.ROTATE:
            raise YaskawaContractError(
                f"NEX7 v1 requires rotary axes; SDK index {sdk_index} is "
                f"{raw_type!r}."
            )
        min_deg = _finite(raw_min, f"axis {sdk_index} minimum limit")
        max_deg = _finite(raw_max, f"axis {sdk_index} maximum limit")
        speed_deg_s = _finite(raw_speed, f"axis {sdk_index} maximum speed")
        if min_deg >= max_deg:
            raise YaskawaContractError(
                f"axis {sdk_index} has invalid limits [{min_deg}, {max_deg}] deg."
            )
        if speed_deg_s <= 0.0:
            raise YaskawaContractError(
                f"axis {sdk_index} maximum speed must be positive."
            )
        axis_specs.append(
            AxisSpec(
                sdk_index=sdk_index,
                axis_type=axis_type,
                min_position_rad=min_deg * DEGREES_TO_RADIANS,
                max_position_rad=max_deg * DEGREES_TO_RADIANS,
                max_speed_rad_s=speed_deg_s * DEGREES_TO_RADIANS,
            )
        )

    firmware_version = _field(firmware_response, "string_version_firmware")
    if not isinstance(firmware_version, str) or not firmware_version.strip():
        raise YaskawaContractError("string_version_firmware must be non-empty.")
    group_name = _field(properties, "group_name", required=False) or f"group-{group_no}"
    if not isinstance(group_name, str) or not group_name.strip():
        raise YaskawaContractError("group_name must be non-empty when provided.")

    return DiscoveryReport(
        controller_type=controller_type,
        firmware_version=firmware_version,
        group_no=group_no,
        group_name=group_name,
        axis_specs=tuple(axis_specs),
        controller_timestamp=_timestamp(firmware_response),
    )


def convert_joint_degrees_to_radians(
    values: Sequence[object], axis_indices: Sequence[int]
) -> tuple[float, ...]:
    """Select controller axes and convert degrees to radians.

    Args:
        values: Fixed-width ACU angle array in degrees.
        axis_indices: Enabled SDK indices from :attr:`DiscoveryReport.sdk_axis_indices`.

    Returns:
        Joint positions in Reforge controller order [rad].

    Raises:
        YaskawaContractError: If indices or values are malformed/non-finite.
    """

    if isinstance(values, (str, bytes, bytearray)) or isinstance(
        axis_indices, (str, bytes, bytearray)
    ):
        raise YaskawaContractError("joint angles and axis indices must be arrays.")

    converted: list[float] = []
    for index in axis_indices:
        if not isinstance(index, int) or index < 0 or index >= len(values):
            raise YaskawaContractError(
                f"joint axis index {index!r} is outside the supplied angle array."
            )
        converted.append(
            _finite(values[index], f"joint angle at SDK index {index}")
            * DEGREES_TO_RADIANS
        )
    if not converted:
        raise YaskawaContractError("joint angle array must contain at least one axis.")
    return tuple(converted)


def convert_cartesian_millimeters_to_meters(
    values: Sequence[object],
) -> tuple[float, ...]:
    """Convert a three-element Cartesian translation from mm to m.

    Args:
        values: Cartesian translation in Yaskawa order [mm].

    Returns:
        Cartesian translation in the same order [m].

    Raises:
        YaskawaContractError: If the input is not a three-element numeric
            sequence or contains a non-finite value.
    """

    if isinstance(values, (str, bytes, bytearray)) or len(values) != 3:
        raise YaskawaContractError(
            f"Cartesian translation must contain three values; received {len(values)}."
        )
    return tuple(
        _finite(value, f"Cartesian translation component {index}")
        * MILLIMETERS_TO_METERS
        for index, value in enumerate(values)
    )


def yaskawa_euler_degrees_to_quaternion(
    rx_deg: object, ry_deg: object, rz_deg: object
) -> tuple[float, float, float, float]:
    """Convert Yaskawa ``Rx, Ry, Rz`` degrees to ``[qx, qy, qz, qw]``.

    Phase 1 freezes the mathematical convention as extrinsic XYZ, equivalent
    to intrinsic ZYX (``Rz * Ry * Rx``).  The result remains provisional for
    Cartesian commands until a Yaskawa known-pose sample confirms the mapping.

    Args:
        rx_deg: Rotation around X [deg].
        ry_deg: Rotation around Y [deg].
        rz_deg: Rotation around Z [deg].

    Returns:
        Unit quaternion in Reforge order ``[qx, qy, qz, qw]``.
    """

    roll = _finite(rx_deg, "Euler rx") * DEGREES_TO_RADIANS / 2.0
    pitch = _finite(ry_deg, "Euler ry") * DEGREES_TO_RADIANS / 2.0
    yaw = _finite(rz_deg, "Euler rz") * DEGREES_TO_RADIANS / 2.0
    sin_roll, cos_roll = math.sin(roll), math.cos(roll)
    sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    quaternion = (
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
    )
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise YaskawaContractError("Euler conversion produced an invalid quaternion.")
    normalized = tuple(component / norm for component in quaternion)
    return (normalized[0], normalized[1], normalized[2], normalized[3])


def parse_mode_response(response: object) -> ModeReport:
    """Validate and normalize one ``GetMode`` response.

    Args:
        response: Protobuf-like ``GetModeResponse`` object or mapping.

    Returns:
        Read-only Teach/Play and Remote-state observation.

    Raises:
        YaskawaContractError: If mode, Remote state, or timestamp is malformed.
        YaskawaResponseError: If the response status is not successful.
    """

    require_success(response, "GetMode")
    raw_mode = _field(response, "mode")
    if isinstance(raw_mode, int) and not isinstance(raw_mode, bool):
        mode = {1: ControllerMode.TEACH, 2: ControllerMode.PLAY}.get(
            raw_mode, ControllerMode.UNKNOWN
        )
    else:
        mode_name = _enum_name(raw_mode)
        if mode_name.endswith("MODE_TEACH") or mode_name == "TEACH":
            mode = ControllerMode.TEACH
        elif mode_name.endswith("MODE_PLAY") or mode_name == "PLAY":
            mode = ControllerMode.PLAY
        else:
            mode = ControllerMode.UNKNOWN
    if mode is ControllerMode.UNKNOWN:
        raise YaskawaContractError(f"Unsupported controller mode {raw_mode!r}.")

    remote = _field(response, "remote")
    if remote not in (0, 1) or isinstance(remote, bool):
        raise YaskawaContractError(
            f"GetMode remote must be integer 0 or 1; received {remote!r}."
        )
    timestamp = _timestamp(response)
    if timestamp is None:
        raise YaskawaContractError(
            "GetMode response is missing a controller timestamp."
        )
    return ModeReport(
        mode=mode,
        remote_enabled=remote == 1,
        controller_timestamp=timestamp,
    )


def parse_feedback_sample(
    joint_response: object,
    torque_response: object,
    cartesian_response: object,
    discovery: DiscoveryReport,
) -> FeedbackSample:
    """Validate synchronized feedback responses and convert them to Reforge units.

    The thin ACU bridge must aggregate responses only when their controller
    timestamps match. Rejecting mixed-cycle values prevents a plausible-looking
    state from being assembled from observations made at different I/O cycles.

    Args:
        joint_response: ``GetFeedbackAxesPosStreamResponse``-like value.
        torque_response: ``GetFeedbackTorqueStreamResponse``-like value in N-m.
        cartesian_response: ``GetFeedbackCartesianPosStreamResponse``-like value.
        discovery: Validated group layout used for fixed-width axis selection.

    Returns:
        Converted, synchronized feedback sample.

    Raises:
        YaskawaContractError: If values, dimensions, tool, or timestamps are invalid.
        YaskawaResponseError: If any response status is not successful.
    """

    responses = (
        (joint_response, "GetFeedbackAxesPosStream"),
        (torque_response, "GetFeedbackTorqueStream"),
        (cartesian_response, "GetFeedbackCartesianPosStream"),
    )
    timestamps: list[int] = []
    for response, operation in responses:
        require_success(response, operation)
        timestamp = _timestamp(response)
        if timestamp is None:
            raise YaskawaContractError(
                f"{operation} response is missing a controller timestamp."
            )
        timestamps.append(timestamp)
    if len(set(timestamps)) != 1:
        raise YaskawaContractError(
            "Feedback responses are not synchronized: controller timestamps "
            f"are {timestamps}."
        )

    axes_pos = _field(_field(joint_response, "axes_pos"), "pos")
    joint_positions = convert_joint_degrees_to_radians(
        axes_pos, discovery.sdk_axis_indices
    )
    torque_values = _field(torque_response, "trq")
    selected_torques = _enabled_values(
        torque_values,
        sum(1 << index for index in discovery.sdk_axis_indices),
        discovery.axis_count,
        "feedback torque",
    )
    joint_torques = tuple(
        _finite(value, f"feedback torque at SDK index {sdk_index}")
        for sdk_index, value in zip(discovery.sdk_axis_indices, selected_torques)
    )

    tool_no = _field(cartesian_response, "tool_no")
    if tool_no != discovery.default_tool_no:
        raise YaskawaContractError(
            f"Feedback Cartesian tool {tool_no!r} does not match requested tool "
            f"{discovery.default_tool_no}."
        )
    frame = _field(_field(cartesian_response, "cartesian_pos"), "pos")
    point = _field(frame, "point")
    translation = convert_cartesian_millimeters_to_meters(
        [_field(point, "x"), _field(point, "y"), _field(point, "z")]
    )
    orient = _field(frame, "orient")
    quaternion = yaskawa_euler_degrees_to_quaternion(
        _field(orient, "rx"), _field(orient, "ry"), _field(orient, "rz")
    )
    return FeedbackSample(
        joint_positions_rad=joint_positions,
        joint_torques_nm=joint_torques,
        tcp_pose=(
            translation[0],
            translation[1],
            translation[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
        ),
        controller_timestamp=timestamps[0],
    )
