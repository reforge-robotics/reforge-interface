"""CRC9/RC9 b-CAP adapter for the DENSO COBOTTA PRO 900."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Protocol, Sequence

from robot.vendor.orin_bcap import BCAPClient

RC9_PROVIDER = "CaoProv.DENSO.VRC9"
BCAP_TCP_PORT = 5007
BCAP_TIMEOUT_MS = 2_000
COBOTTA_JOINT_COUNT = 6
SLAVE_RECV_FORMAT_TIMESTAMP_POSE_JOINT_CURRENT = 0x0054
SLAVE_JOINT_MODE = 0x002
P_TYPE_VALUE_COUNT = 7


class BcapTransport(Protocol):
    """Define the low-level b-CAP calls required by the COBOTTA wrapper."""

    def service_start(self, option: str = "") -> None:
        """Start the b-CAP service.

        Args:
            option: b-CAP service option.
        """

    def service_stop(self) -> None:
        """Stop the b-CAP service."""

    def controller_connect(
        self, name: str, provider: str, machine: str, option: str
    ) -> int:
        """Connect to a controller.

        Args:
            name: Controller name.
            provider: ORiN provider name.
            machine: Provider host name.
            option: Provider option.

        Returns:
            b-CAP controller handle.
        """

    def controller_disconnect(self, handle: int) -> None:
        """Disconnect a controller.

        Args:
            handle: b-CAP controller handle.
        """

    def controller_getrobot(self, handle: int, name: str, option: str = "") -> int:
        """Get a robot handle.

        Args:
            handle: b-CAP controller handle.
            name: Controller robot name.
            option: Provider option.

        Returns:
            b-CAP robot handle.
        """

    def robot_release(self, handle: int) -> None:
        """Release a robot handle.

        Args:
            handle: b-CAP robot handle.
        """

    def robot_execute(self, handle: int, command: str, param: object = None) -> object:
        """Execute an RC9 robot command.

        Args:
            handle: b-CAP robot handle.
            command: RC9 command name.
            param: Command parameters.

        Returns:
            Decoded b-CAP result.
        """

    def robot_move(
        self, handle: int, comp: int, pose: object, option: str = ""
    ) -> None:
        """Send a controller-managed point motion.

        Args:
            handle: b-CAP robot handle.
            comp: RC9 completion mode.
            pose: RC9 pose data.
            option: RC9 motion options.
        """


BcapTransportFactory = Callable[[str, int, int], BcapTransport]


@dataclass(frozen=True, slots=True)
class CobottaJointSample:
    """One normal-mode controller sample.

    Attributes:
        controller_timestamp_ms: Elapsed controller time [ms].
        positions_rad: Joint positions [rad].
        velocities_rad_s: Derived joint velocities [rad/s].
        efforts: DENSO electric current values, not converted to torque.
        velocity_valid: Whether `velocities_rad_s` uses a valid sample interval.
    """

    controller_timestamp_ms: float
    positions_rad: list[float]
    velocities_rad_s: list[float]
    efforts: list[float]
    velocity_valid: bool


class CobottaControlMode(Enum):
    """Represent the b-CAP command mode currently owned by the wrapper."""

    NORMAL = "normal"
    SLAVE = "slave"


class CobottaBcapRobot:
    """Own a CRC9 b-CAP session and normal robot-control operations.

    The wrapper starts TCP/service/controller/robot resources, takes arm
    control, enables motors, and releases every acquired resource in reverse
    order. Public methods use Reforge units; only commands sent to b-CAP use
    DENSO degrees and millimeters.
    """

    def __init__(
        self,
        host: str,
        *,
        port: int = BCAP_TCP_PORT,
        timeout_ms: int = BCAP_TIMEOUT_MS,
        robot_name: str = "Arm",
        joint_count: int = COBOTTA_JOINT_COUNT,
        transport_factory: BcapTransportFactory = BCAPClient,
    ) -> None:
        """Configure a disconnected b-CAP session.

        Args:
            host: CRC9 host name or IP address.
            port: b-CAP TCP port.
            timeout_ms: Timeout passed to the pinned b-CAP implementation [ms].
            robot_name: Controller robot object name.
            joint_count: Number of Reforge arm joints.
            transport_factory: Low-level client factory for injection in tests.

        Raises:
            ValueError: If session configuration is invalid.
        """
        if not host or not robot_name:
            raise ValueError("host and robot_name must not be empty")
        if not 1 <= port <= 65_535 or timeout_ms <= 0 or joint_count <= 0:
            raise ValueError("port, timeout_ms, and joint_count must be positive")
        self._host, self._port, self._timeout_ms = host, port, timeout_ms
        self._robot_name, self._joint_count = robot_name, joint_count
        self._transport_factory = transport_factory
        self._transport: BcapTransport | None = None
        self._controller_handle: int | None = None
        self._robot_handle: int | None = None
        self._has_arm_control = False
        self._motors_enabled = False
        self._gravity_compensation_enabled = False
        self._control_mode = CobottaControlMode.NORMAL
        self._latest_slave_sample: CobottaJointSample | None = None
        self._latest_slave_pose: list[float] | None = None

    @property
    def is_connected(self) -> bool:
        """Return whether the session owns a b-CAP robot handle.

        Returns:
            `True` if connected.
        """
        return self._robot_handle is not None

    @property
    def gravity_compensation_enabled(self) -> bool:
        """Return whether this client enabled `GrvCtrl`.

        Returns:
            `True` if normal-mode gravity compensation is active.
        """
        return self._gravity_compensation_enabled

    @property
    def control_mode(self) -> CobottaControlMode:
        """Return the current b-CAP control mode.

        Returns:
            Normal or Slave Mode.
        """
        return self._control_mode

    def connect(self) -> None:
        """Open b-CAP and establish ordinary arm-control mode.

        Raises:
            RuntimeError: If already connected.
            Exception: If b-CAP setup fails after safe partial cleanup.
        """
        if self._transport is not None:
            raise RuntimeError("b-CAP session is already connected")
        try:
            self._transport = self._transport_factory(
                self._host, self._port, self._timeout_ms
            )
            self._transport.service_start("")
            self._controller_handle = self._transport.controller_connect(
                "", RC9_PROVIDER, "localhost", ""
            )
            self._robot_handle = self._transport.controller_getrobot(
                self._controller_handle, self._robot_name, ""
            )
            self._execute("TakeArm", [0, 0])
            self._has_arm_control = True
            self._execute("Motor", [1, 0])
            self._motors_enabled = True
        except BaseException:
            self._cleanup(raise_errors=False)
            raise

    def disconnect(self) -> None:
        """Release all b-CAP resources in reverse lifecycle order.

        Raises:
            RuntimeError: If one or more cleanup operations fail.
        """
        self._cleanup(raise_errors=True)

    def move_j(
        self, target_joints_rad: Sequence[float], *, speed_percent: float, wait: bool
    ) -> None:
        """Perform a controller-managed J-type point motion.

        Args:
            target_joints_rad: Joint target [rad].
            speed_percent: External speed [%].
            wait: Whether to wait for completion.
        """
        joints_deg = [
            math.degrees(value)
            for value in self._values(
                target_joints_rad, self._joint_count, "target_joints_rad"
            )
        ]
        self._move([joints_deg, "J", "@E"], speed_percent, wait)

    def move_pose(
        self,
        target_quaternion: Sequence[float],
        target_xyz_m: Sequence[float],
        *,
        speed_percent: float,
        wait: bool,
    ) -> None:
        """Perform a controller-managed P-type Cartesian point motion.

        P-type values use millimeters and XYZ angles in degrees. Tool/work-frame
        selection remains controller configuration to validate on hardware.

        Args:
            target_quaternion: Orientation `[qx, qy, qz, qw]` [-].
            target_xyz_m: Position `[x, y, z]` [m].
            speed_percent: External speed [%].
            wait: Whether to wait for completion.
        """
        xyz_mm = [
            value * 1_000.0 for value in self._values(target_xyz_m, 3, "target_xyz_m")
        ]
        self._move(
            [xyz_mm + self._quaternion_to_xyz_deg(target_quaternion) + [0], "P", "@E"],
            speed_percent,
            wait,
        )

    def get_current_joint_sample(self) -> CobottaJointSample:
        """Read `CurJntEx` and convert the flat degree payload to radians.

        Returns:
            Timestamped controller joint sample.

        Raises:
            RuntimeError: If the response has an unexpected shape.
        """
        self._require_normal_control()
        response = self._execute("CurJntEx")
        if isinstance(response, (str, bytes)) or not isinstance(response, Sequence):
            raise RuntimeError("CurJntEx returned a non-array payload")
        if len(response) < self._joint_count + 1:
            raise RuntimeError("CurJntEx returned fewer joints than configured")
        timestamp_ms = self._scalar(response[0], "CurJntEx timestamp")
        joints_deg = self._values(
            response[1 : self._joint_count + 1], self._joint_count, "CurJntEx joints"
        )
        return CobottaJointSample(
            timestamp_ms,
            [math.radians(value) for value in joints_deg],
            [0.0] * self._joint_count,
            [],
            False,
        )

    def get_current_pose(self) -> list[float]:
        """Read normal-mode `CurPos` as a Reforge TCP pose.

        Returns:
            TCP pose `[x, y, z, qx, qy, qz, qw]` in meters and quaternion form.

        Raises:
            RuntimeError: If `CurPos` has an unexpected P-type payload.
        """
        if self._control_mode is CobottaControlMode.SLAVE:
            if self._latest_slave_pose is None:
                raise RuntimeError("Slave Mode has no received TCP pose yet")
            return list(self._latest_slave_pose)
        self._require_normal_control()
        response = self._execute("CurPos")
        if isinstance(response, (str, bytes)) or not isinstance(response, Sequence):
            raise RuntimeError("CurPos returned a non-array payload")
        if len(response) < 6:
            raise RuntimeError("CurPos returned too few P-type values")
        return self._p_type_to_pose(response)

    def get_joint_sample(self) -> CobottaJointSample:
        """Return normal telemetry or the latest telemetry from `slvMove`.

        Returns:
            Latest joint sample in Reforge units.

        Raises:
            RuntimeError: If Slave Mode has not received a `slvMove` response.
        """
        if self._control_mode is CobottaControlMode.NORMAL:
            return self.get_current_joint_sample()
        if self._latest_slave_sample is None:
            raise RuntimeError("Slave Mode has no received joint sample yet")
        return self._latest_slave_sample

    def enter_servo_mode(self) -> None:
        """Enter joint-type b-CAP Slave Mode and configure feedback.

        Gravity compensation is explicitly disabled before entering because RC9
        does not permit `GrvCtrl` while the client controls Slave Mode.

        Raises:
            RuntimeError: If ordinary controller control is unavailable.
        """
        self._require_normal_control()
        if self._gravity_compensation_enabled:
            self.set_gravity_compensation(False)
        self._execute("slvSendFormat", 0x0000)
        self._execute("slvRecvFormat", SLAVE_RECV_FORMAT_TIMESTAMP_POSE_JOINT_CURRENT)
        self._execute("slvChangeMode", SLAVE_JOINT_MODE)
        self._control_mode = CobottaControlMode.SLAVE
        self._latest_slave_sample = None
        self._latest_slave_pose = None

    def enter_position_mode(self) -> None:
        """Leave Slave Mode and ensure ordinary robot-control mode is active.

        Raises:
            RuntimeError: If the session is disconnected.
        """
        self._require_robot_handle()
        if self._control_mode is CobottaControlMode.SLAVE:
            self._execute("slvChangeMode", 0x000)
            self._control_mode = CobottaControlMode.NORMAL
        self._latest_slave_pose = None
        if self._gravity_compensation_enabled:
            self.set_gravity_compensation(False)

    def command_servo_j(self, target_joints_rad: Sequence[float]) -> CobottaJointSample:
        """Send one joint setpoint through `slvMove` and cache its response.

        Args:
            target_joints_rad: Joint target [rad].

        Returns:
            Telemetry received in the `slvMove` response.

        Raises:
            RuntimeError: If Slave Mode is not active or the response is invalid.
        """
        self._require_slave_mode()
        target_deg = [
            math.degrees(value)
            for value in self._values(
                target_joints_rad,
                self._joint_count,
                "target_joints_rad",
            )
        ]
        sample = self._parse_slave_response(self._execute("slvMove", target_deg))
        self._latest_slave_sample = sample
        return sample

    def set_gravity_compensation(self, enabled: bool) -> None:
        """Enable or disable normal-mode RC9 gravity compensation.

        Args:
            enabled: Whether to enable `GrvCtrl`.
        """
        self._require_normal_control()
        self._execute("GrvCtrl", bool(enabled))
        self._gravity_compensation_enabled = enabled

    def _move(self, pose: object, speed_percent: float, wait: bool) -> None:
        """Send one validated normal-mode controller motion.

        Args:
            pose: RC9 J-type or P-type pose data.
            speed_percent: External speed [%].
            wait: Whether to wait for completion.
        """
        speed = self._scalar(speed_percent, "speed_percent")
        if not 0.0 < speed <= 100.0:
            raise ValueError("speed_percent must be in the interval (0, 100]")
        self._require_normal_control()
        self._execute("ExtSpeed", [speed, speed, speed])
        self._require_transport().robot_move(
            self._require_robot_handle(), 1, pose, "" if wait else "NEXT"
        )

    def _cleanup(self, *, raise_errors: bool) -> None:
        """Best-effort cleanup, preserving the first failure after all steps.

        Args:
            raise_errors: Raise a summary error after cleanup failures.
        """
        errors: list[BaseException] = []

        def run(operation: Callable[[], object]) -> None:
            """Run a cleanup operation without preventing later cleanup.

            Args:
                operation: Cleanup operation.
            """
            try:
                operation()
            except BaseException as exc:  # noqa: BLE001
                errors.append(exc)

        if (
            self._robot_handle is not None
            and self._control_mode is CobottaControlMode.SLAVE
        ):
            run(lambda: self._execute("slvChangeMode", 0x000))
        self._control_mode = CobottaControlMode.NORMAL
        self._latest_slave_sample = None
        self._latest_slave_pose = None
        if self._robot_handle is not None and self._gravity_compensation_enabled:
            run(lambda: self._execute("GrvCtrl", False))
        self._gravity_compensation_enabled = False
        if self._robot_handle is not None and self._motors_enabled:
            run(lambda: self._execute("Motor", [0, 0]))
        self._motors_enabled = False
        if self._robot_handle is not None and self._has_arm_control:
            run(lambda: self._execute("GiveArm"))
        self._has_arm_control = False

        transport, robot_handle = self._transport, self._robot_handle
        self._robot_handle = None
        if transport is not None and robot_handle is not None:
            run(lambda: transport.robot_release(robot_handle))
        controller_handle = self._controller_handle
        self._controller_handle = None
        if transport is not None and controller_handle is not None:
            run(lambda: transport.controller_disconnect(controller_handle))
        if transport is not None:
            run(transport.service_stop)
            close = getattr(transport, "close", None)
            if callable(close):
                run(close)
        self._transport = None
        if errors and raise_errors:
            raise RuntimeError("b-CAP cleanup failed") from errors[0]

    def _execute(self, command: str, parameter: object = None) -> object:
        """Execute an RC9 command against the active robot.

        Args:
            command: RC9 command name.
            parameter: Command parameter.

        Returns:
            Decoded b-CAP result.
        """
        return self._require_transport().robot_execute(
            self._require_robot_handle(), command, parameter
        )

    def _require_normal_control(self) -> None:
        """Ensure ordinary controller control is active.

        Raises:
            RuntimeError: If the robot session is not ready for normal control.
        """
        self._require_robot_handle()
        if self._control_mode is not CobottaControlMode.NORMAL:
            raise RuntimeError("normal robot control is unavailable in Slave Mode")
        if not self._has_arm_control or not self._motors_enabled:
            raise RuntimeError("normal robot control is not active")

    def _require_slave_mode(self) -> None:
        """Ensure joint-type b-CAP Slave Mode is active.

        Raises:
            RuntimeError: If the session is not connected or not in Slave Mode.
        """
        self._require_robot_handle()
        if self._control_mode is not CobottaControlMode.SLAVE:
            raise RuntimeError("command_servo_j requires active Slave Mode")

    def _parse_slave_response(self, response: object) -> CobottaJointSample:
        """Parse timestamp, P+J data, and electric current from `slvMove`.

        Receive format `0x0054` returns `[timestamp_ms, p_plus_j, current]`.
        The P-type prefix has seven values before J-type joint values. DENSO
        calls the final field electric current, so it is returned unchanged as
        effort/current telemetry and never identified as torque.

        Args:
            response: Decoded b-CAP `slvMove` return value.

        Returns:
            Reforge-unit position/velocity sample with DENSO current values.

        Raises:
            RuntimeError: If the response does not match receive format `0x0054`.
        """
        if isinstance(response, (str, bytes)) or not isinstance(response, Sequence):
            raise RuntimeError("slvMove returned a non-array payload")
        if len(response) != 3:
            raise RuntimeError("slvMove response does not match receive format 0x0054")
        timestamp_ms = self._scalar(response[0], "slvMove timestamp")
        pose_and_joints = response[1]
        currents = response[2]
        if isinstance(pose_and_joints, (str, bytes)) or not isinstance(
            pose_and_joints, Sequence
        ):
            raise RuntimeError("slvMove P+J response is not an array")
        joint_start = P_TYPE_VALUE_COUNT
        joint_stop = joint_start + self._joint_count
        if len(pose_and_joints) < joint_stop:
            raise RuntimeError("slvMove P+J response contains too few joint values")
        positions_deg = self._values(
            pose_and_joints[joint_start:joint_stop],
            self._joint_count,
            "slvMove joint positions",
        )
        if isinstance(currents, (str, bytes)) or not isinstance(currents, Sequence):
            raise RuntimeError("slvMove electric-current response is not an array")
        efforts = self._values(
            currents[: self._joint_count],
            self._joint_count,
            "slvMove electric currents",
        )
        positions_rad = [math.radians(value) for value in positions_deg]
        velocities_rad_s, velocity_valid = self._derive_slave_velocity(
            timestamp_ms,
            positions_rad,
        )
        self._latest_slave_pose = self._p_type_to_pose(pose_and_joints)
        return CobottaJointSample(
            timestamp_ms,
            positions_rad,
            velocities_rad_s,
            efforts,
            velocity_valid,
        )

    def _p_type_to_pose(self, p_type_values: Sequence[float]) -> list[float]:
        """Convert a P-type prefix to a Reforge TCP pose.

        Args:
            p_type_values: P-type values beginning with XYZ and XYZ angles.

        Returns:
            TCP pose `[x, y, z, qx, qy, qz, qw]` in Reforge units.

        Raises:
            RuntimeError: If the P-type prefix is malformed.
        """
        if len(p_type_values) < 6:
            raise RuntimeError("P-type response contains too few pose values")
        values = self._values(p_type_values[:6], 6, "P-type pose")
        return [
            *(coordinate_mm / 1_000.0 for coordinate_mm in values[:3]),
            *self._xyz_deg_to_quaternion(values[3:]),
        ]

    def _derive_slave_velocity(
        self,
        timestamp_ms: float,
        positions_rad: Sequence[float],
    ) -> tuple[list[float], bool]:
        """Derive velocity from strictly increasing controller timestamps.

        A zero vector with `False` validity denotes that no physical derivative
        is available yet; it avoids division by zero or misleading velocity for
        duplicate, non-monotonic, or malformed controller timestamps.

        Args:
            timestamp_ms: Controller timestamp [ms].
            positions_rad: Current joint positions [rad].

        Returns:
            Derived velocities [rad/s] and a validity indicator.
        """
        previous = self._latest_slave_sample
        if previous is None:
            return [0.0] * self._joint_count, False
        elapsed_ms = timestamp_ms - previous.controller_timestamp_ms
        if not math.isfinite(elapsed_ms) or elapsed_ms <= 0.0:
            return [0.0] * self._joint_count, False
        elapsed_s = elapsed_ms / 1_000.0
        return (
            [
                (position - previous_position) / elapsed_s
                for position, previous_position in zip(
                    positions_rad,
                    previous.positions_rad,
                    strict=True,
                )
            ],
            True,
        )

    def _require_transport(self) -> BcapTransport:
        """Return the connected transport.

        Returns:
            Active b-CAP transport.
        """
        if self._transport is None:
            raise RuntimeError("b-CAP session is not connected")
        return self._transport

    def _require_robot_handle(self) -> int:
        """Return the acquired robot handle.

        Returns:
            Active b-CAP robot handle.
        """
        if self._robot_handle is None:
            raise RuntimeError("b-CAP robot handle is not connected")
        return self._robot_handle

    @staticmethod
    def _values(values: Sequence[float], length: int, name: str) -> list[float]:
        """Validate a fixed-length finite numeric sequence.

        Args:
            values: Candidate numeric values.
            length: Required length.
            name: Input name for error messages.

        Returns:
            Validated values.
        """
        if isinstance(values, (str, bytes)) or len(values) != length:
            raise ValueError(f"{name} must contain exactly {length} values")
        return [CobottaBcapRobot._scalar(value, name) for value in values]

    @staticmethod
    def _scalar(value: object, name: str) -> float:
        """Validate and return one finite real value.

        Args:
            value: Candidate value.
            name: Input name for error messages.

        Returns:
            Finite float value.
        """
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a finite real number")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite real number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{name} must be a finite real number")
        return number

    @staticmethod
    def _quaternion_to_xyz_deg(quaternion: Sequence[float]) -> list[float]:
        """Convert `[qx, qy, qz, qw]` to XYZ angles in degrees.

        Args:
            quaternion: Quaternion orientation [-].

        Returns:
            XYZ orientation angles [deg].
        """
        qx, qy, qz, qw = CobottaBcapRobot._values(quaternion, 4, "target_quaternion")
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            raise ValueError("target_quaternion must have nonzero magnitude")
        qx, qy, qz, qw = (qx / norm, qy / norm, qz / norm, qw / norm)
        roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    @staticmethod
    def _xyz_deg_to_quaternion(angles_deg: Sequence[float]) -> list[float]:
        """Convert XYZ orientation angles in degrees to `[qx, qy, qz, qw]`.

        Args:
            angles_deg: XYZ orientation angles [deg].

        Returns:
            Unit quaternion ` [qx, qy, qz, qw]` [-].
        """
        roll, pitch, yaw = (
            math.radians(value)
            for value in CobottaBcapRobot._values(angles_deg, 3, "CurPos angles")
        )
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
