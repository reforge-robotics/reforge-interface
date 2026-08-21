"""Rotation-vector <-> quaternion conversion tests.

`get_tcp_pose()` converts RTDE's rotation-vector TCP orientation to Reforge's
`[qx, qy, qz, qw]` quaternion; `command_move_pose()` converts the other
direction. Both go through `scipy.spatial.transform.Rotation`, whose
`as_quat()`/`from_quat()` already use Reforge's scalar-last ordering -- these
tests pin that ordering down and check round trips near zero and near pi,
where rotation-vector representations are most prone to sign/wrap bugs.
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def rotvec_to_quat_xyzw(rotvec):
    """Mirrors get_tcp_pose()'s conversion direction."""
    return Rotation.from_rotvec(rotvec).as_quat().tolist()


def quat_xyzw_to_rotvec(quat):
    """Mirrors command_move_pose()'s conversion direction."""
    return Rotation.from_quat(quat).as_rotvec()


def assert_rotations_equivalent(rotvec_a, rotvec_b, tol=1e-6):
    """Compare via the rotation itself, not raw components.

    A rotation vector's axis/angle has a +/-2*pi*axis ambiguity in general,
    so component-wise comparison is the wrong check; comparing the relative
    rotation's angle is robust to that ambiguity.
    """
    relative_angle = (Rotation.from_rotvec(rotvec_a).inv() * Rotation.from_rotvec(rotvec_b)).magnitude()
    assert relative_angle < tol, (rotvec_a, rotvec_b, relative_angle)


# --- Ordering: Reforge expects [qx, qy, qz, qw] (scalar-last) -----------------------


def test_identity_rotation_round_trips_to_identity_quaternion():
    quat = rotvec_to_quat_xyzw(np.zeros(3))
    assert quat == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_quat_component_order_is_xyzw_not_wxyz():
    # A 90 deg rotation about +Z has a nonzero qz and qw, and zero qx/qy --
    # this pins down which slot is the scalar term.
    quat = rotvec_to_quat_xyzw([0.0, 0.0, np.pi / 2])
    qx, qy, qz, qw = quat
    assert qx == pytest.approx(0.0, abs=1e-9)
    assert qy == pytest.approx(0.0, abs=1e-9)
    assert qz == pytest.approx(np.sin(np.pi / 4))
    assert qw == pytest.approx(np.cos(np.pi / 4))


# --- Round trips: near zero, near pi, and a general orientation ---------------------


@pytest.mark.parametrize(
    "label,rotvec",
    [
        ("near zero", np.array([1e-6, -2e-6, 3e-6])),
        ("near pi, single axis", np.array([np.pi - 1e-4, 0.0, 0.0])),
        ("near pi, mixed axes", (np.pi - 1e-4) * np.array([1.0, 1.0, 1.0]) / np.sqrt(3)),
        ("general orientation", np.array([0.3, -0.7, 1.1])),
    ],
)
def test_rotvec_quat_round_trip(label, rotvec):
    quat = rotvec_to_quat_xyzw(rotvec)
    rotvec_back = quat_xyzw_to_rotvec(quat)

    assert_rotations_equivalent(rotvec, rotvec_back)


def test_quat_rotvec_round_trip_starting_from_quaternion():
    quat = np.array([0.5, 0.5, 0.5, 0.5])
    quat = quat / np.linalg.norm(quat)

    rotvec = quat_xyzw_to_rotvec(quat)
    quat_back = rotvec_to_quat_xyzw(rotvec)

    assert_rotations_equivalent(Rotation.from_quat(quat).as_rotvec(), Rotation.from_quat(quat_back).as_rotvec())
