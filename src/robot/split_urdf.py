#!/usr/bin/env python3
"""Split a bimanual URDF into left- and right-arm models.

This splitter deliberately does not infer arm membership from joint or link names.
The operator identifies the first actuator joint for each arm.  Descendant
joints are followed until the chain ends.  If a link has multiple child joints,
the operator must identify the last joint belonging to the arm (or provide it
with ``--left-end-joint`` / ``--right-end-joint``).

Each generated URDF retains every original link, visual, and collision element.
Only revolute joints on the selected arm path remain movable.  Every other
movable joint, including continuous and prismatic joints, is converted to a
fixed joint at its URDF zero position.

The generated URDF root is a new massless link whose origin is coincident with
the selected first actuator's joint origin at zero position.  Its orientation
is chosen so that the first actuator's axis is parallel to world Z, with its
positive direction along world -Z.  The Axol arm therefore extends above the
world z=0 plane, toward world +Z.
A fixed joint connects that new root to the complete original robot.  The torso
and inactive arm consequently remain stationary collision geometry.
World-frame pose constants used with a generated model must therefore be
re-expressed in that model's new root frame.

Example::

    python3 -m robot.split_urdf split src/robot/urdf/axol.urdf \
        --left-first-joint left_s1_0 \
        --right-first-joint right_s1_0 \
        --visualize

Existing generated URDFs can be viewed without splitting them again::

    python3 -m robot.split_urdf simulate \
        src/robot/urdf/axol-left.urdf \
        src/robot/urdf/axol-right.urdf

For compatibility, the original command form without the explicit ``split``
subcommand remains supported.

The optional viewer requires ``viser[urdf]``.  The script expects a URDF,
not a xacro file, and assumes the URDF describes one
connected tree.
"""

from __future__ import annotations

import argparse
import copy
import math
import re
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import TypeAlias
from xml.etree import ElementTree as ET

Matrix4: TypeAlias = list[list[float]]
_MOVABLE_JOINT_TYPES = {"continuous", "floating", "planar", "prismatic", "revolute"}
_FIXED_JOINT_FIELDS = {
    "axis",
    "calibration",
    "dynamics",
    "limit",
    "mimic",
    "safety_controller",
}


class SplitError(ValueError):
    """Raised when the source URDF or requested arm selection is ambiguous."""


@dataclass(frozen=True)
class UrdfGraph:
    """The link/joint topology needed by the splitter."""

    root_link: str
    links: dict[str, ET.Element]
    joints: dict[str, ET.Element]
    outgoing: dict[str, list[ET.Element]]
    incoming: dict[str, ET.Element]


@dataclass(frozen=True)
class ArmSelection:
    """Resolved path and independently movable joints for one output model."""

    label: str
    first_joint: str
    path: tuple[str, ...]
    active_joints: tuple[str, ...]


def _joint_parent(joint: ET.Element) -> str:
    parent = joint.find("parent")
    if parent is None or not parent.get("link"):
        raise SplitError(f"Joint {joint.get('name')!r} has no valid <parent link=...>.")
    return parent.get("link", "")


def _joint_child(joint: ET.Element) -> str:
    child = joint.find("child")
    if child is None or not child.get("link"):
        raise SplitError(f"Joint {joint.get('name')!r} has no valid <child link=...>.")
    return child.get("link", "")


def _build_graph(robot: ET.Element) -> UrdfGraph:
    links: dict[str, ET.Element] = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not name:
            raise SplitError("Every <link> must have a name.")
        if name in links:
            raise SplitError(f"Duplicate link name: {name!r}.")
        links[name] = link

    joints: dict[str, ET.Element] = {}
    outgoing: dict[str, list[ET.Element]] = defaultdict(list)
    incoming: dict[str, ET.Element] = {}
    for joint in robot.findall("joint"):
        name = joint.get("name")
        if not name:
            raise SplitError("Every <joint> must have a name.")
        if name in joints:
            raise SplitError(f"Duplicate joint name: {name!r}.")
        parent = _joint_parent(joint)
        child = _joint_child(joint)
        if parent not in links:
            raise SplitError(
                f"Joint {name!r} references missing parent link {parent!r}."
            )
        if child not in links:
            raise SplitError(f"Joint {name!r} references missing child link {child!r}.")
        if child in incoming:
            other = incoming[child].get("name")
            raise SplitError(
                f"Link {child!r} has more than one parent joint: {other!r} and {name!r}."
            )
        joints[name] = joint
        outgoing[parent].append(joint)
        incoming[child] = joint

    roots = [name for name in links if name not in incoming]
    if len(roots) != 1:
        raise SplitError(
            "The URDF must contain one connected tree; "
            f"found {len(roots)} candidate root links: {roots}."
        )

    # A traversal catches cycles and disconnected components that the root test
    # alone cannot explain clearly.
    visited_links: set[str] = set()
    stack = [roots[0]]
    while stack:
        link_name = stack.pop()
        if link_name in visited_links:
            raise SplitError(f"The URDF joint graph contains a cycle at {link_name!r}.")
        visited_links.add(link_name)
        stack.extend(_joint_child(joint) for joint in outgoing.get(link_name, ()))
    if visited_links != set(links):
        missing = sorted(set(links) - visited_links)
        raise SplitError(f"Links are disconnected from root {roots[0]!r}: {missing}.")

    return UrdfGraph(
        root_link=roots[0],
        links=links,
        joints=joints,
        outgoing=dict(outgoing),
        incoming=incoming,
    )


def _resolve_mesh_path(filename: str, source_directory: Path, *, context: str) -> Path:
    """Resolve one mesh filename using the source URDF directory.

    ``package://PACKAGE/path`` is interpreted as ``path`` relative to the
    source URDF directory. Ordinary relative paths use that same directory.

    Args:
        filename: Mesh filename from the URDF.
        source_directory: Directory containing the source URDF.
        context: Human-readable link and geometry role for errors.

    Returns:
        Absolute path to the existing mesh file.

    Raises:
        SplitError: If the filename is empty, malformed, or does not exist.
    """

    if not filename:
        raise SplitError(f"{context} has a <mesh> without a filename.")

    if filename.startswith("package://"):
        package_relative = filename.removeprefix("package://")
        package_name, separator, relative = package_relative.partition("/")
        if (
            not package_name
            or not separator
            or not relative
            or Path(relative).is_absolute()
        ):
            raise SplitError(f"{context} has malformed mesh path {filename!r}.")
        path = source_directory / relative
    else:
        path = Path(filename).expanduser()
        if not path.is_absolute():
            path = source_directory / path

    resolved = path.resolve()
    if not resolved.is_file():
        raise SplitError(f"{context} mesh file does not exist: {resolved}")
    return resolved


def _validate_source_meshes(
    robot: ET.Element, root_link: str, source_directory: Path
) -> None:
    """Validate the requested visual/collision mesh contract for every link.

    The graph root alone may omit geometry when it contains neither a
    ``<visual>`` nor ``<collision>`` element. Every other link, and a root that
    declares either role, must contain at least one mesh-backed element for
    both roles. Every declared mesh filename must resolve to an existing file.

    Args:
        robot: Source URDF ``<robot>`` element.
        root_link: Name of the graph root link.
        source_directory: Directory containing the source URDF.

    Raises:
        SplitError: If a required role is missing a mesh or a mesh path is
            invalid.
    """

    for link in robot.findall("link"):
        link_name = link.get("name", "<unnamed>")
        visual_elements = link.findall("visual")
        collision_elements = link.findall("collision")
        if link_name == root_link and not visual_elements and not collision_elements:
            continue

        for role, elements in (
            ("visual", visual_elements),
            ("collision", collision_elements),
        ):
            meshes = [
                mesh
                for element in elements
                for mesh in element.findall("./geometry/mesh")
            ]
            if not meshes:
                raise SplitError(
                    f"Link {link_name!r} requires a mesh-backed <{role}> element."
                )
            for mesh in meshes:
                _resolve_mesh_path(
                    mesh.get("filename", ""),
                    source_directory,
                    context=f"Link {link_name!r} <{role}>",
                )


def _rewrite_mesh_paths(robot: ET.Element, source_directory: Path) -> None:
    """Replace generated mesh filenames with resolved filesystem paths.

    Args:
        robot: Generated URDF ``<robot>`` element to update.
        source_directory: Directory containing the unmodified source URDF.

    Raises:
        SplitError: If a mesh filename cannot be resolved.
    """

    for link in robot.findall("link"):
        link_name = link.get("name", "<unnamed>")
        for role in ("visual", "collision"):
            for mesh in link.findall(f"./{role}/geometry/mesh"):
                resolved = _resolve_mesh_path(
                    mesh.get("filename", ""),
                    source_directory,
                    context=f"Link {link_name!r} <{role}>",
                )
                mesh.set("filename", str(resolved))


def _find_descendant_joint_path(
    graph: UrdfGraph, start_joint: str, end_joint: str
) -> list[ET.Element] | None:
    """Return the unique descendant path from start to end, if it exists."""

    start = graph.joints[start_joint]

    def visit(joint: ET.Element, ancestors: set[str]) -> list[ET.Element] | None:
        name = joint.get("name", "")
        if name in ancestors:
            raise SplitError(f"Cycle encountered while tracing joint {name!r}.")
        if name == end_joint:
            return [joint]
        next_ancestors = ancestors | {name}
        for child_joint in graph.outgoing.get(_joint_child(joint), ()):
            suffix = visit(child_joint, next_ancestors)
            if suffix is not None:
                return [joint, *suffix]
        return None

    return visit(start, set())


def _format_branch(last_joint: ET.Element, choices: list[ET.Element]) -> str:
    child_link = _joint_child(last_joint)
    lines = [
        (
            f"Branch encountered after joint {last_joint.get('name')!r} "
            f"at link {child_link!r}:"
        ),
    ]
    for joint in choices:
        lines.append(
            f"  - {joint.get('name')} [{joint.get('type')}]: "
            f"{_joint_parent(joint)} -> {_joint_child(joint)}"
        )
    return "\n".join(lines)


def _resolve_arm_selection(
    graph: UrdfGraph,
    *,
    label: str,
    first_joint_name: str,
    end_joint_name: str | None,
) -> ArmSelection:
    if first_joint_name not in graph.joints:
        raise SplitError(f"Unknown {label} first actuator joint: {first_joint_name!r}.")

    if end_joint_name is not None:
        if end_joint_name not in graph.joints:
            raise SplitError(f"Unknown {label} end joint: {end_joint_name!r}.")
        path = _find_descendant_joint_path(graph, first_joint_name, end_joint_name)
        if path is None:
            raise SplitError(
                f"Joint {end_joint_name!r} is not a descendant of "
                f"{first_joint_name!r}."
            )
    else:
        path = []
        current = graph.joints[first_joint_name]
        while True:
            path.append(current)
            choices = graph.outgoing.get(_joint_child(current), [])
            if not choices:
                break
            if len(choices) == 1:
                current = choices[0]
                continue

            branch_message = _format_branch(current, choices)
            print(f"\n{label.capitalize()} arm: {branch_message}", file=sys.stderr)
            if not sys.stdin.isatty():
                option = f"--{label}-end-joint"
                raise SplitError(
                    f"Cannot resolve the {label} arm non-interactively. "
                    f"Pass {option} with the last joint belonging to the arm."
                )
            print(
                "Enter the name of the last joint belonging to this arm. "
                f"Enter {current.get('name')!r} if the arm ends before the branch.",
                file=sys.stderr,
            )
            selected_end = input(f"{label} arm last joint: ").strip()
            if not selected_end:
                raise SplitError("A joint name is required to resolve the branch.")
            if selected_end not in graph.joints:
                raise SplitError(f"Unknown joint: {selected_end!r}.")
            resolved = _find_descendant_joint_path(
                graph, first_joint_name, selected_end
            )
            if resolved is None:
                raise SplitError(
                    f"Joint {selected_end!r} is not a descendant of "
                    f"{first_joint_name!r}."
                )
            path = resolved
            break

    path_names = tuple(joint.get("name", "") for joint in path)
    active = tuple(
        joint.get("name", "") for joint in path if joint.get("type") == "revolute"
    )
    return ArmSelection(
        label=label,
        first_joint=first_joint_name,
        path=path_names,
        active_joints=active,
    )


def _identity() -> Matrix4:
    return [[1.0 if row == col else 0.0 for col in range(4)] for row in range(4)]


def _multiply(a: Matrix4, b: Matrix4) -> Matrix4:
    return [
        [sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4)]
        for row in range(4)
    ]


def _origin_transform(joint: ET.Element) -> Matrix4:
    origin = joint.find("origin")
    xyz_text = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy_text = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    try:
        xyz = [float(value) for value in xyz_text.split()]
        roll, pitch, yaw = [float(value) for value in rpy_text.split()]
    except ValueError as exc:
        raise SplitError(
            f"Joint {joint.get('name')!r} has a non-numeric origin."
        ) from exc
    if len(xyz) != 3 or len(rpy_text.split()) != 3:
        raise SplitError(f"Joint {joint.get('name')!r} must have three-value xyz/rpy.")

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # URDF fixed-axis RPY: Rz(yaw) @ Ry(pitch) @ Rx(roll).
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    return [
        [rotation[row][0], rotation[row][1], rotation[row][2], xyz[row]]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def _inverse_rigid(transform: Matrix4) -> Matrix4:
    rotation_t = [[transform[col][row] for col in range(3)] for row in range(3)]
    translation = [transform[row][3] for row in range(3)]
    inverse_translation = [
        -sum(rotation_t[row][k] * translation[k] for k in range(3)) for row in range(3)
    ]
    return [[*rotation_t[row], inverse_translation[row]] for row in range(3)] + [
        [0.0, 0.0, 0.0, 1.0]
    ]


def _joint_axis(joint: ET.Element) -> list[float]:
    """Return a selected revolute joint's normalized local axis."""

    if joint.get("type") != "revolute":
        raise SplitError(
            f"Joint {joint.get('name')!r} must be revolute; "
            f"found type {joint.get('type')!r}."
        )
    axis = joint.find("axis")
    axis_text = axis.get("xyz", "1 0 0") if axis is not None else "1 0 0"
    try:
        values = [float(value) for value in axis_text.split()]
    except ValueError as exc:
        raise SplitError(
            f"Joint {joint.get('name')!r} has a non-numeric axis."
        ) from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise SplitError(
            f"Joint {joint.get('name')!r} must have a finite three-value axis."
        )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise SplitError(f"Joint {joint.get('name')!r} has a zero-length axis.")
    return [value / norm for value in values]


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _rotation_aligning_axis_with_world_z(axis: list[float]) -> Matrix4:
    """Map the positive actuator axis to -Z so the Axol arm extends toward +Z."""

    target = [0.0, 0.0, -1.0]
    cross = _cross(axis, target)
    sine = math.sqrt(sum(value * value for value in cross))
    cosine = max(-1.0, min(1.0, sum(a * b for a, b in zip(axis, target))))

    if sine <= 1e-12:
        if cosine > 0.0:
            return _identity()
        # The vectors are antiparallel. Pick a deterministic axis orthogonal
        # to the source and rotate by pi: R = 2*u*u^T - I.
        reference = [1.0, 0.0, 0.0] if abs(axis[0]) < 0.9 else [0.0, 1.0, 0.0]
        rotation_axis = _cross(axis, reference)
        axis_norm = math.sqrt(sum(value * value for value in rotation_axis))
        rotation_axis = [value / axis_norm for value in rotation_axis]
        rotation = [
            [
                2.0 * rotation_axis[row] * rotation_axis[col]
                - (1.0 if row == col else 0.0)
                for col in range(3)
            ]
            for row in range(3)
        ]
    else:
        # Rodrigues' formula with an unnormalized cross-product matrix.
        x, y, z = cross
        skew = [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]]
        skew_squared = [
            [sum(skew[row][k] * skew[k][col] for k in range(3)) for col in range(3)]
            for row in range(3)
        ]
        scale = (1.0 - cosine) / (sine * sine)
        rotation = [
            [
                (1.0 if row == col else 0.0)
                + skew[row][col]
                + skew_squared[row][col] * scale
                for col in range(3)
            ]
            for row in range(3)
        ]

    return [[*rotation[row], 0.0] for row in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _matrix_to_xyz_rpy(transform: Matrix4) -> tuple[list[float], list[float]]:
    xyz = [transform[row][3] for row in range(3)]
    r20 = max(-1.0, min(1.0, transform[2][0]))
    pitch = math.asin(-r20)
    if abs(math.cos(pitch)) > 1e-10:
        roll = math.atan2(transform[2][1], transform[2][2])
        yaw = math.atan2(transform[1][0], transform[0][0])
    else:
        # At gimbal lock, select the equivalent representation with roll=0.
        roll = 0.0
        yaw = math.atan2(-transform[0][1], transform[1][1])
    return xyz, [roll, pitch, yaw]


def _root_to_joint_frame(graph: UrdfGraph, joint_name: str) -> Matrix4:
    selected = graph.joints[joint_name]
    parent_link = _joint_parent(selected)
    ancestors: list[ET.Element] = []
    cursor = parent_link
    while cursor != graph.root_link:
        incoming = graph.incoming.get(cursor)
        if incoming is None:
            raise SplitError(
                f"Could not trace link {cursor!r} back to root {graph.root_link!r}."
            )
        ancestors.append(incoming)
        cursor = _joint_parent(incoming)
    ancestors.reverse()

    transform = _identity()
    for joint in [*ancestors, selected]:
        transform = _multiply(transform, _origin_transform(joint))
    return transform


def _unique_name(preferred: str, existing: set[str]) -> str:
    if preferred not in existing:
        return preferred
    index = 2
    while f"{preferred}_{index}" in existing:
        index += 1
    return f"{preferred}_{index}"


def _freeze_joint(joint: ET.Element) -> None:
    joint.set("type", "fixed")
    for child in list(joint):
        if child.tag in _FIXED_JOINT_FIELDS:
            joint.remove(child)


def _remove_inactive_transmissions(
    robot: ET.Element, frozen_joint_names: set[str]
) -> list[str]:
    """Remove standard transmissions that refer to newly fixed joints."""

    removed: list[str] = []
    for transmission in list(robot.findall("transmission")):
        referenced = {joint.get("name", "") for joint in transmission.findall("joint")}
        if referenced & frozen_joint_names:
            removed.append(transmission.get("name", "<unnamed>"))
            robot.remove(transmission)
    return removed


def _format_number(value: float) -> str:
    if abs(value) < 1e-14:
        value = 0.0
    return f"{value:.17g}"


def _format_vector(values: list[float]) -> str:
    return " ".join(_format_number(value) for value in values)


def _element_signature(element: ET.Element) -> tuple[object, ...]:
    """Return a whitespace-insensitive structural signature."""

    text = (element.text or "").strip()
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        text,
        tuple(_element_signature(child) for child in element),
    )


def _generate_arm_tree(
    source_tree: ET.ElementTree,
    source_graph: UrdfGraph,
    selection: ArmSelection,
    source_directory: Path,
) -> tuple[ET.ElementTree, list[str], list[str]]:
    """Generate one split-arm tree while preserving the existing split rules.

    Args:
        source_tree: Parsed source URDF tree.
        source_graph: Validated topology of the source URDF.
        selection: Arm path whose revolute joints remain movable.
        source_directory: Directory used to resolve source mesh filenames.

    Returns:
        Generated tree, names of frozen joints, and removed transmissions.

    Raises:
        SplitError: If generation violates a splitter invariant or a generated
            mesh path cannot be resolved.
    """

    generated_tree = copy.deepcopy(source_tree)
    robot = generated_tree.getroot()
    generated_graph = _build_graph(robot)
    active = set(selection.active_joints)
    frozen: list[str] = []

    for name, joint in generated_graph.joints.items():
        if name not in active and joint.get("type") in _MOVABLE_JOINT_TYPES:
            frozen.append(name)
            _freeze_joint(joint)

    removed_transmissions = _remove_inactive_transmissions(robot, set(frozen))

    root_to_actuator = _root_to_joint_frame(source_graph, selection.first_joint)
    actuator_axis = _joint_axis(source_graph.joints[selection.first_joint])
    actuator_in_split_root = _rotation_aligning_axis_with_world_z(actuator_axis)
    split_root_to_source_root = _multiply(
        actuator_in_split_root,
        _inverse_rigid(root_to_actuator),
    )
    xyz, rpy = _matrix_to_xyz_rpy(split_root_to_source_root)

    link_names = set(generated_graph.links)
    joint_names = set(generated_graph.joints)
    safe_label = re.sub(r"[^A-Za-z0-9_]", "_", selection.label)
    base_link_name = _unique_name(f"__split_{safe_label}_arm_base", link_names)
    base_joint_name = _unique_name(
        f"__split_{safe_label}_arm_base_to_original_root", joint_names
    )

    new_base = ET.Element("link", {"name": base_link_name})
    first_joint_index = next(
        (index for index, child in enumerate(robot) if child.tag == "joint"),
        len(robot),
    )
    robot.insert(first_joint_index, new_base)

    root_joint = ET.Element("joint", {"name": base_joint_name, "type": "fixed"})
    ET.SubElement(
        root_joint,
        "origin",
        {"xyz": _format_vector(xyz), "rpy": _format_vector(rpy)},
    )
    ET.SubElement(root_joint, "parent", {"link": base_link_name})
    ET.SubElement(root_joint, "child", {"link": source_graph.root_link})
    robot.append(root_joint)

    _validate_generated_tree(
        source_tree=source_tree,
        generated_tree=generated_tree,
        selection=selection,
        expected_root=base_link_name,
        source_root_to_actuator=root_to_actuator,
        new_root_joint=root_joint,
    )
    _rewrite_mesh_paths(robot, source_directory)
    return generated_tree, frozen, removed_transmissions


def _validate_generated_tree(
    *,
    source_tree: ET.ElementTree,
    generated_tree: ET.ElementTree,
    selection: ArmSelection,
    expected_root: str,
    source_root_to_actuator: Matrix4,
    new_root_joint: ET.Element,
) -> None:
    source_robot = source_tree.getroot()
    generated_robot = generated_tree.getroot()
    source_graph = _build_graph(source_robot)
    generated_graph = _build_graph(generated_robot)

    if generated_graph.root_link != expected_root:
        raise SplitError(
            f"Generated root is {generated_graph.root_link!r}, expected {expected_root!r}."
        )

    movable = {
        name
        for name, joint in generated_graph.joints.items()
        if joint.get("type") != "fixed"
    }
    if movable != set(selection.active_joints):
        raise SplitError(
            f"Generated movable joints {sorted(movable)} do not equal selected "
            f"joints {sorted(selection.active_joints)}."
        )

    # No original link is modified or removed.  This specifically protects all
    # visual/collision geometry, inertials, materials, and mesh paths.
    for name, source_link in source_graph.links.items():
        generated_link = generated_graph.links.get(name)
        if generated_link is None:
            raise SplitError(f"Generated URDF is missing original link {name!r}.")
        if _element_signature(generated_link) != _element_signature(source_link):
            raise SplitError(f"Generated URDF modified original link {name!r}.")

    # The new base origin must coincide with the first actuator origin at q=0,
    # and its actuator axis must be parallel to world Z.  Axol extends opposite
    # the positive actuator direction, so positive points -Z and the arm +Z.
    split_root_to_actuator = _multiply(
        _origin_transform(new_root_joint),
        source_root_to_actuator,
    )
    origin_error = max(abs(split_root_to_actuator[row][3]) for row in range(3))
    actuator_axis = _joint_axis(source_graph.joints[selection.first_joint])
    axis_in_split_root = [
        sum(split_root_to_actuator[row][col] * actuator_axis[col] for col in range(3))
        for row in range(3)
    ]
    axis_error = max(
        abs(actual - expected)
        for actual, expected in zip(axis_in_split_root, (0.0, 0.0, -1.0))
    )
    if origin_error > 1e-9 or axis_error > 1e-9:
        raise SplitError(
            "Generated base does not place the first actuator at its origin "
            "with its positive axis along world -Z; "
            f"origin error is {origin_error:.3g}, axis error is {axis_error:.3g}."
        )


def _load_urdf(path: Path) -> ET.ElementTree:
    try:
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.parse(path, parser=parser)
    except (OSError, ET.ParseError) as exc:
        raise SplitError(f"Could not parse URDF {path}: {exc}") from exc
    if tree.getroot().tag != "robot":
        raise SplitError(f"Expected a <robot> root element in {path}.")
    return tree


def _write_tree(tree: ET.ElementTree, destination: Path, *, force: bool) -> None:
    if destination.exists() and not force:
        raise SplitError(
            f"Refusing to overwrite {destination}; pass --force to replace it."
        )
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="    ")
    tree.write(
        destination, encoding="utf-8", xml_declaration=False, short_empty_elements=True
    )


def _prompt_joint(value: str | None, prompt: str, graph: UrdfGraph) -> str:
    if value:
        return value
    if not sys.stdin.isatty():
        raise SplitError(f"Missing required input: {prompt.rstrip(': ')}.")
    print("Available revolute joints (parent -> child):", file=sys.stderr)
    for name, joint in graph.joints.items():
        if joint.get("type") == "revolute":
            print(
                f"  - {name}: {_joint_parent(joint)} -> {_joint_child(joint)}",
                file=sys.stderr,
            )
    entered = input(prompt).strip()
    if not entered:
        raise SplitError("A joint name is required.")
    return entered


def _print_summary(
    selection: ArmSelection,
    frozen: list[str],
    removed_transmissions: list[str],
    output: Path,
    graph: UrdfGraph,
) -> None:
    ignored_on_path = [
        name for name in selection.path if graph.joints[name].get("type") != "revolute"
    ]
    print(f"\n{selection.label.capitalize()} output: {output}")
    print(f"  First actuator frame: {selection.first_joint}")
    print("  First actuator axis: parallel to world Z (positive direction -Z)")
    print(
        f"  Active/addressable joints ({len(selection.active_joints)}): "
        + (", ".join(selection.active_joints) or "none")
    )
    print(
        f"  Frozen formerly movable joints ({len(frozen)}): "
        + (", ".join(frozen) or "none")
    )
    if ignored_on_path:
        print(
            "  Non-revolute path joints retained as fixed: "
            + ", ".join(ignored_on_path)
        )
    if removed_transmissions:
        print(
            "  Removed transmissions for frozen joints: "
            + ", ".join(removed_transmissions)
        )


def _add_viser_joint_controls(server: object, robot: object, label: str) -> None:
    """Add degree-valued joint sliders for one Viser URDF.

    Viser is an optional dependency, so the parameters intentionally use
    ``object`` rather than importing its runtime-only handle types globally.
    """

    gui = server.gui  # type: ignore[attr-defined]
    joint_names = robot.get_actuated_joint_names()  # type: ignore[attr-defined]
    joint_limits = robot.get_actuated_joint_limits()  # type: ignore[attr-defined]
    sliders: list[object] = []
    initial_values_deg: list[float] = []

    with gui.add_folder(f"{label.capitalize()} arm joints"):
        for joint_name in joint_names:
            lower_rad, upper_rad = joint_limits[joint_name]
            lower_rad = (
                -math.pi
                if lower_rad is None or not math.isfinite(lower_rad)
                else float(lower_rad)
            )
            upper_rad = (
                math.pi
                if upper_rad is None or not math.isfinite(upper_rad)
                else float(upper_rad)
            )
            if lower_rad >= upper_rad:
                lower_rad, upper_rad = -math.pi, math.pi
            initial_rad = min(max(0.0, lower_rad), upper_rad)
            initial_deg = math.degrees(initial_rad)
            initial_values_deg.append(initial_deg)
            sliders.append(
                gui.add_slider(
                    label=f"{joint_name} [deg]",
                    min=math.degrees(lower_rad),
                    max=math.degrees(upper_rad),
                    step=1.0,
                    initial_value=initial_deg,
                )
            )
        reset_button = gui.add_button(f"Reset {label} arm")

    def update_robot(_: object | None = None) -> None:
        robot.update_cfg(  # type: ignore[attr-defined]
            [math.radians(float(slider.value)) for slider in sliders]  # type: ignore[attr-defined]
        )

    for slider in sliders:
        slider.on_update(update_robot)  # type: ignore[attr-defined]

    def reset_robot(_: object) -> None:
        for slider, initial_value in zip(sliders, initial_values_deg):
            slider.value = initial_value  # type: ignore[attr-defined]
        update_robot()

    reset_button.on_click(reset_robot)
    update_robot()


def simulate(
    left_urdf: str | Path,
    right_urdf: str | Path,
    *,
    port: int = 8080,
) -> None:
    """Display two existing split URDFs side-by-side until interrupted."""

    if not 1 <= port <= 65535:
        raise SplitError("Viser port must be between 1 and 65535.")

    left_path = Path(left_urdf).expanduser().resolve()
    right_path = Path(right_urdf).expanduser().resolve()
    for label, path in (("left", left_path), ("right", right_path)):
        if not path.is_file():
            raise SplitError(
                f"The {label} URDF does not exist or is not a file: {path}"
            )

    try:
        import time

        import viser
        from viser.extras import ViserUrdf
    except ImportError as exc:
        raise SplitError(
            "Simulation requires Viser URDF support; install it with "
            "`python -m pip install 'viser[urdf]'`."
        ) from exc

    server = viser.ViserServer(host="127.0.0.1", port=port)
    try:
        server.initial_camera.position = (1.8, -2.4, 1.2)
        server.initial_camera.look_at = (0.0, 0.0, -0.25)
        server.scene.add_grid(
            "/grid",
            width=2.5,
            height=2.5,
            position=(0.0, 0.0, 0.0),
        )
        server.scene.add_frame(
            "/left",
            position=(0.0, 0.7, 0.0),
            axes_length=0.12,
            axes_radius=0.004,
        )
        server.scene.add_frame(
            "/right",
            position=(0.0, -0.7, 0.0),
            axes_length=0.12,
            axes_radius=0.004,
        )
        server.scene.add_label(
            "/left/label",
            "Left-arm output",
            position=(0.0, 0.0, 0.18),
        )
        server.scene.add_label(
            "/right/label",
            "Right-arm output",
            position=(0.0, 0.0, 0.18),
        )

        left_robot = ViserUrdf(
            server,
            urdf_or_path=left_path,
            root_node_name="/left/robot",
            mesh_color_override=(0.45, 0.65, 1.0, 0.55),
            collision_mesh_color_override=(1.0, 0.25, 0.05, 0.35),
            load_meshes=True,
            load_collision_meshes=True,
        )
        right_robot = ViserUrdf(
            server,
            urdf_or_path=right_path,
            root_node_name="/right/robot",
            mesh_color_override=(0.45, 0.9, 0.6, 0.55),
            collision_mesh_color_override=(1.0, 0.25, 0.05, 0.35),
            load_meshes=True,
            load_collision_meshes=True,
        )
        robots = (left_robot, right_robot)
        for robot in robots:
            robot.show_visual = True
            robot.show_collision = False

        visual_toggle = server.gui.add_checkbox(
            "Show visual geometry", initial_value=True
        )
        collision_toggle = server.gui.add_checkbox(
            "Show collision geometry", initial_value=False
        )

        def update_geometry_visibility(_: object) -> None:
            for robot in robots:
                robot.show_visual = bool(visual_toggle.value)
                robot.show_collision = bool(collision_toggle.value)

        visual_toggle.on_update(update_geometry_visibility)
        collision_toggle.on_update(update_geometry_visibility)
        _add_viser_joint_controls(server, left_robot, "left")
        _add_viser_joint_controls(server, right_robot, "right")

        print(f"\nViser viewer: http://127.0.0.1:{port}")
        print("The selected models are shown side-by-side. Press Ctrl+C to close.")
        while True:
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nViser viewer closed.")
    except Exception as exc:
        raise SplitError(f"Could not start the Viser viewer: {exc}") from exc
    finally:
        server.stop()


def _default_output(source: Path, label: str) -> Path:
    return source.with_name(f"{source.stem}-{label}{source.suffix}")


def _add_split_arguments(parser: argparse.ArgumentParser) -> None:
    """Add arguments used only when generating split URDFs.

    Args:
        parser: Split subcommand parser to configure.
    """

    parser.add_argument("urdf", type=Path, help="Source bimanual URDF.")
    parser.add_argument(
        "--left-first-joint",
        help="First actuator joint of the left arm.",
    )
    parser.add_argument(
        "--right-first-joint",
        help="First actuator joint of the right arm.",
    )
    parser.add_argument(
        "--left-end-joint",
        help=(
            "Last left-arm joint; required non-interactively if its subtree "
            "branches."
        ),
    )
    parser.add_argument(
        "--right-end-joint",
        help=(
            "Last right-arm joint; required non-interactively if its subtree "
            "branches."
        ),
    )
    parser.add_argument("--left-output", type=Path, help="Left output URDF path.")
    parser.add_argument("--right-output", type=Path, help="Right output URDF path.")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing output URDFs.",
    )
    parser.add_argument(
        "--visualize",
        "--viser",
        action="store_true",
        help="Open both generated URDFs side-by-side in Viser after writing them.",
    )
    parser.add_argument(
        "--viser-port",
        type=int,
        default=8080,
        help="Port for --visualize (default: %(default)s).",
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    raw_args = list(sys.argv[1:] if argv is None else argv)
    commands = {"split", "simulate"}
    if raw_args and raw_args[0] not in commands | {"-h", "--help"}:
        # Preserve the original `script.py SOURCE ...` split invocation.
        raw_args.insert(0, "split")

    parser = argparse.ArgumentParser(
        description="Split a bimanual URDF or view existing split URDFs."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    split_parser = subparsers.add_parser(
        "split",
        help="Generate one URDF per arm.",
        description=(
            "Generate one URDF per arm while retaining the complete robot as "
            "visual and collision geometry."
        ),
    )
    _add_split_arguments(split_parser)

    simulate_parser = subparsers.add_parser(
        "simulate",
        help="View two existing split URDFs without regenerating them.",
    )
    simulate_parser.add_argument("left_urdf", type=Path, help="Left-arm URDF.")
    simulate_parser.add_argument("right_urdf", type=Path, help="Right-arm URDF.")
    simulate_parser.add_argument(
        "--viser-port",
        type=int,
        default=8080,
        help="Viser port (default: %(default)s).",
    )
    return parser.parse_args(raw_args)


def main(argv: list[str] | None = None) -> int:
    """Run the split or simulation command.

    Args:
        argv: Optional arguments excluding the Python executable and module.

    Returns:
        Zero after the requested command completes.

    Raises:
        SplitError: If validation, splitting, or output fails.
    """

    args = _parse_args(argv)
    if args.command == "simulate":
        simulate(args.left_urdf, args.right_urdf, port=args.viser_port)
        return 0

    source = args.urdf.resolve()
    left_output = (args.left_output or _default_output(source, "left")).resolve()
    right_output = (args.right_output or _default_output(source, "right")).resolve()
    if left_output == right_output:
        raise SplitError("Left and right output paths must be different.")
    if source in {left_output, right_output}:
        raise SplitError("An output path must not overwrite the source URDF.")

    tree = _load_urdf(source)
    graph = _build_graph(tree.getroot())
    _validate_source_meshes(tree.getroot(), graph.root_link, source.parent)
    left_first = _prompt_joint(
        args.left_first_joint, "First actuator joint of the left arm: ", graph
    )
    right_first = _prompt_joint(
        args.right_first_joint, "First actuator joint of the right arm: ", graph
    )
    if left_first == right_first:
        raise SplitError("Left and right first actuator joints must be different.")

    left_selection = _resolve_arm_selection(
        graph,
        label="left",
        first_joint_name=left_first,
        end_joint_name=args.left_end_joint,
    )
    right_selection = _resolve_arm_selection(
        graph,
        label="right",
        first_joint_name=right_first,
        end_joint_name=args.right_end_joint,
    )

    left_tree, left_frozen, left_transmissions = _generate_arm_tree(
        tree, graph, left_selection, source.parent
    )
    right_tree, right_frozen, right_transmissions = _generate_arm_tree(
        tree, graph, right_selection, source.parent
    )

    _write_tree(left_tree, left_output, force=args.force)
    try:
        _write_tree(right_tree, right_output, force=args.force)
    except Exception:
        # Do not delete the successfully written left output: an existing file
        # may have been overwritten intentionally with --force.  Report the
        # partial result clearly instead.
        print(
            f"Left output was written before the right output failed: {left_output}",
            file=sys.stderr,
        )
        raise

    _print_summary(left_selection, left_frozen, left_transmissions, left_output, graph)
    _print_summary(
        right_selection, right_frozen, right_transmissions, right_output, graph
    )
    if args.visualize:
        simulate(left_output, right_output, port=args.viser_port)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SplitError as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
