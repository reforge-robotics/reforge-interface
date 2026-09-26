"""Focused regression tests for one-time URDF preparation."""

from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET

import pytest

from robot import split_urdf

_TEST_LINKS = {
    "base",
    "left_arm_link",
    "left_tcp",
    "right_arm_link",
    "right_tcp",
}


def _write_ascii_stl(path: Path) -> None:
    """Write a small valid triangle mesh for the fixture's link geometry.

    Args:
        path: Destination STL path.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        """solid fixture
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 0.01 0 0
      vertex 0 0.01 0
    endloop
  endfacet
endsolid fixture
""",
        encoding="utf-8",
    )


def _add_mesh(link: ET.Element, role: str, filename: str) -> None:
    """Add one mesh-backed visual or collision element to a link.

    Args:
        link: Link element that owns the geometry.
        role: Geometry role, either ``visual`` or ``collision``.
        filename: Mesh filename stored in the URDF.
    """
    element = ET.SubElement(link, role)
    geometry = ET.SubElement(element, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {"filename": filename, "scale": "0.001 0.001 0.001"},
    )


def _add_link(
    robot: ET.Element,
    name: str,
    filename: str,
    *,
    roles: tuple[str, ...] = ("visual", "collision"),
) -> None:
    """Add a link with inertial data and the selected mesh geometry roles.

    Args:
        robot: Fixture robot element.
        name: New link name.
        filename: Mesh filename used by every selected role.
        roles: Geometry roles to add.
    """
    link = ET.SubElement(robot, "link", {"name": name})
    if name != "base":
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", {"value": "0.1"})
        ET.SubElement(
            inertial,
            "inertia",
            {
                "ixx": "0.001",
                "ixy": "0",
                "ixz": "0",
                "iyy": "0.001",
                "iyz": "0",
                "izz": "0.001",
            },
        )
    for role in roles:
        _add_mesh(link, role, filename)


def _add_revolute_joint(robot: ET.Element, name: str, child: str) -> None:
    """Add a bounded shoulder joint used by the two arm branches.

    Args:
        robot: Fixture robot element.
        name: Joint name.
        child: Child link name.
    """
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "revolute"})
    ET.SubElement(joint, "parent", {"link": "base"})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
    ET.SubElement(
        joint,
        "limit",
        {"lower": "-3.2", "upper": "3.2", "effort": "10", "velocity": "2"},
    )


def _create_bimanual_urdf(
    directory: Path,
    *,
    root_roles: tuple[str, ...] = (),
) -> Path:
    """Create a small, valid bimanual URDF and its shared mesh asset.

    All non-root links use one mesh for both visual and collision geometry.
    A package URI and a relative path both resolve to the same file.

    Args:
        directory: Directory that receives the fixture URDF and mesh.
        root_roles: Geometry roles included on the graph root.

    Returns:
        Path to the generated fixture URDF.
    """
    _write_ascii_stl(directory / "meshes" / "tiny.stl")
    robot = ET.Element("robot", {"name": "test_bimanual"})
    _add_link(robot, "base", "meshes/tiny.stl", roles=root_roles)
    _add_link(robot, "left_arm_link", "meshes/tiny.stl")
    _add_link(robot, "left_tcp", "package://fixture_package/meshes/tiny.stl")
    _add_link(robot, "right_arm_link", "meshes/tiny.stl")
    _add_link(robot, "right_tcp", "package://fixture_package/meshes/tiny.stl")

    _add_revolute_joint(robot, "left_shoulder", "left_arm_link")
    left_tcp_joint = ET.SubElement(
        robot, "joint", {"name": "left_tcp_mount", "type": "fixed"}
    )
    ET.SubElement(left_tcp_joint, "parent", {"link": "left_arm_link"})
    ET.SubElement(left_tcp_joint, "child", {"link": "left_tcp"})
    ET.SubElement(left_tcp_joint, "origin", {"xyz": "1 0 0", "rpy": "0 0 0"})

    _add_revolute_joint(robot, "right_shoulder", "right_arm_link")
    right_tcp_joint = ET.SubElement(
        robot, "joint", {"name": "right_tcp_mount", "type": "fixed"}
    )
    ET.SubElement(right_tcp_joint, "parent", {"link": "right_arm_link"})
    ET.SubElement(right_tcp_joint, "child", {"link": "right_tcp"})
    ET.SubElement(right_tcp_joint, "origin", {"xyz": "1 0 0", "rpy": "0 0 0"})

    source = directory / "fixture.urdf"
    ET.ElementTree(robot).write(source, encoding="utf-8", xml_declaration=True)
    return source


def _split(source: Path, *, force: bool = False) -> None:
    """Generate both arm models from the fixture without interactive input.

    Args:
        source: Fixture source URDF.
        force: Whether existing generated files may be replaced.
    """
    arguments = [
        "split",
        str(source),
        "--left-first-joint",
        "left_shoulder",
        "--right-first-joint",
        "right_shoulder",
    ]
    if force:
        arguments.append("--force")
    assert split_urdf.main(arguments) == 0


def _replace_geometry_with_box(source: Path, link_name: str, role: str) -> None:
    """Replace one link's mesh geometry with a primitive to test mesh checks.

    Args:
        source: Fixture source URDF.
        link_name: Link whose geometry is changed.
        role: Geometry role changed to a primitive.
    """
    tree = ET.parse(source)
    link = tree.getroot().find(f"./link[@name='{link_name}']")
    assert link is not None
    geometry = link.find(f"./{role}/geometry")
    assert geometry is not None
    for child in list(geometry):
        geometry.remove(child)
    ET.SubElement(geometry, "box", {"size": "0.01 0.01 0.01"})
    tree.write(source, encoding="utf-8", xml_declaration=True)


@pytest.mark.parametrize("role", ["visual", "collision"])
def test_each_link_requires_mesh_backed_visual_and_collision(
    tmp_path: Path,
    role: str,
) -> None:
    """Reject a non-root link whose visual or collision has no mesh.

    Args:
        tmp_path: Isolated fixture directory.
        role: Geometry role replaced by a primitive.
    """
    source = _create_bimanual_urdf(tmp_path)
    _replace_geometry_with_box(source, "left_arm_link", role)

    with pytest.raises(split_urdf.SplitError) as error:
        _split(source)

    assert "left_arm_link" in str(error.value)
    assert role in str(error.value).lower()


@pytest.mark.parametrize("role", ["visual", "collision"])
def test_only_geometryless_root_is_exempt(tmp_path: Path, role: str) -> None:
    """Require both mesh roles when the root has only one geometry role.

    Args:
        tmp_path: Isolated fixture directory.
        role: The sole geometry role included on the root.
    """
    source = _create_bimanual_urdf(tmp_path, root_roles=(role,))

    with pytest.raises(split_urdf.SplitError) as error:
        _split(source)

    assert "base" in str(error.value)
    missing_role = "collision" if role == "visual" else "visual"
    assert missing_role in str(error.value).lower()


def test_missing_mesh_file_fails_with_link_context(tmp_path: Path) -> None:
    """Reject an unresolved mesh file and identify its owning link.

    Args:
        tmp_path: Isolated fixture directory.
    """
    source = _create_bimanual_urdf(tmp_path)
    tree = ET.parse(source)
    link = tree.getroot().find("./link[@name='left_arm_link']")
    assert link is not None
    for mesh in link.findall("./visual/geometry/mesh") + link.findall(
        "./collision/geometry/mesh"
    ):
        mesh.set("filename", "meshes/missing.stl")
    tree.write(source, encoding="utf-8", xml_declaration=True)

    with pytest.raises(split_urdf.SplitError) as error:
        _split(source)

    assert "left_arm_link" in str(error.value)
    assert "missing.stl" in str(error.value)


def test_split_converts_mesh_paths_preserves_source_and_keeps_split_behavior(
    tmp_path: Path,
) -> None:
    """Keep the input unchanged while converting meshes and preserving both arms.

    Args:
        tmp_path: Isolated fixture directory.
    """
    source = _create_bimanual_urdf(tmp_path)
    source_bytes = source.read_bytes()
    expected_mesh = (tmp_path / "meshes" / "tiny.stl").resolve()

    _split(source)

    assert source.read_bytes() == source_bytes
    for side in ("left", "right"):
        output = tmp_path / f"fixture-{side}.urdf"
        generated = ET.parse(output).getroot()
        link_names = {link.get("name") for link in generated.findall("link")}
        assert _TEST_LINKS <= link_names
        mesh_paths = [
            mesh.get("filename") for mesh in generated.findall("./link/*/geometry/mesh")
        ]
        assert mesh_paths
        assert set(mesh_paths) == {str(expected_mesh)}
        assert all(Path(path).is_file() for path in mesh_paths if path is not None)

        joints = {
            joint.get("name"): joint.get("type") for joint in generated.findall("joint")
        }
        assert joints[f"{side}_shoulder"] == "revolute"
        other_side = "right" if side == "left" else "left"
        assert joints[f"{other_side}_shoulder"] == "fixed"
        assert any(
            joint.get("type") == "fixed"
            and joint.find("parent") is not None
            and joint.find("parent").get("link") == f"__split_{side}_arm_base"
            and joint.find("child") is not None
            and joint.find("child").get("link") == "base"
            for joint in generated.findall("joint")
        )
