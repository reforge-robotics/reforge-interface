"""Checks for accidental private deployment data in the customer tree."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def _text_files() -> list[Path]:
    roots = [
        ROOT / "src",
        ROOT / "docker_scripts",
        ROOT / "ui-widget",
        ROOT / "README.md",
        ROOT / "MANIFEST.in",
        ROOT / "pyproject.toml",
        ROOT / "install_reforge_shaper.sh",
        ROOT / "run_shaper_example.sh",
    ]
    paths: list[Path] = []
    for root in roots:
        candidates = root.rglob("*") if root.is_dir() else [root]
        paths.extend(
            path
            for path in candidates
            if path.is_file()
            and path != Path(__file__).resolve()
            and "__pycache__" not in path.parts
            and path.suffix.lower()
            in {
                ".html",
                ".md",
                ".py",
                ".sh",
                ".toml",
                ".xml",
                ".json",
                ".cpp",
                ".hpp",
            }
        )
    return sorted(set(paths))


def test_customer_tree_contains_no_recorded_private_literals() -> None:
    forbidden_patterns = (
        re.compile(r"(?i)\bapi(?:rr)?_live_[a-z0-9_-]{20,}\b"),
        re.compile(
            r"\b(?=[a-z0-9-]*\d)[a-z0-9]{5,8}(?:-[a-z0-9]{5,8}){3,}\b",
            re.IGNORECASE,
        ),
        re.compile(r"\b10\.0\.0\.\d+(?::\d+)?\b"),
        re.compile(r"\bbot_[a-z0-9]{12,}\b", re.IGNORECASE),
        re.compile(r"(?i)/(?:Users/nosed|home/ipereira)(?:/|$)"),
        re.compile(r"standard_bots_qualification"),
        re.compile(r"reforge_standard_bots_qualification"),
    )
    for path in _text_files():
        content = path.read_text(errors="ignore")
        assert not [
            pattern.pattern for pattern in forbidden_patterns if pattern.search(content)
        ]


def test_selected_model_manifest_matches_assets() -> None:
    model_dir = ROOT / "src/robot/models/current/shaper"
    manifest = json.loads(
        (model_dir / "standard_bots_model_manifest.json").read_text()
    )
    assert manifest["trajectory_contract"]["sample_period_s"] == 0.005
    assert manifest["trajectory_contract"]["point_count"] == 1053
    assert manifest["robot"]["joint_count"] == 6
    for artifact in manifest["model"]["axis_artifacts"]:
        path = model_dir / artifact["file"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == artifact["sha256"]
    for key in ("native_artifact", "bundle"):
        path = model_dir / manifest["model"][key]
        hash_key = f"{key}_sha256"
        assert (
            hashlib.sha256(path.read_bytes()).hexdigest()
            == manifest["model"][hash_key]
        )
    urdf = ROOT / "src/robot/urdf/modelone.urdf"
    assert (
        hashlib.sha256(urdf.read_bytes()).hexdigest()
        == manifest["urdf"]["sha256"]
    )
