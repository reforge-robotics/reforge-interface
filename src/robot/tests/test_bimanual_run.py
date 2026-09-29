"""Focused tests for explicit bimanual URDF preparation in the CLI."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from robot import run


def _configure_cli(
    monkeypatch: pytest.MonkeyPatch,
    package_dir: Path,
    *,
    use_left: bool,
) -> tuple[Path, Path]:
    """Point the CLI's package resources at an isolated fixture directory."""
    source = package_dir / "urdf" / "fixture.urdf"
    side = "left" if use_left else "right"
    selected = package_dir / "urdf" / f"fixture-{side}.urdf"
    source.parent.mkdir(parents=True, exist_ok=True)
    source.write_text("<robot name='fixture'/>", encoding="utf-8")

    monkeypatch.setattr(run, "__file__", str(package_dir / "run.py"))
    monkeypatch.setattr(run, "files", lambda _package: package_dir)
    monkeypatch.setattr(run.robot_interface, "BASE_URDF_PATH", "urdf/fixture.urdf")
    monkeypatch.setattr(
        run.robot_interface,
        "URDF_PATH",
        f"urdf/fixture-{'left' if use_left else 'right'}.urdf",
    )
    monkeypatch.setattr(run.robot_interface, "USE_LEFT", use_left)
    return source, selected


def _argument_path(arguments: list[str], option: str) -> Path:
    return Path(arguments[arguments.index(option) + 1])


@pytest.mark.parametrize("use_left", [True, False])
def test_bimanual_calibrate_splits_and_dispatches_selected_arm(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    use_left: bool,
) -> None:
    """The flag prepares both arms, then calibration receives the selected one."""
    _source, selected = _configure_cli(monkeypatch, tmp_path, use_left=use_left)
    split_calls: list[list[str]] = []
    calibration_calls: list[dict[str, Any]] = []

    def split(arguments: list[str]) -> int:
        split_calls.append(arguments)
        left_output = _argument_path(arguments, "--left-output")
        right_output = _argument_path(arguments, "--right-output")
        left_output.write_text("left", encoding="utf-8")
        right_output.write_text("right", encoding="utf-8")
        return 0

    def calibrate(**kwargs: Any) -> str:
        calibration_calls.append(kwargs)
        return "calibrated"

    monkeypatch.setattr(run.split_urdf, "main", split)
    monkeypatch.setattr(run.run_helpers, "main", calibrate)

    result = run.main(["calibrate", "127.0.0.1", "--bimanual", "--robot_id", "fixture"])

    assert result == "calibrated"
    assert len(split_calls) == 1
    assert Path(split_calls[0][1]) == _source
    assert len(calibration_calls) == 1
    dispatched = calibration_calls[0]
    assert dispatched["argv"] == ["calibrate", "127.0.0.1", "--robot_id", "fixture"]
    assert Path(dispatched["simulator_configuration"].urdf_path) == selected


def test_existing_pair_skips_split(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A complete pair is reused without prompting the splitter."""
    source = tmp_path / "fixture.urdf"
    left = tmp_path / "fixture-left.urdf"
    right = tmp_path / "fixture-right.urdf"
    for path in (source, left, right):
        path.write_text(path.name, encoding="utf-8")

    def unexpected_split(_arguments: list[str]) -> int:
        pytest.fail("splitter ran despite both arm URDFs already existing")

    monkeypatch.setattr(run.split_urdf, "main", unexpected_split)

    run._ensure_bimanual_urdfs(source, right, use_left=False)

    assert left.read_text(encoding="utf-8") == left.name
    assert right.read_text(encoding="utf-8") == right.name


def test_missing_arm_splits_without_overwriting_existing_arm(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The splitter fills a missing arm while preserving the existing output."""
    source = tmp_path / "fixture.urdf"
    left = tmp_path / "fixture-left.urdf"
    right = tmp_path / "fixture-right.urdf"
    source.write_text("source", encoding="utf-8")
    right.write_text("keep this right model", encoding="utf-8")
    calls: list[list[str]] = []

    def split(arguments: list[str]) -> int:
        calls.append(arguments)
        split_left = _argument_path(arguments, "--left-output")
        split_right = _argument_path(arguments, "--right-output")
        assert split_left == left
        assert split_right != right
        split_left.write_text("generated left", encoding="utf-8")
        split_right.write_text("temporary right", encoding="utf-8")
        return 0

    monkeypatch.setattr(run.split_urdf, "main", split)

    run._ensure_bimanual_urdfs(source, right, use_left=False)

    assert len(calls) == 1
    assert left.read_text(encoding="utf-8") == "generated left"
    assert right.read_text(encoding="utf-8") == "keep this right model"


def test_failed_split_stops_before_calibration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A split error prevents calibration dispatch."""
    _configure_cli(monkeypatch, tmp_path, use_left=False)
    calibration_calls: list[dict[str, Any]] = []
    monkeypatch.setattr(run.split_urdf, "main", lambda _arguments: 2)
    monkeypatch.setattr(
        run.run_helpers,
        "main",
        lambda **kwargs: calibration_calls.append(kwargs),
    )

    with pytest.raises(RuntimeError, match="split failed with status 2"):
        run.main(["calibrate", "127.0.0.1", "--bimanual"])

    assert calibration_calls == []


def test_split_input_error_exits_cleanly_before_calibration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Interactive splitter errors become concise CLI errors, not tracebacks."""
    _configure_cli(monkeypatch, tmp_path, use_left=False)
    calibration_calls: list[dict[str, Any]] = []

    def fail_split(_arguments: list[str]) -> int:
        raise run.split_urdf.SplitError("joint missing")

    monkeypatch.setattr(run.split_urdf, "main", fail_split)
    monkeypatch.setattr(
        run.run_helpers,
        "main",
        lambda **kwargs: calibration_calls.append(kwargs),
    )

    with pytest.raises(SystemExit, match="error: joint missing"):
        run.main(["calibrate", "127.0.0.1", "--bimanual"])

    assert calibration_calls == []


@pytest.mark.parametrize("robot_ip", ["sim", "127.0.0.1"])
def test_plain_calibration_uses_base_urdf_without_splitting(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    robot_ip: str,
) -> None:
    """Unflagged calibration selects the base model for both simulator and hardware."""
    source, selected = _configure_cli(monkeypatch, tmp_path, use_left=False)
    dispatches: list[dict[str, Any]] = []
    hardware_kwargs: list[dict[str, Any]] = []

    def unexpected_split(_arguments: list[str]) -> int:
        pytest.fail("Unflagged calibration must not split the base URDF")

    class DummyInterface:
        """Capture the model selected for the hardware factory."""

        def __init__(self, **kwargs: Any) -> None:
            hardware_kwargs.append(kwargs)

        def close(self) -> None:
            """Allow the CLI resource stack to close the test interface."""

    def dispatch(**kwargs: Any) -> None:
        dispatches.append(kwargs)
        if robot_ip != "sim":
            kwargs["robot_interface_class"](robot_ip=robot_ip)

    monkeypatch.setattr(run.split_urdf, "main", unexpected_split)
    monkeypatch.setattr(run.robot_interface, "RobotInterface", DummyInterface)
    monkeypatch.setattr(run.run_helpers, "main", dispatch)

    run.main(["calibrate", robot_ip])

    assert len(dispatches) == 1
    dispatched = dispatches[0]
    assert Path(dispatched["simulator_configuration"].urdf_path) == source
    assert Path(dispatched["kinecal_source_urdf_path"]) == source
    assert not selected.exists()
    if robot_ip == "sim":
        assert hardware_kwargs == []
    else:
        assert len(hardware_kwargs) == 1
        assert hardware_kwargs[0]["urdf_path"] == run.robot_interface.BASE_URDF_PATH


def test_bimanual_sim_calibration_leaves_first_joints_to_splitter(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The flag invokes the splitter without preselecting arm joints."""
    source, selected = _configure_cli(monkeypatch, tmp_path, use_left=False)
    split_calls: list[list[str]] = []
    dispatches: list[dict[str, Any]] = []

    def split(arguments: list[str]) -> int:
        split_calls.append(arguments)
        _argument_path(arguments, "--left-output").write_text("left", encoding="utf-8")
        _argument_path(arguments, "--right-output").write_text(
            "right", encoding="utf-8"
        )
        return 0

    monkeypatch.setattr(run.split_urdf, "main", split)
    monkeypatch.setattr(
        run.run_helpers, "main", lambda **kwargs: dispatches.append(kwargs)
    )

    run.main(["calibrate", "sim", "--bimanual", "--nv", "1", "--nr", "1"])

    assert len(split_calls) == 1
    split_args = split_calls[0]
    assert Path(split_args[1]) == source
    assert _argument_path(split_args, "--left-output") == source.with_name(
        "fixture-left.urdf"
    )
    assert _argument_path(split_args, "--right-output") == selected
    assert "--left-first-joint" not in split_args
    assert "--right-first-joint" not in split_args
    assert len(dispatches) == 1
    assert Path(dispatches[0]["simulator_configuration"].urdf_path) == selected
    assert dispatches[0]["argv"] == ["calibrate", "sim", "--nv", "1", "--nr", "1"]


def test_validation_sim_does_not_split_missing_arm_models(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Validation requires an existing URDF and does not trigger splitting."""
    source, selected = _configure_cli(monkeypatch, tmp_path, use_left=False)

    def unexpected_split(_arguments: list[str]) -> int:
        pytest.fail("Validation must not split arm URDFs")

    monkeypatch.setattr(run.split_urdf, "main", unexpected_split)

    with pytest.raises(FileNotFoundError, match="Robot URDF resource not found"):
        run.main(
            [
                "joint_tracker_performance_validation",
                "sim",
                "--mode",
                "single-speed",
                "--urdf-path",
                str(selected),
            ]
        )

    assert source.exists()
    assert not selected.exists()


def test_sim_calibration_forwards_scan_options_without_injecting_defaults(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Preserve explicit options and leave the SDK defaults to the SDK."""
    source, selected = _configure_cli(monkeypatch, tmp_path, use_left=False)
    dispatched: list[dict[str, Any]] = []

    def unexpected_split(_arguments: list[str]) -> int:
        pytest.fail("Unflagged simulator calibration must not split the URDF")

    monkeypatch.setattr(run.split_urdf, "main", unexpected_split)
    monkeypatch.setattr(
        run.run_helpers, "main", lambda **kwargs: dispatched.append(kwargs)
    )

    scan_options = ["--axes", "2", "--maxfreq=3", "--nv", "1", "--nr", "1"]
    run.main(["calibrate", "sim", *scan_options])
    run.main(["calibrate", "sim"])

    assert len(dispatched) == 2
    assert dispatched[0]["argv"] == ["calibrate", "sim", *scan_options]
    assert dispatched[1]["argv"] == ["calibrate", "sim"]
    assert all(
        Path(call["simulator_configuration"].urdf_path) == source
        for call in dispatched
    )
    assert not selected.exists()
