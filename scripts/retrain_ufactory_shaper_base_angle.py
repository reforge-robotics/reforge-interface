#!/usr/bin/env python3
"""Augment and retrain the public UF850 Shaper models across base angles.

Run from the ``reforge-interface`` checkout with the matching Reforge Core
Python environment activated. The source captures remain read-only; synthetic
static metadata and generated calibration maps are written below ignored robot
data, and trained artifacts go to a separate ignored staging directory until
they are explicitly promoted.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import pickle
import sys
from pathlib import Path
from typing import Any

import numpy as np
import torch


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ROBOT_ROOT = REPOSITORY_ROOT / "src" / "robot"
DATA_ROOT = ROBOT_ROOT / "data"
MODEL_ROOT = ROBOT_ROOT / "models" / "current" / "shaper"
DEFAULT_SOURCE = DATA_ROOT / "combined-payloads-2026-8-26-to-30"
DEFAULT_STAGE = DATA_ROOT / "ufactory-2026-8-26-to-30-base-angle-all-axis"
DEFAULT_TRAINED_MODELS = DATA_ROOT / "ufactory-2026-8-26-to-30-base-angle-all-axis-models"
DEFAULT_CORE_ROOT = REPOSITORY_ROOT.parent / "reforge-core"
URDF_PATH = ROBOT_ROOT / "urdf" / "uf850.urdf"
TRAINING_EPOCHS = 40_000
TRAINING_SEED = 20261029
BASE_JOINT_LOWER_RAD = -2.0 * math.pi
BASE_JOINT_UPPER_RAD = 2.0 * math.pi
MAX_DISPLACEMENT_RAD = math.radians(10.0)
ANGLE_COUNT = 9


def _sha256(path: Path) -> str:
    """Return a file's SHA-256 digest."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _angle_grid_rad() -> tuple[float, ...]:
    """Build nine evenly spaced angles inside UF850 joint1's 10-degree margin."""
    return tuple(
        float(value)
        for value in np.linspace(
            BASE_JOINT_LOWER_RAD + MAX_DISPLACEMENT_RAD,
            BASE_JOINT_UPPER_RAD - MAX_DISPLACEMENT_RAD,
            ANGLE_COUNT,
        )
    )


def _copy_angle_tagged_static(
    *, source: Path, destination: Path, angle_rad: float
) -> None:
    """Copy one static calibration row while assigning synthetic base metadata."""
    with source.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"Static metadata has no header: {source}")
        rows = list(reader)
    if len(rows) != 1:
        raise ValueError(f"Expected one static metadata row in {source}")
    row = rows[0]
    required = {"run_role", "swept_axis", "reference_base_joint", "base_j0_angle"}
    missing = sorted(required - set(row))
    if missing:
        raise ValueError(f"{source} is missing static metadata fields {missing}")
    row["run_role"] = "base_only_axis"
    row["swept_axis"] = "0"
    row["reference_base_joint"] = "0"
    row["base_j0_angle"] = format(angle_rad, ".17g")
    with destination.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=reader.fieldnames)
        writer.writeheader()
        writer.writerow(row)


def prepare_augmented_dataset(source: Path, destination: Path) -> dict[str, Any]:
    """Create a schema-v3 synthetic dataset without changing source captures.

    Normal runs remain at their measured zero-angle reference. Each recorded
    every measured joint-axis sweep is reused as a synthetic
    ``base_only_axis`` row at every nonzero grid angle; zero-angle observations
    remain as normal runs. The training interpretation is rotational symmetry,
    so only static base-angle metadata changes while measured motion and
    vibration targets are reused.
    """
    source = source.resolve()
    destination = destination.resolve()
    manifest_path = source / "calibration_manifest.json"
    parameter_path = source / "identification_parameters.csv"
    if not manifest_path.is_file() or not parameter_path.is_file():
        raise FileNotFoundError(f"Missing manifest or parameters under {source}")
    if destination.exists():
        raise FileExistsError(
            f"Staging directory already exists: {destination}. Choose a new path."
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if int(manifest.get("schema_version", 0)) != 1:
        raise ValueError("Expected the documented schema-v1 combined corpus")
    if manifest.get("base_joint_count") != 1:
        raise ValueError("Expected one UF850 base joint")
    if manifest.get("reference_base_angles_rad") != [0.0]:
        raise ValueError("Expected the source corpus to use base angle 0 rad")

    angles = _angle_grid_rad()
    if len(angles) != ANGLE_COUNT or not any(abs(angle) < 1e-12 for angle in angles):
        raise ValueError("The configured angle grid must contain nine angles and zero")
    runs = manifest.get("runs")
    if not isinstance(runs, list):
        raise ValueError("Source manifest runs must be a list")
    normal_runs: list[dict[str, Any]] = []
    base_runs: list[dict[str, Any]] = []
    for run in runs:
        if run.get("run_role") != "normal_full_axis":
            raise ValueError("Unexpected non-normal run in the source corpus")
        if int(run.get("swept_axis", -1)) != int(run.get("axis_index", -2)):
            raise ValueError(f"Inconsistent source axis metadata in {run.get('run_id')}")
        motion_source = source / str(run["motion_filename"])
        static_source = source / str(run["static_filename"])
        if not motion_source.is_file() or not static_source.is_file():
            raise FileNotFoundError(f"Source run has missing artifacts: {run.get('run_id')}")
        normal = dict(run)
        normal["motion_filename"] = motion_source.name
        normal["static_filename"] = static_source.name
        normal_runs.append(normal)
        # The original normal run already supplies the zero-angle sample.
        for grid_index, angle_rad in enumerate(angles):
            if abs(angle_rad) < 1e-12:
                continue
            synthetic_static_name = (
                f"synthetic_static_a{grid_index:02d}_"
                f"{str(run['run_id']).replace('/', '_')}.csv"
            )
            synthetic_static_path = destination / synthetic_static_name
            # Defer writes until the staging folder has been created below.
            synthetic = dict(run)
            synthetic.update(
                {
                    "run_id": f"synthetic-a{grid_index:02d}-{run['run_id']}",
                    "run_role": "base_only_axis",
                    # Base-only metadata represents the virtual base-angle
                    # setting; preserve the original motion axis separately.
                    "base_joint_angles_rad": [angle_rad],
                    "synthetic_source_axis": int(run["swept_axis"]),
                    "swept_axis": 0,
                    "motion_filename": motion_source.name,
                    "static_filename": synthetic_static_name,
                    "pose_index": len(runs) + len(base_runs),
                    "status": "completed",
                    "synthetic_source_run_id": run["run_id"],
                }
            )
            base_runs.append(synthetic)

    destination.mkdir(parents=True)
    if source.stat().st_dev != destination.stat().st_dev:
        raise OSError("Source and staging folders must share a filesystem for hard links")
    linked_names = {
        str(run[field])
        for run in normal_runs
        for field in ("motion_filename", "static_filename")
    }
    for name in sorted(linked_names):
        os.link(source / name, destination / name)
    for run in base_runs:
        source_run = next(
            item
            for item in normal_runs
            if item["run_id"] == run["synthetic_source_run_id"]
        )
        static_source = source / Path(source_run["static_filename"]).name
        angle_grid_index = next(
            index
            for index, angle in enumerate(angles)
            if math.isclose(angle, run["base_joint_angles_rad"][0], abs_tol=1e-12)
        )
        _copy_angle_tagged_static(
            source=static_source,
            destination=destination / run["static_filename"],
            angle_rad=run["base_joint_angles_rad"][0],
        )
        run.pop("synthetic_source_run_id")
        run["synthetic_grid_index"] = angle_grid_index

    output_manifest = dict(manifest)
    output_manifest.update(
        {
            "schema_version": 3,
            "base_joint_angle_count": ANGLE_COUNT,
            "base_joint_indices": [0],
            "base_joint_limits_rad": [[BASE_JOINT_LOWER_RAD, BASE_JOINT_UPPER_RAD]],
            "base_joint_grid_rad": [list(angles)],
            "base_joint_grid_margin_source": ["max_displacement"],
            "reference_base_angles_rad": [0.0],
            "runs": normal_runs + base_runs,
            "synthetic_angle_augmentation": {
                "method": "reuse all measured axis sweeps with angle-tagged static metadata",
                "symmetry_assumption": "all UF850 Shaper axis responses are rotationally symmetric about joint1",
                "source_dataset": str(source),
                "source_manifest_sha256": _sha256(manifest_path),
                "source_identification_parameters_sha256": _sha256(parameter_path),
                "source_run_count": len(normal_runs),
                "augmented_source_run_count": len(normal_runs),
                "synthetic_angle_count": len(angles) - 1,
                "synthetic_run_count": len(base_runs),
                "angles_rad": list(angles),
                "created_by": Path(__file__).name,
            },
        }
    )
    (destination / "calibration_manifest.json").write_text(
        json.dumps(output_manifest, indent=2) + "\n", encoding="utf-8"
    )

    with parameter_path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = list(reader.fieldnames or [])
        parameter_rows = list(reader)
    if len(parameter_rows) != 1:
        raise ValueError("Expected one identification parameter row")
    params = parameter_rows[0]
    params.update(
        {
            "calibration_schema_version": "3",
            "base_joint_count": "1",
            "base_joint_angle_count": str(ANGLE_COUNT),
            "base_joint_indices": "0",
            "base_joint_reference_angles": "0",
            "base_joint_grid_flat": ";".join(format(a, ".17g") for a in angles),
            "feature_names": "j0_rad;v_deg;r_mm;inertia",
            "feature_units": "rad;deg;mm;kg_m2",
            "planned_run_count": str(len(normal_runs) + len(base_runs)),
        }
    )
    with (destination / "identification_parameters.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerow(params)

    return {
        "path": str(destination),
        "angles_rad": list(angles),
        "source_runs": len(normal_runs),
        "synthetic_runs": len(base_runs),
        "total_runs": len(normal_runs) + len(base_runs),
        "source_manifest_sha256": _sha256(manifest_path),
    }


def _architecture_from_torchscript(path: Path) -> tuple[int, ...]:
    """Recover a model's hidden widths from its TorchScript body layers."""
    model = torch.jit.load(str(path), map_location="cpu")
    weights = [
        (int(name.split(".")[1]), tuple(parameter.shape))
        for name, parameter in model.named_parameters()
        if name.startswith("body.") and name.endswith(".weight")
    ]
    weights.sort(key=lambda item: item[0])
    if not weights:
        raise ValueError(f"No hidden Linear layers found in {path}")
    widths = tuple(shape[0] for _, shape in weights)
    input_width = weights[0][1][1]
    if input_width != 4:
        raise ValueError(f"{path} expects {input_width} features; expected 4")
    return widths


def prepare_augmented_maps(
    *, dataset: Path, source_dataset: Path, core_root: Path, chunk_size: int = 32
) -> dict[str, Any]:
    """Expand saved baseline maps by cloning only base-axis rows.

    Reuses the documented 277-row map corpus so transfer functions and FRFs
    are not recomputed. Every source map digest and row/run alignment is checked
    before the augmented ignored map shards are written beside the dataset.
    """
    core_source = core_root / "src"
    sdk_source = core_source / "core_sdk"
    sys.path.insert(0, str(sdk_source))
    sys.path.insert(0, str(core_source))

    from reforge_cloud_api.identification.map_generation import CalibrationMap
    from reforge_cloud_api.identification.process_calibration_data import (
        load_calibration_maps,
    )
    from reforge_core.util.utility import read_identification_parameters

    dataset = dataset.resolve()
    source_dataset = source_dataset.resolve()
    params = read_identification_parameters(data_folder=str(dataset))
    params.saved_maps = True
    params.max_map_size = chunk_size
    params.start_pose = 0
    params.poses = int(params.planned_run_count)
    source_params = read_identification_parameters(data_folder=str(source_dataset))
    source_params.max_map_size = chunk_size
    source_params.start_pose = 0
    source_params.poses = int(source_params.planned_run_count)
    map_paths = load_calibration_maps(str(source_dataset), source_params)
    source_manifest = json.loads(
        (source_dataset / "calibration_manifest.json").read_text(encoding="utf-8")
    )
    augmented_manifest = json.loads(
        (dataset / "calibration_manifest.json").read_text(encoding="utf-8")
    )
    expected_source_ids = [str(run["run_id"]) for run in source_manifest["runs"]]
    source_rows: list[tuple[Any, int]] = []
    map_hashes: dict[str, str] = {}
    for map_path in map_paths:
        path = Path(map_path)
        map_hashes[path.name] = _sha256(path)
        with path.open("rb") as stream:
            calibration_map = pickle.load(stream)
        for row_index in range(calibration_map.initialPositions.shape[0]):
            source_rows.append((calibration_map, row_index))

    if len(source_rows) != len(expected_source_ids):
        raise ValueError(
            f"Saved-map row count {len(source_rows)} does not match source manifest "
            f"run count {len(expected_source_ids)}"
        )
    for row_index, (calibration_map, local_row) in enumerate(source_rows):
        row_metadata = calibration_map.featureMetadataByPoseAxis[local_row]
        present = [entry for entry in row_metadata if entry is not None]
        if len(present) != 1 or present[0].get("source_run_id") != expected_source_ids[row_index]:
            raise ValueError(
                f"Saved-map row {row_index} does not match manifest order: "
                f"expected={expected_source_ids[row_index]!r}, metadata={present!r}"
            )

    grid = _angle_grid_rad()
    synthetic_runs = [
        run
        for run in augmented_manifest["runs"]
        if run.get("run_role") == "base_only_axis"
    ]
    synthetic_by_source_and_angle = {
        (
            str(run["run_id"]).partition("-")[2].partition("-")[2],
            int(run["synthetic_grid_index"]),
        ): run
        for run in synthetic_runs
    }
    expanded_rows: list[tuple[Any, int, dict[str, Any] | None]] = [
        (calibration_map, local_row, None)
        for calibration_map, local_row in source_rows
    ]
    cloned_count = 0
    for source_row_index, (calibration_map, local_row) in enumerate(source_rows):
        row_metadata = calibration_map.featureMetadataByPoseAxis[local_row]
        present = [entry for entry in row_metadata if entry is not None]
        if len(present) != 1:
            raise ValueError(f"Source map row {source_row_index} must have exactly one axis")
        source_metadata = present[0]
        measured_axis = int(source_metadata["source_axis_index"])
        source_run_id = str(source_metadata["source_run_id"])
        # Original static measurements were acquired at nominal zero; normalize
        # encoder tolerance to the persisted reference angle for this feature.
        normalized = dict(source_metadata)
        normalized["base_joint_angles_rad"] = [0.0]
        expanded_rows[source_row_index] = (
            calibration_map,
            local_row,
            {"axis_metadata": normalized, "axis": measured_axis},
        )
        for grid_index, angle in enumerate(grid):
            if abs(angle) < 1e-12:
                continue
            synthetic_run = synthetic_by_source_and_angle.get((source_run_id, grid_index))
            if synthetic_run is None:
                raise ValueError(
                    f"Missing synthetic manifest row for {source_run_id} at grid {grid_index}"
                )
            metadata = dict(source_metadata)
            metadata.update(
                {
                    "source_run_id": synthetic_run["run_id"],
                    "run_role": "base_only_axis",
                    "swept_axis": 0,
                    "base_joint_angles_rad": [angle],
                    "synthetic_source_run_id": source_run_id,
                    "synthetic_source_axis": measured_axis,
                    "synthetic_grid_index": grid_index,
                }
            )
            expanded_rows.append((calibration_map, local_row, metadata))
            cloned_count += 1

    expected_total = len(expected_source_ids) + len(synthetic_runs)
    if len(expanded_rows) != expected_total or cloned_count != len(synthetic_runs):
        raise ValueError(
            f"Expanded rows mismatch: total={len(expanded_rows)}/{expected_total}, "
            f"synthetic={cloned_count}/{len(synthetic_runs)}"
        )

    # Remove stale map shards only when their names match this generated output.
    # All source captures/maps are in a separate directory and are untouched.
    for stale in dataset.glob("UFACTORY_robot_calibration_map_lastPose*_numAxes3_startPose*.pkl"):
        stale.unlink()
    for start in range(0, len(expanded_rows), chunk_size):
        end = min(start + chunk_size, len(expanded_rows))
        chunk = CalibrationMap(
            numPositions=end - start,
            axesCommanded=int(params.axes),
            numJoints=int(params.num_joints),
        )
        for output_row, (source_map, source_row, metadata_override) in enumerate(
            expanded_rows[start:end]
        ):
            for field in (
                "allWn",
                "allZeta",
                "allInertia",
                "allV",
                "allR",
                "initialPositions",
            ):
                getattr(chunk, field)[output_row] = getattr(source_map, field)[source_row]
            chunk.sineSweepNumerators[output_row] = pickle.loads(
                pickle.dumps(source_map.sineSweepNumerators[source_row])
            )
            chunk.sineSweepDenominators[output_row] = pickle.loads(
                pickle.dumps(source_map.sineSweepDenominators[source_row])
            )
            metadata = pickle.loads(
                pickle.dumps(source_map.featureMetadataByPoseAxis[source_row])
            )
            if metadata_override is not None:
                if "axis_metadata" in metadata_override:
                    metadata[metadata_override["axis"]] = metadata_override[
                        "axis_metadata"
                    ]
                else:
                    metadata[int(metadata_override["source_axis_index"])] = (
                        metadata_override
                    )
            chunk.featureMetadataByPoseAxis[output_row] = metadata
        filename = (
            f"UFACTORY_robot_calibration_map_lastPose{end}_numAxes3_startPose{start}.pkl"
        )
        with (dataset / filename).open("wb") as stream:
            pickle.dump(chunk, stream)

    # Validate loader's expected chunk boundaries and all row-level angle labels.
    augmented_params = read_identification_parameters(data_folder=str(dataset))
    augmented_params.max_map_size = chunk_size
    augmented_params.start_pose = 0
    augmented_params.poses = expected_total
    output_paths = load_calibration_maps(str(dataset), augmented_params)
    if len(output_paths) != math.ceil(expected_total / chunk_size):
        raise ValueError("Generated map shard count does not match the loader layout")
    output_ids: list[str] = []
    angle_counts_by_axis = {
        axis: {format(angle, ".17g"): 0 for angle in grid} for axis in range(3)
    }
    for map_path in output_paths:
        with open(map_path, "rb") as stream:
            calibration_map = pickle.load(stream)
        for row_metadata in calibration_map.featureMetadataByPoseAxis:
            present = [entry for entry in row_metadata if entry is not None]
            if len(present) != 1:
                raise ValueError("Augmented map row must contain exactly one measured axis")
            entry = present[0]
            output_ids.append(str(entry["source_run_id"]))
            angle_key = format(float(entry["base_joint_angles_rad"][0]), ".17g")
            source_axis = int(entry.get("synthetic_source_axis", entry["source_axis_index"]))
            angle_counts_by_axis[source_axis][angle_key] += 1
    if len(output_ids) != expected_total or len(set(output_ids)) != expected_total:
        raise ValueError("Generated map rows are not a unique one-to-one manifest mapping")
    expected_per_axis = {0: 91, 1: 93, 2: 93}
    expected_angle_counts = {
        axis: {format(angle, ".17g"): expected_per_axis[axis] for angle in grid}
        for axis in range(3)
    }
    if angle_counts_by_axis != expected_angle_counts:
        raise ValueError(
            f"Unexpected per-axis base-angle row counts: {angle_counts_by_axis}"
        )

    report = {
        "dataset": str(dataset),
        "source_dataset": str(source_dataset),
        "source_manifest_sha256": _sha256(source_dataset / "calibration_manifest.json"),
        "source_map_sha256": map_hashes,
        "source_rows": len(source_rows),
        "synthetic_rows": cloned_count,
        "output_rows": expected_total,
        "output_map_shards": len(output_paths),
        "chunk_size": chunk_size,
        "base_angle_row_counts_by_axis": angle_counts_by_axis,
    }
    (dataset / "augmented_map_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def train_augmented_models(
    *,
    dataset: Path,
    output: Path,
    source_models: Path,
    urdf_path: Path,
    core_root: Path,
) -> dict[str, Any]:
    """Train fixed per-axis architectures and publish a staged mixed bundle."""
    core_source = core_root / "src"
    sdk_source = core_source / "core_sdk"
    if not (core_source / "reforge_cloud_api").is_dir() or not (
        sdk_source / "reforge_core"
    ).is_dir():
        raise FileNotFoundError(f"Could not resolve Reforge Core source at {core_root}")
    sys.path.insert(0, str(sdk_source))
    sys.path.insert(0, str(core_source))

    from reforge_cloud_api.identification.calibration_metadata import (
        load_shaper_calibration_lineage,
    )
    from reforge_cloud_api.identification.map_fitter import MapFitter
    from reforge_cloud_api.identification.process_calibration_data import (
        DEFAULT_MAX_MAP_POSES,
        load_calibration_maps,
        publish_shaper_bundle,
        resolve_calibration_processing_layout,
    )
    from reforge_core.util.utility import read_identification_parameters

    if output.exists():
        raise FileExistsError(f"Training output already exists: {output}")
    args = read_identification_parameters(data_folder=str(dataset))
    lineage = load_shaper_calibration_lineage(dataset, parameters=args)
    layout = resolve_calibration_processing_layout(str(dataset), args)
    args.poses = layout.processing_row_count
    args.max_map_size = min(
        int(getattr(args, "max_map_size", DEFAULT_MAX_MAP_POSES)), args.poses
    )
    args.max_map_size = min(args.max_map_size, 32)
    args.start_pose = int(getattr(args, "start_pose", 0))
    args.saved_maps = True
    map_names = load_calibration_maps(data_folder=str(dataset), args=args)
    if not map_names:
        raise RuntimeError("Calibration map generation returned no maps")

    architectures = {
        axis: _architecture_from_torchscript(
            source_models / f"axis{axis}_model.pt"
        )
        for axis in range(int(args.axes))
    }
    axis_entries: dict[str, Any] = {}
    axis_fit_summaries: dict[str, Any] = {}
    for target_axis, hidden in architectures.items():
        print(
            f"[FIT] target_axis={target_axis} hidden={hidden} "
            f"epochs={TRAINING_EPOCHS} use_optuna=False",
            flush=True,
        )
        np.random.seed(TRAINING_SEED + target_axis)
        torch.manual_seed(TRAINING_SEED + target_axis)
        fitter = MapFitter(
            map_names=map_names,
            num_positions=layout.processing_row_count,
            axes_commanded=int(args.axes),
            num_joints=int(args.num_joints),
            **lineage.to_map_fitter_kwargs(),
            nn_saturation_frequency_hz=float(args.max_freq - args.freq_space),
        )
        fitter.fit_shaper_neural_network_twohead(
            hidden=hidden,
            epochs=TRAINING_EPOCHS,
            use_optuna=False,
        )
        axis_entries[f"axis{target_axis}"] = fitter.nn_models[f"axis{target_axis}"]
        axis_fit_summaries[f"axis{target_axis}"] = {
            "hidden": list(hidden),
            "epochs": TRAINING_EPOCHS,
            "use_optuna": False,
            "seed": TRAINING_SEED + target_axis,
        }
        # Retain completed axis passes for recovery and audit.
        pass_directory = output / f"fit_axis{target_axis}"
        fitter.save_models_torchscript(str(pass_directory))

    combined = MapFitter(
        map_names=map_names,
        num_positions=layout.processing_row_count,
        axes_commanded=int(args.axes),
        num_joints=int(args.num_joints),
        **lineage.to_map_fitter_kwargs(),
        nn_saturation_frequency_hz=float(args.max_freq - args.freq_space),
    )
    combined.nn_models = axis_entries
    combined.models_exist = True
    output.mkdir(parents=True, exist_ok=True)
    bundle_directory = output / "bundle"
    publish_shaper_bundle(
        map_fitter=combined,
        directory=str(bundle_directory),
        cloud_folder=str(dataset),
        output_sensor=str(args.output_sensor),
        num_axes=int(args.axes),
        lineage=lineage,
    )
    report = {
        "dataset": str(dataset.resolve()),
        "source_models": str(source_models.resolve()),
        "bundle": str(bundle_directory.resolve()),
        "augmentation_report": json.loads(
            (dataset / "augmented_map_report.json").read_text(encoding="utf-8")
        ),
        "angle_grid_rad": _angle_grid_rad(),
        "epochs": TRAINING_EPOCHS,
        "use_optuna": False,
        "seed_base": TRAINING_SEED,
        "architecture_by_axis": axis_fit_summaries,
        "processing_rows": layout.processing_row_count,
        "calibration_map_count": len(map_names),
        "pr_546_compatibility": {
            "trained_on_multiple_base_angles_field_emitted": False,
            "reason": (
                "The current Reforge Core checkout predates the optional PR #546 "
                "bundle field; omitting it preserves compatibility with older "
                "strict runtime parsers. PR #546 derives that flag from only the "
                "base-prefix axis models, while this retraining covers every "
                "per-axis model across the nine-angle grid."
            ),
        },
    }
    (output / "training_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    return report


def compare_staged_models(
    *,
    dataset: Path,
    trained_bundle: Path,
    source_models: Path,
    core_root: Path,
) -> dict[str, Any]:
    """Compare current and staged models at zero and synthetic off-angles.

    The measured labels are only available at the original near-zero base
    setting. Off-angle predictions therefore quantify model sensitivity under
    the explicit symmetry assumption; they are not physical validation data.
    """
    core_source = core_root / "src"
    sdk_source = core_source / "core_sdk"
    sys.path.insert(0, str(sdk_source))
    sys.path.insert(0, str(core_source))

    from reforge_cloud_api.identification.calibration_metadata import (
        load_shaper_calibration_lineage,
    )
    from reforge_cloud_api.identification.map_fitter import MapFitter
    from reforge_cloud_api.identification.process_calibration_data import (
        load_calibration_maps,
        resolve_calibration_processing_layout,
    )
    from reforge_core.util.utility import read_identification_parameters

    dataset = dataset.resolve()
    trained_bundle = trained_bundle.resolve()
    source_models = source_models.resolve()
    args = read_identification_parameters(data_folder=str(dataset))
    args.poses = int(args.planned_run_count)
    args.max_map_size = min(int(getattr(args, "max_map_size", 32)), 32)
    args.start_pose = 0
    args.saved_maps = True
    lineage = load_shaper_calibration_lineage(dataset, parameters=args)
    layout = resolve_calibration_processing_layout(str(dataset), args)
    map_names = load_calibration_maps(str(dataset), args)
    fitter = MapFitter(
        map_names=map_names,
        num_positions=layout.processing_row_count,
        axes_commanded=int(args.axes),
        num_joints=int(args.num_joints),
        **lineage.to_map_fitter_kwargs(),
        nn_saturation_frequency_hz=float(args.max_freq - args.freq_space),
    )

    feature_index = fitter.feature_schema.feature_names.index("j0_rad")
    def _predict(model: torch.jit.ScriptModule, features: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        with torch.inference_mode():
            order_prediction, modes = model(torch.from_numpy(features.astype(np.float32)))
        return order_prediction.numpy(), modes.numpy()
    axis_reports: dict[str, Any] = {}
    for axis in range(int(args.axes)):
        axis_metadata = fitter._feature_metadata_by_axis_row[axis]
        observed_indices = [
            index
            for index, item in enumerate(axis_metadata)
            if item is not None and item.get("run_role") == "normal_full_axis"
        ]
        expected_rows = {0: 91, 1: 93, 2: 93}[axis]
        if len(observed_indices) != expected_rows:
            raise ValueError(
                f"Expected {expected_rows} measured axis-{axis} rows, "
                f"found {len(observed_indices)}"
            )
        measured_features = fitter._build_axis_feature_matrix(axis)[
            observed_indices
        ].copy()
        measured_features[:, feature_index] = 0.0
        def _log_damping(values: np.ndarray) -> np.ndarray:
            with np.errstate(divide="ignore", invalid="ignore"):
                return np.log(np.asarray(values, dtype=np.float64) + 1e-6)

        targets = np.column_stack(
            [
                np.asarray(fitter.wn1[axis], dtype=np.float64)[observed_indices],
                _log_damping(
                    np.asarray(fitter.zeta1[axis], dtype=np.float64)[observed_indices]
                ),
                np.asarray(fitter.wn2[axis], dtype=np.float64)[observed_indices],
                _log_damping(
                    np.asarray(fitter.zeta2[axis], dtype=np.float64)[observed_indices]
                ),
            ]
        )
        target_mask = np.isfinite(targets)
        class_targets = (
            np.asarray(fitter.order[axis], dtype=np.int64)[observed_indices] == 2
        ).astype(np.float32)
        current = torch.jit.load(
            str(source_models / f"axis{axis}_model.pt"), map_location="cpu"
        )
        staged = torch.jit.load(
            str(trained_bundle / f"axis{axis}_model.pt"), map_location="cpu"
        )
        current.eval()
        staged.eval()
        current_zero_order, current_zero_modes = _predict(current, measured_features)
        staged_zero_order, staged_zero_modes = _predict(staged, measured_features)

        def _regression_mse(prediction: np.ndarray) -> float:
            squared = (prediction - targets) ** 2
            return float(np.nanmean(np.where(target_mask, squared, np.nan)))

        def _classification_metrics(prediction: np.ndarray) -> dict[str, float]:
            probabilities = np.clip(prediction.reshape(-1), 1e-7, 1.0 - 1e-7)
            bce = -np.mean(
                class_targets * np.log(probabilities)
                + (1.0 - class_targets) * np.log(1.0 - probabilities)
            )
            accuracy = np.mean((probabilities >= 0.5) == (class_targets >= 0.5))
            return {"bce": float(bce), "accuracy": float(accuracy)}

        sensitivity_by_model: dict[str, dict[str, float]] = {}
        for model_name, model in (("current", current), ("staged", staged)):
            deltas: list[np.ndarray] = []
            probability_deltas: list[np.ndarray] = []
            for angle in _angle_grid_rad():
                if abs(angle) < 1e-12:
                    continue
                off_angle_features = measured_features.copy()
                off_angle_features[:, feature_index] = angle
                order_prediction, mode_prediction = _predict(model, off_angle_features)
                zero_modes = (
                    current_zero_modes if model_name == "current" else staged_zero_modes
                )
                zero_order = (
                    current_zero_order if model_name == "current" else staged_zero_order
                )
                deltas.append(np.abs(mode_prediction - zero_modes))
                probability_deltas.append(np.abs(order_prediction - zero_order))
            all_mode_deltas = np.concatenate(deltas, axis=0)
            all_probability_deltas = np.concatenate(probability_deltas, axis=0)
            sensitivity_by_model[model_name] = {
                "mean_absolute_mode_delta": float(np.mean(all_mode_deltas)),
                "p95_absolute_mode_delta": float(np.quantile(all_mode_deltas, 0.95)),
                "mean_absolute_order_probability_delta": float(
                    np.mean(all_probability_deltas)
                ),
            }
        axis_reports[f"axis{axis}"] = {
            "measured_zero_angle_rows": len(observed_indices),
            "zero_angle_measured_labels": {
                "current_regression_mse": _regression_mse(current_zero_modes),
                "staged_regression_mse": _regression_mse(staged_zero_modes),
                "current_classification": _classification_metrics(current_zero_order),
                "staged_classification": _classification_metrics(staged_zero_order),
            },
            "off_angle_sensitivity": sensitivity_by_model,
        }

    report = {
        "source_model_sha256": {
            f"axis{axis}_model.pt": _sha256(source_models / f"axis{axis}_model.pt")
            for axis in range(3)
        },
        "staged_bundle": str(trained_bundle),
        "measured_zero_angle_rows_by_axis": {
            axis: {0: 91, 1: 93, 2: 93}[axis] for axis in range(int(args.axes))
        },
        "feature_schema": list(fitter.feature_schema.feature_names),
        "grid_rad": list(_angle_grid_rad()),
        "labels_note": (
            "Only the source near-zero setting is measured; off-angle predictions "
            "are evaluated under the synthetic rotational-symmetry assumption."
        ),
        "per_axis": axis_reports,
    }
    report_path = trained_bundle.parent / "model_comparison.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return {"report_path": str(report_path), **report}


def main() -> None:
    """Run dataset augmentation or the full fixed-architecture retraining."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    augment = subparsers.add_parser(
        "augment", help="Build the ignored synthetic dataset"
    )
    augment.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    augment.add_argument("--output", type=Path, default=DEFAULT_STAGE)
    maps = subparsers.add_parser(
        "prepare-maps", help="Expand validated precomputed baseline map rows"
    )
    maps.add_argument("--dataset", type=Path, default=DEFAULT_STAGE)
    maps.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    maps.add_argument("--core-root", type=Path, default=DEFAULT_CORE_ROOT)
    maps.add_argument("--chunk-size", type=int, default=32)
    train = subparsers.add_parser("train", help="Generate maps and retrain staged models")
    train.add_argument("--dataset", type=Path, default=DEFAULT_STAGE)
    train.add_argument("--output", type=Path, default=DEFAULT_TRAINED_MODELS)
    train.add_argument("--source-models", type=Path, default=MODEL_ROOT)
    train.add_argument("--urdf", type=Path, default=URDF_PATH)
    train.add_argument("--core-root", type=Path, default=DEFAULT_CORE_ROOT)
    compare = subparsers.add_parser(
        "compare", help="Compare staged and current inference at zero/off angles"
    )
    compare.add_argument("--dataset", type=Path, default=DEFAULT_STAGE)
    compare.add_argument(
        "--bundle", type=Path, default=DEFAULT_TRAINED_MODELS / "bundle"
    )
    compare.add_argument("--source-models", type=Path, default=MODEL_ROOT)
    compare.add_argument("--core-root", type=Path, default=DEFAULT_CORE_ROOT)
    args = parser.parse_args()

    if args.command == "augment":
        result = prepare_augmented_dataset(args.source, args.output)
    elif args.command == "prepare-maps":
        result = prepare_augmented_maps(
            dataset=args.dataset,
            source_dataset=args.source,
            core_root=args.core_root,
            chunk_size=args.chunk_size,
        )
    elif args.command == "compare":
        result = compare_staged_models(
            dataset=args.dataset,
            trained_bundle=args.bundle,
            source_models=args.source_models,
            core_root=args.core_root,
        )
    else:
        result = train_augmented_models(
            dataset=args.dataset.resolve(),
            output=args.output.resolve(),
            source_models=args.source_models.resolve(),
            urdf_path=args.urdf.resolve(),
            core_root=args.core_root.resolve(),
        )
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
