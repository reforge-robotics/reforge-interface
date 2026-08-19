"""Exercise installed-consumer failures and audit the executable closure."""

from __future__ import annotations

import argparse
import json
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    """Parse paths supplied by CTest.

    Returns:
        Installed-consumer validation inputs.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--assets-dir", type=Path, required=True)
    parser.add_argument("--expected-metrics", type=Path, required=True)
    parser.add_argument("--package-dir", type=Path, required=True)
    parser.add_argument("--cmake-prefix-path", default="")
    return parser.parse_args()


def _run(
    command: list[str], *, env: dict[str, str] | None = None
) -> subprocess.CompletedProcess[str]:
    """Run a command while capturing diagnostics.

    Args:
        command: Complete subprocess argument vector.
        env: Optional subprocess environment.

    Returns:
        Completed process including stdout and stderr.
    """

    return subprocess.run(command, check=False, capture_output=True, text=True, env=env)


def _require_failure(
    command: list[str],
    expected_text: str,
    *,
    env: dict[str, str] | None = None,
) -> None:
    """Require a command to fail with one actionable diagnostic.

    Args:
        command: Complete subprocess argument vector.
        expected_text: Case-insensitive diagnostic fragment.
        env: Optional subprocess environment.

    Raises:
        AssertionError: If the command succeeds or omits the diagnostic.
    """

    completed = _run(command, env=env)
    diagnostic = completed.stdout + completed.stderr
    if completed.returncode == 0:
        raise AssertionError(f"command unexpectedly succeeded: {' '.join(command)}")
    if expected_text.lower() not in diagnostic.lower():
        raise AssertionError(
            f"failure did not mention {expected_text!r}:\n{diagnostic}"
        )


def _audit_dynamic_dependencies(binary: Path) -> None:
    """Reject unresolved, private, or unqualified runtime dependencies.

    Args:
        binary: Example executable to inspect.

    Raises:
        AssertionError: If the executable dependency closure is unsafe.
    """

    if platform.system() == "Linux" and shutil.which("ldd"):
        command = ["ldd", str(binary)]
    elif platform.system() == "Darwin" and shutil.which("otool"):
        command = ["otool", "-L", str(binary)]
    else:
        return
    completed = _run(command)
    if completed.returncode != 0:
        raise AssertionError(completed.stdout + completed.stderr)
    dependency_lines = (completed.stdout + completed.stderr).splitlines()
    if platform.system() == "Darwin" and dependency_lines:
        dependency_lines = dependency_lines[1:]
    dependency_listing = "\n".join(dependency_lines).lower()
    forbidden_fragments = (
        "not found",
        "libtorch",
        "torch/lib",
        "site-packages/torch",
        "/reforge-core/build/",
        "/reforge-core/src/",
    )
    for fragment in forbidden_fragments:
        if fragment in dependency_listing:
            raise AssertionError(
                f"executable has forbidden dependency fragment {fragment!r}:\n"
                f"{dependency_listing}"
            )


def _validate_configure_failures(args: argparse.Namespace, temporary: Path) -> None:
    """Verify missing installed-package and plotting dependencies fail clearly.

    Args:
        args: Installed-consumer validation inputs.
        temporary: Isolated directory for negative CMake configurations.
    """

    clean_env = os.environ.copy()
    clean_env.pop("CMAKE_PREFIX_PATH", None)
    clean_env.pop("LD_LIBRARY_PATH", None)
    _require_failure(
        [
            "cmake",
            "-S",
            str(args.source_dir),
            "-B",
            str(temporary / "missing-package"),
            "-DCMAKE_DISABLE_FIND_PACKAGE_ReforgeShaper=TRUE",
        ],
        "ReforgeShaper",
        env=clean_env,
    )

    plotting_command = [
        "cmake",
        "-S",
        str(args.source_dir),
        "-B",
        str(temporary / "missing-plotting"),
        f"-DReforgeShaper_DIR={args.package_dir}",
        "-DCMAKE_DISABLE_FIND_PACKAGE_Python3=TRUE",
    ]
    if args.cmake_prefix_path:
        plotting_command.append(f"-DCMAKE_PREFIX_PATH={args.cmake_prefix_path}")
    _require_failure(plotting_command, "Python3", env=clean_env)


def _validate_artifact_failures(args: argparse.Namespace, temporary: Path) -> None:
    """Verify missing and malformed deterministic artifacts fail clearly.

    Args:
        args: Installed-consumer validation inputs.
        temporary: Isolated directory containing copied asset variants.
    """

    for relative_path in (
        Path("model/model_bundle.json"),
        Path("model/shaper_models.native.json"),
        Path("modelone.urdf"),
    ):
        copied_assets = temporary / f"missing-{relative_path.name}"
        shutil.copytree(args.assets_dir, copied_assets)
        (copied_assets / relative_path).unlink()
        _require_failure(
            [
                str(args.binary),
                "--assets-dir",
                str(copied_assets),
                "--headless",
                "--output-dir",
                str(temporary / "unused-output"),
            ],
            "assets directory must contain",
        )

    malformed_assets = temporary / "malformed-model"
    shutil.copytree(args.assets_dir, malformed_assets)
    (malformed_assets / "model/shaper_models.native.json").write_text(
        "{ malformed native model", encoding="utf-8"
    )
    completed = _run(
        [
            str(args.binary),
            "--assets-dir",
            str(malformed_assets),
            "--headless",
            "--output-dir",
            str(temporary / "malformed-output"),
        ]
    )
    malformed_diagnostic = completed.stdout + completed.stderr
    if (
        completed.returncode == 0
        or "not valid json" not in malformed_diagnostic.lower()
    ):
        raise AssertionError(
            "malformed native model did not produce an actionable failure:\n"
            + malformed_diagnostic
        )


def _validate_baseline_mismatch(args: argparse.Namespace, temporary: Path) -> None:
    """Verify a frozen-baseline identity mismatch exits nonzero.

    Args:
        args: Installed-consumer validation inputs.
        temporary: Isolated directory for the changed manifest.
    """

    mismatched_manifest = temporary / "mismatched-metrics.json"
    expected = json.loads(args.expected_metrics.read_text(encoding="utf-8"))
    expected["dominant_natural_frequency_rad_s"] = 0.0
    mismatched_manifest.write_text(json.dumps(expected), encoding="utf-8")
    _require_failure(
        [
            str(args.binary),
            "--assets-dir",
            str(args.assets_dir),
            "--baseline",
            str(mismatched_manifest),
            "--headless",
            "--output-dir",
            str(temporary / "baseline-mismatch-output"),
        ],
        "dominant natural frequency failed Standard Bots parity",
    )

    missing_manifest = temporary / "missing-metrics.json"
    _require_failure(
        [str(args.binary), "--baseline", str(missing_manifest), "--headless"],
        "baseline manifest is missing or unreadable",
    )
    malformed_manifest = temporary / "malformed-metrics.json"
    malformed_manifest.write_text("{ malformed baseline", encoding="utf-8")
    _require_failure(
        [str(args.binary), "--baseline", str(malformed_manifest), "--headless"],
        "baseline manifest is missing numeric member",
    )

    expected["dominant_natural_frequency_rad_s"] = json.loads(
        args.expected_metrics.read_text(encoding="utf-8")
    )["dominant_natural_frequency_rad_s"]
    expected["artifact_sha256"] = "0" * 64
    mismatched_manifest.write_text(json.dumps(expected), encoding="utf-8")
    _require_failure(
        [
            os.environ.get("PYTHON", sys.executable),
            str(args.source_dir / "validation/compare_with_python.py"),
            "--binary",
            str(args.binary),
            "--assets-dir",
            str(args.assets_dir),
            "--expected-metrics",
            str(mismatched_manifest),
        ],
        "not the frozen Standard Bots bundle",
    )


def main() -> None:
    """Run installed-consumer negative cases and dependency inspection."""

    args = _parse_args()
    _audit_dynamic_dependencies(args.binary)
    with tempfile.TemporaryDirectory(prefix="shaper-consumer-validation-") as raw:
        temporary = Path(raw)
        _validate_configure_failures(args, temporary)
        _validate_artifact_failures(args, temporary)
        _validate_baseline_mismatch(args, temporary)
    print("Installed consumer failures and dependency audit: PASS")


if __name__ == "__main__":
    main()
