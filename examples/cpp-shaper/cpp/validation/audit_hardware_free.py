"""Audit the C++ example for robot, network, and credential dependencies."""

from __future__ import annotations

import argparse
import platform
import re
import shutil
import subprocess
import tempfile
from pathlib import Path


def _parse_args() -> argparse.Namespace:
    """Parse executable, source, and asset paths.

    Returns:
        Parsed audit paths.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--assets-dir", type=Path, required=True)
    return parser.parse_args()


def _dependency_listing(binary: Path) -> str:
    """Return the platform-native dynamic dependency listing.

    Args:
        binary: Executable to inspect.

    Returns:
        Combined dependency-tool output, or an empty string when unavailable.
    """

    command = None
    if platform.system() == "Linux" and shutil.which("ldd"):
        command = ["ldd", str(binary)]
    elif platform.system() == "Darwin" and shutil.which("otool"):
        command = ["otool", "-L", str(binary)]
    if command is None:
        return ""
    return subprocess.run(
        command, check=True, capture_output=True, text=True
    ).stdout.lower()


def _network_syscall_lines(trace_text: str) -> list[str]:
    """Return actual network syscalls from a network-only `strace` capture.

    Modern `strace` versions append successful process-exit bookkeeping even
    when the selected syscall class is empty. Those markers are not syscalls.

    Args:
        trace_text: Complete network-only `strace` output.

    Returns:
        Trace lines that represent network activity or unexpected output.
    """

    return [
        line
        for line in trace_text.splitlines()
        if not re.fullmatch(r"(?:\d+\s+)?\+\+\+ exited with 0 \+\+\+", line)
    ]


def _run_network_syscall_audit(binary: Path, assets_dir: Path) -> bool:
    """Run a Linux network-syscall trace when `strace` is available.

    Args:
        binary: Executable to run.
        assets_dir: Frozen Standard Bots asset directory.

    Returns:
        True when a syscall trace was performed, otherwise false.
    """

    if platform.system() != "Linux" or shutil.which("strace") is None:
        return False
    with tempfile.TemporaryDirectory(prefix="shaper-network-audit-") as temporary:
        trace_path = Path(temporary) / "network.trace"
        completed = subprocess.run(
            [
                "strace",
                "-f",
                "-e",
                "trace=network",
                "-o",
                str(trace_path),
                str(binary),
                "--assets-dir",
                str(assets_dir),
                "--output-dir",
                str(Path(temporary) / "figures"),
                "--headless",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            raise AssertionError(
                "example failed under network syscall audit:\n"
                + completed.stdout
                + completed.stderr
            )
        trace_lines = _network_syscall_lines(trace_path.read_text())
        if trace_lines:
            raise AssertionError(
                "example issued a network syscall:\n" + "\n".join(trace_lines)
            )
    return True


def main() -> None:
    """Run static, link-closure, execution, and optional syscall audits."""

    args = _parse_args()
    source = args.source.read_text()
    forbidden_source_patterns = {
        "network API": r"\b(?:socket|connect|sendto|getaddrinfo|curl_easy_\w+)\s*\(",
        "robot or ROS header": (
            r"#include\s*[<\"](?:rclcpp|ros|standard_bots|robot_sdk)/"
        ),
        "credential access": r"\b(?:API_KEY|PASSWORD|TOKEN|CREDENTIALS)\b",
    }
    for label, pattern in forbidden_source_patterns.items():
        if re.search(pattern, source, flags=re.IGNORECASE):
            raise AssertionError(f"C++ example contains forbidden {label}")

    dependencies = _dependency_listing(args.binary)
    for forbidden in (
        "libcurl",
        "libssl",
        "libcrypto",
        "rclcpp",
        "ros2",
        "standard_bots",
    ):
        if forbidden in dependencies:
            raise AssertionError(f"binary links forbidden dependency: {forbidden}")

    with tempfile.TemporaryDirectory(prefix="shaper-hardware-audit-") as temporary:
        completed = subprocess.run(
            [
                str(args.binary),
                "--assets-dir",
                str(args.assets_dir),
                "--output-dir",
                str(Path(temporary) / "figures"),
                "--headless",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
    if completed.returncode != 0 or "Validation: PASS" not in completed.stdout:
        raise AssertionError(
            "hardware-free executable audit failed:\n"
            + completed.stdout
            + completed.stderr
        )
    traced = _run_network_syscall_audit(args.binary, args.assets_dir)
    print(
        "Hardware/network audit: PASS "
        f"(network syscall trace: {'performed' if traced else 'not available'})"
    )


if __name__ == "__main__":
    main()
