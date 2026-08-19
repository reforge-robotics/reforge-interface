"""Validate deterministic headless images from the C++ Shaper example."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
from typing import Any

from PIL import Image


def _parse_args() -> argparse.Namespace:
    """Parse plot-regression command-line arguments.

    Returns:
        Parsed binary, asset, and expected-manifest paths.
    """

    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--assets-dir", type=Path, required=True)
    parser.add_argument("--expected", type=Path, required=True)
    return parser.parse_args()


def _difference_hash(image: Image.Image) -> int:
    """Compute a 64-bit difference hash insensitive to small pixel drift.

    Args:
        image: Rendered image to reduce to luminance gradients.

    Returns:
        Perceptual hash encoded as an integer.
    """

    luminance = image.convert("L").resize((9, 8), Image.Resampling.LANCZOS)
    pixels = list(luminance.getdata())
    result = 0
    for row in range(8):
        for column in range(8):
            result <<= 1
            result |= pixels[row * 9 + column] > pixels[row * 9 + column + 1]
    return result


def _validate_image(path: Path, expected: dict[str, Any]) -> None:
    """Validate one PNG's dimensions, content, and perceptual distance.

    Args:
        path: Generated PNG path.
        expected: Frozen dimensions, hash, and distance threshold.

    Raises:
        AssertionError: If the image does not match the frozen contract.
    """

    if path.stat().st_size < expected["minimum_bytes"]:
        raise AssertionError(f"plot is unexpectedly small: {path}")
    with Image.open(path) as image:
        if list(image.size) != expected["dimensions_px"]:
            raise AssertionError(
                f"{path.name} dimensions {image.size} do not match "
                f"{expected['dimensions_px']}"
            )
        actual_hash = _difference_hash(image)
    expected_hash = int(expected["difference_hash"], 16)
    distance = (actual_hash ^ expected_hash).bit_count()
    if distance > expected["maximum_hash_distance"]:
        raise AssertionError(
            f"{path.name} perceptual distance {distance} exceeds "
            f"{expected['maximum_hash_distance']}"
        )


def main() -> None:
    """Run the headless example and validate exactly three generated PNGs."""

    args = _parse_args()
    expected = json.loads(args.expected.read_text(encoding="utf-8"))
    with tempfile.TemporaryDirectory(prefix="shaper-plot-regression-") as temporary:
        temporary_path = Path(temporary)
        output_directory = temporary_path / "output"
        environment = os.environ.copy()
        environment["MPLBACKEND"] = "Agg"
        environment["MPLCONFIGDIR"] = str(temporary_path / "matplotlib")
        completed = subprocess.run(
            [
                str(args.binary),
                "--assets-dir",
                str(args.assets_dir),
                "--output-dir",
                str(output_directory),
                "--headless",
            ],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        if completed.returncode != 0:
            raise AssertionError(
                "headless plot run failed:\n" + completed.stdout + completed.stderr
            )
        if len(re.findall(r"residual vibration:", completed.stdout)) != 6:
            raise AssertionError(
                "C++ output must contain six controller-on/off residual comparisons"
            )

        generated = {path.name: path for path in output_directory.glob("*.png")}
        expected_names = set(expected["images"])
        if set(generated) != expected_names:
            raise AssertionError(
                f"generated PNG set {sorted(generated)} does not match "
                f"{sorted(expected_names)}"
            )
        for filename, image_contract in expected["images"].items():
            _validate_image(generated[filename], image_contract)

    print("Headless plot regression: PASS (3 images, perceptual threshold met)")


if __name__ == "__main__":
    main()
