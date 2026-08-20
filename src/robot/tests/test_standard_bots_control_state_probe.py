"""Tests for the read-only Standard Bots ROS-control probe."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pytest

from robot.standard_bots_control_state_probe import _is_enabled, _sdk_url


def test_sdk_url_normalizes_endpoint() -> None:
    assert _sdk_url("10.0.0.4:3000") == "http://10.0.0.4:3000"
    assert _sdk_url("https://robot.example") == "https://robot.example"
    with pytest.raises(ValueError, match="ENDPOINT"):
        _sdk_url("")


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        ("Enabled", True),
        ("ROSControlStateEnum.Enabled", True),
        (SimpleNamespace(state="Enabled"), True),
        ("Disabled", False),
        (SimpleNamespace(state="Disabled"), False),
    ],
)
def test_is_enabled_is_exact(value: object, expected: bool) -> None:
    assert _is_enabled(value) is expected


def test_probe_imports_sdk_only_when_queried(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delitem(sys.modules, "standardbots", raising=False)
    assert _is_enabled("Enabled")
