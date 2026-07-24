"""
Smoke test that the Python bindings of all roboplan_ros packages are importable.

This catches packaging issues for downstream packages that may use the bindings.
"""

import importlib

import pytest

BINDINGS_MODULES = [
    "roboplan_ros.cpp",
    "roboplan_ros.visualization",
]


@pytest.mark.parametrize("module", BINDINGS_MODULES)
def test_import_bindings(module: str) -> None:
    importlib.import_module(module)
