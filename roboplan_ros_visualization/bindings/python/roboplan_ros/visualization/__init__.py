import os

# Python >= 3.8 on Windows does not search PATH for the DLLs an extension
# module links against, and in a colcon workspace they live outside this
# package. Register the PATH entries explicitly before importing.
if os.name == "nt":
    for _entry in os.environ.get("PATH", "").split(os.pathsep):
        if _entry and os.path.isdir(_entry):
            os.add_dll_directory(_entry)

# Import dependencies to guarantee types are registered before use.
from roboplan.core import Scene  # noqa: F401
from roboplan.simple_ik import _simple_ik_ext  # noqa: F401

from ._visualization_ext import (
    markerFromJointTrajectory,
    RoboplanVisualizer,
    RoboplanIKMarker,
)

__all__ = ["markerFromJointTrajectory", "RoboplanVisualizer", "RoboplanIKMarker"]
