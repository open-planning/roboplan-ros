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

from ._cpp_ext import (  # noqa: F401
    JointMapping,
    JointStateConverterMap,
    buildConversionMap,
    fromJointState,
    fromJointTrajectory,
    fromTransformStamped,
    poseToSE3,
    se3ToPose,
    toJointState,
    toJointTrajectory,
    toTransformStamped,
)

__all__ = [
    "JointMapping",
    "JointStateConverterMap",
    "buildConversionMap",
    "fromJointState",
    "fromJointTrajectory",
    "fromTransformStamped",
    "poseToSE3",
    "se3ToPose",
    "toJointState",
    "toJointTrajectory",
    "toTransformStamped",
]
