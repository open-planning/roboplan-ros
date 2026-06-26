from collections.abc import Callable, Sequence
from typing import Annotated

import numpy
from numpy.typing import NDArray
import roboplan.core._core_ext


class RoboplanVisualizer:
    """
    Tool to build RViz MarkerArray messages from a RoboPlan scene and joint configuration.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, urdf_xml: str, frame_id: str = 'world', ns: str = '/roboplan', group_name: str = '', color: object | None = None) -> None: ...

    def markers_from_configuration(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> object:
        """
        Compute marker array for the given joint configuration, restricted to the currently selected joint group (see the constructor and set_group).
        """

    def set_group(self, group_name: str) -> None:
        """
        Select the joint group whose links are rendered. Pass an empty string for the whole scene. Raises if the group name is not found in the scene.
        """

    def clear_markers(self) -> object:
        """Return a MarkerArray that deletes all previously published markers."""

    def set_color(self, color: object) -> None:
        """Set a color override for all geometry markers."""

    def clear_color(self) -> None:
        """Remove the color override, reverting to per-geometry colors."""

class RoboplanIKMarker:
    """
    IK solver backend with interactive marker support for 6-DOF pose control.
    """

    def __init__(self, scene: roboplan.core._core_ext.Scene, base_link: str, tip_link: str, ik_solve_fn: Callable) -> None: ...

    def construct_imarker(self) -> object:
        """Build an InteractiveMarker message for the current target pose."""

    def process_feedback(self, feedback: object) -> object:
        """
        Process InteractiveMarkerFeedback. Returns joint positions on success, or else None.
        """

    def set_seed_configuration(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the seed joint positions for the next solve."""

def markerFromJointTrajectory(scene: roboplan.core._core_ext.Scene, trajectory: roboplan.core._core_ext.JointTrajectory, frame_names: Sequence[str], frame_id: str = 'world', ns: str = '/roboplan_path', color: object | None = None, line_width: float = 0.01) -> object:
    """
    Build a LINE_LIST Marker tracing the Cartesian path of the given frames along a joint trajectory via forward kinematics.
    """
