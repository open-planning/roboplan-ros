from typing import Annotated

import numpy
from numpy.typing import NDArray
import roboplan.roboplan_ext.core
import roboplan.roboplan_ext.simple_ik


class RoboplanVisualizer:
    """
    Tool to build RViz MarkerArray messages from a RoboPlan scene and joint configuration.
    """

    def __init__(self, scene: roboplan.roboplan_ext.core.Scene, urdf_xml: str, frame_id: str = 'world', ns: str = '/roboplan', color: object | None = None) -> None: ...

    def markers_from_configuration(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> object:
        """Compute marker array for the given joint configuration."""

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

    def __init__(self, scene: roboplan.roboplan_ext.core.Scene, joint_group: str, base_link: str, tip_link: str, options: roboplan.roboplan_ext.simple_ik.SimpleIkOptions = ...) -> None: ...

    def construct_imarker(self) -> object:
        """Build an InteractiveMarker message for the current target pose."""

    def process_feedback(self, feedback: object) -> object:
        """
        Process InteractiveMarkerFeedback. Returns joint positions on success, or else None.
        """

    @property
    def last_joint_positions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The last successful joint positions (or initial seed)."""

    def set_joint_positions(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the seed joint positions for the next solve."""
