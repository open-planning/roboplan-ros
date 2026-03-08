#!/usr/bin/env python3

from roboplan.core import Scene
from roboplan_ros_visualization.bindings import RoboplanVisualizer


BOX_URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <link name="box_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
  <joint name="fixed_joint" type="fixed">
    <parent link="world"/>
    <child link="box_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
</robot>
"""


EMPTY_SRDF = """<?xml version="1.0"?>
<robot name="test_robot">
</robot>
"""


def test_import():
    from roboplan_ros_visualization import bindings

    assert hasattr(bindings, "RoboplanVisualizer")


def test_visualize_configuration():
    print("Constructing scene")
    scene = Scene(name="test", urdf=BOX_URDF, srdf=EMPTY_SRDF)
    viz = RoboplanVisualizer(scene=scene, urdf_xml=BOX_URDF)

    q = scene.getCurrentJointPositions()
    print("Visualizing config scene")
    marker_array = viz.visualize_configuration(q)
    assert (
        len(marker_array.markers) == 1
    ), f"Expected 1 marker, got {len(marker_array.markers)}"

    # marker_array = viz.clear_markers()
    # assert len(marker_array.markers) == 1, f"Expected 1 marker, got {len(marker_array.markers)}"
