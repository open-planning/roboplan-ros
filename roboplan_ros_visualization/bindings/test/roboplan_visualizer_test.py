#!/usr/bin/env python3

from roboplan.core import Scene
from roboplan_ros.visualization import RoboplanVisualizer


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


TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="two_link_robot">
  <link name="world"/>
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="link2">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
"""


TWO_LINK_SRDF = """<?xml version="1.0"?>
<robot name="two_link_robot">
  <group name="first">
    <joint name="joint1"/>
  </group>
</robot>
"""


def test_import():
    from roboplan_ros import visualization

    assert hasattr(visualization, "RoboplanVisualizer")


def test_visualize_configuration():
    scene = Scene(name="test", urdf=BOX_URDF, srdf=EMPTY_SRDF)
    viz = RoboplanVisualizer(scene=scene, urdf_xml=BOX_URDF)

    q = scene.getCurrentJointPositions()
    marker_array = viz.markers_from_configuration(q)
    assert (
        len(marker_array.markers) == 1
    ), f"Expected 1 marker, got {len(marker_array.markers)}"

    marker_array = viz.clear_markers()
    assert (
        len(marker_array.markers) == 1
    ), f"Expected 1 marker, got {len(marker_array.markers)}"


def test_visualize_joint_group():
    scene = Scene(name="test", urdf=TWO_LINK_URDF, srdf=TWO_LINK_SRDF)
    viz = RoboplanVisualizer(scene=scene, urdf_xml=TWO_LINK_URDF)

    q = scene.getCurrentJointPositions()

    # The whole scene has two visual geometries.
    all_markers = viz.markers_from_configuration(q)
    assert (
        len(all_markers.markers) == 2
    ), f"Expected 2 markers, got {len(all_markers.markers)}"

    # The "first" group only drives link1, so a single marker should be returned.
    viz.set_group("first")
    group_markers = viz.markers_from_configuration(q)
    assert (
        len(group_markers.markers) == 1
    ), f"Expected 1 marker, got {len(group_markers.markers)}"

    # Switching back to the whole scene renders both geometries again.
    viz.set_group("")
    all_markers = viz.markers_from_configuration(q)
    assert (
        len(all_markers.markers) == 2
    ), f"Expected 2 markers, got {len(all_markers.markers)}"


def test_constructor_group_name():
    scene = Scene(name="test", urdf=TWO_LINK_URDF, srdf=TWO_LINK_SRDF)
    # Configure the group at construction time.
    viz = RoboplanVisualizer(scene=scene, urdf_xml=TWO_LINK_URDF, group_name="first")

    q = scene.getCurrentJointPositions()

    # The constructor's group ("first") is used.
    markers = viz.markers_from_configuration(q)
    assert len(markers.markers) == 1, f"Expected 1 marker, got {len(markers.markers)}"
