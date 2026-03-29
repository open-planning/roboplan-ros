#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import InteractiveMarkerFeedback

from roboplan.core import Scene
from roboplan_ros_cpp.bindings import se3ToPose
from roboplan_ros_visualization.bindings import RoboplanIKMarker
from roboplan.simple_ik import SimpleIkOptions


TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="world"/>
  <link name="link1">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>
"""

TWO_LINK_SRDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <group name="arm">
    <chain base_link="world" tip_link="link2"/>
  </group>
</robot>
"""


def test_process_feedback():
    scene = Scene(name="test", urdf=TWO_LINK_URDF, srdf=TWO_LINK_SRDF)
    options = SimpleIkOptions()
    options.max_iters = 1000
    ik = RoboplanIKMarker(
        scene=scene,
        joint_group="arm",
        base_link="world",
        tip_link="link2",
        options=options,
    )

    # Compute a reachable target via FK at a known configuration
    q_target = scene.getCurrentJointPositions()
    q_target[0] = 0.5
    q_target[1] = 0.5
    fk_target = scene.forwardKinematics(q_target, "link2")
    target_pose = se3ToPose(fk_target)

    # Solve via process_feedback
    feedback = InteractiveMarkerFeedback()
    feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    feedback.pose = target_pose
    result = ik.process_feedback(feedback)

    # Verify FK of solution matches the target
    assert result is not None
    fk_result = scene.forwardKinematics(result, "link2")
    assert abs(fk_result[0, 3] - target_pose.position.x) < 0.01
    assert abs(fk_result[1, 3] - target_pose.position.y) < 0.01
    assert abs(fk_result[2, 3] - target_pose.position.z) < 0.01
