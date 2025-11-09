#!/usr/bin/env python3

import os
import xacro
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger


from roboplan.core import Scene, JointConfiguration, PathShortcutter
from roboplan.simple_ik import SimpleIkOptions
from roboplan.rrt import RRTOptions, RRT
from roboplan.toppra import PathParameterizerTOPPRA

from roboplan_ros_py.type_conversions import from_joint_state, to_joint_trajectory
from roboplan_ros_visualization.interactive_marker_ik import InteractiveMarkerIK
from roboplan_ros_visualization.roboplan_visualizer import RoboplanVisualizer


class InteractiveMarkerIKNode(Node):
    """
    ROS 2 node that provides an interactive marker for IK-based pose control.
    Publishes joint states when IK solutions are found.
    """

    def __init__(self):
        super().__init__("interactive_marker_ik_node")

        # Declare parameters with Franka FR3 defaults
        self.declare_parameter("joint_group", "fr3_arm")
        self.declare_parameter("base_link", "fr3_link0")
        self.declare_parameter("tip_link", "fr3_hand")
        self.declare_parameter("robot_description_package", "roboplan_example_models")
        self.declare_parameter("urdf_filename", "fr3.urdf")
        self.declare_parameter("srdf_filename", "fr3.srdf")
        self.declare_parameter("yaml_config_filename", "fr3_config.yaml")

        # Get parameter values
        self.joint_group = self.get_parameter("joint_group").value
        self.base_link = self.get_parameter("base_link").value
        self.tip_link = self.get_parameter("tip_link").value
        robot_description_package = self.get_parameter(
            "robot_description_package"
        ).value
        urdf_filename = self.get_parameter("urdf_filename").value
        srdf_filename = self.get_parameter("srdf_filename").value
        yaml_config_filename = self.get_parameter("yaml_config_filename").value

        # Get robot description files
        models_dir = os.path.join(
            get_package_share_directory(robot_description_package),
            "models",
            "franka_robot_model",
        )
        self.get_logger().info(f"Loading configuration from {models_dir}...")

        urdf_path = os.path.join(models_dir, urdf_filename)
        srdf_path = os.path.join(models_dir, srdf_filename)
        yaml_config_path = os.path.join(models_dir, yaml_config_filename)

        # Process files with xacro
        self.get_logger().info("Processing URDF and SRDF with xacro...")
        urdf_xml = xacro.process_file(urdf_path).toxml()
        srdf_xml = xacro.process_file(srdf_path).toxml()

        # Create the RoboPlan scene
        self.get_logger().info("Creating RoboPlan scene...")
        package_paths = [get_package_share_directory(robot_description_package)]

        self.scene = Scene(
            name="franka_ik_scene",
            urdf=urdf_xml,
            srdf=srdf_xml,
            package_paths=package_paths,
            yaml_config_path=yaml_config_path,
        )

        # Start in a reasonable pose, this could come from hardware.
        self.latest_joint_positions = np.array(
            [
                0.0,
                -0.785,
                0.0,
                -2.356,
                0.0,
                1.571,
                0.785,
                0,
                0,
            ]
        )
        self.scene.setJointPositions(self.latest_joint_positions)

        # Set the IK solver options
        options = SimpleIkOptions()
        options.group_name = self.joint_group
        options.max_iters = 100
        options.step_size = 0.25

        # Setup a visualize marker model for rendering IK solutions. The markers will
        # be published to "/roboplan_ik/markers" and are renderable in rviz.
        self.ik_viz = RoboplanVisualizer(
            node=self,
            scene=self.scene,
            urdf_xml=urdf_xml,
            package_paths=package_paths,
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
            frame_id="world",
            namespace="roboplan_ik",
        )

        # Create the interactive marker controller, which will call the ik_viz to
        # update anytime a solution is found.
        self.get_logger().info("Creating interactive marker controller...")
        self.imarker_ik = InteractiveMarkerIK(
            node=self,
            scene=self.scene,
            joint_group=self.joint_group,
            base_link=self.base_link,
            tip_link=self.tip_link,
            options=options,
            solve_callback=self.ik_viz.visualize_configuration,
            namespace="roboplan_ik",
        )

        # Set up an RRT instance, path shortener, and path parameterizer for path planning
        # to IK solutions.
        options = RRTOptions()
        options.group_name = self.joint_group
        options.max_connection_distance = 3.0
        options.collision_check_step_size = 0.01
        options.max_nodes = 10000
        options.max_planning_time = 5.0
        options.rrt_connect = True

        self.rrt = RRT(self.scene, options)
        self.path_shortcutter = PathShortcutter(self.scene, self.joint_group)
        self.path_toppra = PathParameterizerTOPPRA(self.scene, self.joint_group)

        # Set up a trigger service to initiate path planning
        self.planning_service = self.create_service(
            Trigger, "~/plan_to_pose", self._plan_path_cb
        )

        # Subscribe to throttled joint states topic
        self.latest_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states_throttled", self._joint_state_cb, 1
        )

        self.get_logger().info(
            "Move the interactive marker in RViz to visualize IK solutions"
        )

    def destroy_node(self):
        """
        Override the default to ensure a clean shutdown.
        """
        self.imarker_ik.shutdown()
        super().destroy_node()

    def _joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def _plan_path_cb(self, _, response):
        """
        Attempts to plan a path from robot's current pose to the ik marker's latest solution.
        """

        # Setup the start and goal pose from the latest joint states and ik solution, respectively
        self.get_logger().info("Planning requested, getting latest joint state...")
        if not self.latest_joint_state:
            response.success = False
            response.message = "No start joint states available"
            return response
        latest_joint_state = from_joint_state(self.latest_joint_state)

        self.get_logger().info(f"Before: {latest_joint_state.positions}")
        q = np.array(latest_joint_state.positions, dtype=np.float64)
        self.scene.applyMimics(q)
        self.get_logger().info(f"After: {q}")
        # self.get_logger().info(f"After: {latest_joint_state}")

        # Mimic joints need to be included
        start = JointConfiguration()
        start.positions = q

        goal = JointConfiguration()
        goal.positions = self.imarker_ik.last_joint_positions

        # Plan a path
        self.get_logger().info("Planning requested between poses...")
        path = self.rrt.plan(start, goal)
        if not path:
            response.success = False
            response.message = "Failed to find a path between poses"
            return response
        self.get_logger().info(f"Path found:\n{path}")

        # Apply short cutting
        self.get_logger().info("Shortcutting path...")
        shortened_path = self.path_shortcutter.shortcut(path, 0.01, max_iters=1000)
        self.get_logger().info(f"Shortcutted path:\n{shortened_path}")

        # Path parameterize it
        self.get_logger().info("Generating trajectory...")
        trajectory = self.path_toppra.generate(shortened_path)
        self.latest_trajectory = to_joint_trajectory(trajectory)
        self.get_logger().info("Trajectory generated!")

        # We did it
        response.success = True
        response.message = (
            f"Successfully planned path with {len(shortened_path.positions)} points."
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerIKNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
