#!/usr/bin/env python3

"""
Example node demonstrating a basic plan, preview, and move workflow:
  1. Set a target pose with an interactive marker (IK)
  2. Plan a trajectory to that pose
  3. Preview the trajectory with visualization
  4. Execute (publish to "hardware")

Intended as an example _only_. Consumers are expected to use this as a
reference, rather than hardened application code.
"""

import os
import time
import threading
import xacro

from ament_index_python.packages import get_package_share_path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from interactive_markers import InteractiveMarkerServer
from std_srvs.srv import Trigger
from interactive_markers import MenuHandler

from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    PathShortcuttingOptions,
    PathShortcutter,
    Scene,
)
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.rrt import RRT, RRTOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan_ros.visualization import (
    RoboplanVisualizer,
    RoboplanIKMarker,
    markerFromJointTrajectory,
)
from roboplan_ros.cpp import (
    buildConversionMap,
    fromJointState,
    toJointTrajectory,
    se3ToPose,
)
from roboplan_ros_py.trajectory_publisher import TrajectoryPublisher
from roboplan_ros_examples import run_node, spin_executor
from roboplan_ros_examples.utils import (
    JointStateSubscriber,
    ParallelGripperClient,
    add_box_obstacles,
    obstacle_marker_array,
)


class PlanAndExecuteNode(Node):
    """
    Example node with a set-pose, plan, preview, execute workflow.

    Uses an interactive marker for IK-based target selection, RRT for
    planning, TOPP-RA for time parameterization, and wraps a joint
    trajectory controller's action interface for execution.

    No monitoring provided, this is example code only!
    """

    def __init__(self):
        super().__init__("plan_and_execute_node")

        # Scene file params, defaults to FR3 packages
        self.declare_parameter("robot_description_package", "roboplan_example_models")
        self.declare_parameter(
            "robot_descriptions_model_path", "models/franka_robot_model"
        )
        self.declare_parameter("urdf_filename", "fr3.urdf")
        self.declare_parameter("srdf_filename", "fr3.srdf")
        self.declare_parameter("yaml_config_filename", "fr3_config.yaml")

        # HW connection params
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "action_server_name", "/fr3_arm_controller/follow_joint_trajectory"
        )

        # Gripper params, for commanding a parallel gripper controller
        self.declare_parameter(
            "gripper_action_name", "/fr3_gripper_controller/gripper_cmd"
        )
        self.declare_parameter("gripper_joint", "fr3_finger_joint1")
        self.declare_parameter("gripper_open_position", 0.04)
        self.declare_parameter("gripper_closed_position", 0.0)

        # IK and Planner params
        self.declare_parameter("include_shortcutting", True)
        self.declare_parameter("joint_group", "fr3_arm")
        self.declare_parameter("base_link", "fr3_link0")
        self.declare_parameter("tip_link", "fr3_hand")

        # Optional YAML file with box obstacles to add to the planning scene
        self.declare_parameter("obstacles_config_file", "")

        # Get parameter values
        robot_description_package = self.get_parameter(
            "robot_description_package"
        ).value
        robot_descriptions_model_path = self.get_parameter(
            "robot_descriptions_model_path"
        ).value
        urdf_filename = self.get_parameter("urdf_filename").value
        srdf_filename = self.get_parameter("srdf_filename").value
        yaml_config_filename = self.get_parameter("yaml_config_filename").value

        joint_state_topic = self.get_parameter("joint_state_topic").value
        action_server_name = self.get_parameter("action_server_name").value

        gripper_action_name = self.get_parameter("gripper_action_name").value
        self._gripper_joint = self.get_parameter("gripper_joint").value
        self._gripper_open_position = self.get_parameter("gripper_open_position").value
        self._gripper_closed_position = self.get_parameter(
            "gripper_closed_position"
        ).value

        self._include_shortcutting = self.get_parameter("include_shortcutting").value
        self._joint_group = self.get_parameter("joint_group").value
        self._base_link = self.get_parameter("base_link").value
        self._tip_link = self.get_parameter("tip_link").value

        # Get robot description files and setup the scene
        pkg_share_dir = get_package_share_path(robot_description_package).as_posix()
        models_dir = os.path.join(
            pkg_share_dir,
            robot_descriptions_model_path,
        )
        urdf_xml = xacro.process_file(os.path.join(models_dir, urdf_filename)).toxml()
        srdf_xml = xacro.process_file(os.path.join(models_dir, srdf_filename)).toxml()
        yaml_config_path = os.path.join(models_dir, yaml_config_filename)
        package_paths = [pkg_share_dir]
        self._scene = Scene(
            name="plan_execute_scene",
            urdf=urdf_xml,
            srdf=srdf_xml,
            package_paths=package_paths,
            yaml_config_path=yaml_config_path,
        )
        self._q_indices = self._scene.getJointGroupInfo(self._joint_group).q_indices

        # Optionally add obstacles (e.g., a tabletop) to the planning scene so
        # that IK and planning avoid them.
        self._obstacles = []
        obstacles_config_file = self.get_parameter("obstacles_config_file").value
        if obstacles_config_file:
            self._obstacles = add_box_obstacles(self._scene, obstacles_config_file)
            self.get_logger().info(
                f"Added {len(self._obstacles)} obstacle(s) to the planning scene."
            )

        # Subscribe to joint states to keep the scene in sync with hardware. These
        # can bog down other CBs, so putting it out here keeps the rest of the node
        # responsive.
        self._js_subscriber = JointStateSubscriber(topic=joint_state_topic)

        # Wait for joint states
        while self._js_subscriber.last_joint_state is None:
            self.get_logger().info("Waiting for joint positions...")
            time.sleep(1.0)

        # Once we have joint states we can build the conversion map
        self._conversion_map = buildConversionMap(
            self._scene, self._js_subscriber.last_joint_state
        )

        # Set up the IK solver and wrap it in a callback for the marker
        ik_options = SimpleIkOptions()
        ik_options.group_name = self._joint_group
        ik_options.max_iters = 100
        ik_options.step_size = 0.25
        ik_options.max_time = 0.05
        ik_options.check_collisions = True
        # Increases likelihood of finding an "optimal" solution
        ik_options.fast_return = False

        # Setup an IK solver and function to pass to the interactive marker
        ik_solver = SimpleIk(self._scene, ik_options)
        q_indices = self._q_indices
        joint_group = self._joint_group
        base_link = self._base_link
        tip_link = self._tip_link
        scene = self._scene

        def ik_solve_fn(target_pose, seed):
            goal = CartesianConfiguration()
            goal.base_frame = base_link
            goal.tip_frame = tip_link
            goal.tform = target_pose
            seed_jc = JointConfiguration()
            seed_jc.positions = seed[q_indices]
            solution = JointConfiguration()
            if ik_solver.solveIk(goal, seed_jc, solution):
                return scene.toFullJointPositions(joint_group, solution.positions)
            return None

        self._ik_marker = RoboplanIKMarker(
            scene=self._scene,
            base_link=self._base_link,
            tip_link=self._tip_link,
            ik_solve_fn=ik_solve_fn,
        )

        # Set up planning utilities
        self._rrt_options = RRTOptions()
        self._rrt_options.group_name = self._joint_group
        self._rrt_options.max_connection_distance = 3.0
        self._rrt_options.collision_check_step_size = 0.05
        self._rrt_options.max_planning_time = 5.0
        self._rrt_options.rrt_connect = True
        self._rrt_options.max_nodes = 1000
        self._rrt_options.goal_biasing_probability = 0.05
        self._rrt_options.collision_check_use_bisection = True
        self._include_shortcutting = True
        self._max_shortcutting_iters = 250

        self._shortcutting_options = PathShortcuttingOptions(
            group_name=self._joint_group,
            max_step_size=self._rrt_options.collision_check_step_size,
            max_iters=self._max_shortcutting_iters,
        )

        self._rrt = RRT(self._scene, self._rrt_options)
        self._toppra = PathParameterizerTOPPRA(self._scene, self._joint_group)
        self._shortcutter = PathShortcutter(self._scene, self._shortcutting_options)
        self._traj_dt = 0.01

        # Default QoS for visualization
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Configure elements for determining and previewing poses from the
        # iMarker
        self._marker_node = Node("imarker_server_node")
        self._ik_server = InteractiveMarkerServer(self._marker_node, "roboplan_ik")
        self._ik_server.insert(
            self._ik_marker.construct_imarker(),
            feedback_callback=self._on_ik_feedback,
        )
        self._ik_server.applyChanges()

        # Needs its own executor for responsiveness
        self._marker_executor = SingleThreadedExecutor()
        self._marker_executor.add_node(self._marker_node)
        self._marker_thread = threading.Thread(
            target=spin_executor, daemon=True, args=(self._marker_executor,)
        )
        self._marker_thread.start()

        # Add menu to the iMarker for service access
        menu = MenuHandler()
        menu.insert("Plan", callback=self._on_plan_menu)
        menu.insert("Preview", callback=self._on_preview_menu)
        menu.insert("Execute", callback=self._on_execute_menu)
        menu.insert("Reset", callback=self._on_reset_menu)
        menu.insert("Open Gripper", callback=self._on_open_gripper_menu)
        menu.insert("Close Gripper", callback=self._on_close_gripper_menu)
        menu.apply(self._ik_server, "ik_target")
        self._ik_server.applyChanges()

        # IK determined target pose in blue
        self._ik_visualizer = RoboplanVisualizer(
            scene=self._scene,
            group_name=self._joint_group,
            urdf_xml=urdf_xml,
            frame_id="world",
            ns="roboplan_ik",
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5),
        )
        self._ik_marker_pub = self.create_publisher(
            MarkerArray, "roboplan_ik/markers", qos
        )

        # Configure tools for previewing trajectories, the markers will be
        # published in green.
        self._traj_visualizer = RoboplanVisualizer(
            scene=self._scene,
            group_name=self._joint_group,
            urdf_xml=urdf_xml,
            frame_id="world",
            ns="roboplan_traj",
            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.3),
        )
        self._traj_marker_pub = self.create_publisher(
            MarkerArray, "roboplan_trajectory/markers", qos
        )
        self._player = TrajectoryPublisher(
            self._scene,
            self._traj_visualizer,
            self._traj_marker_pub,
            self._q_indices,
        )

        # Publish the planned end-effector path as a light green line upon planning.
        # Unlike the IK/preview markers (which are republished continuously), the path
        # is published exactly once per plan, so use a latched QoS.
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._planned_path_color = ColorRGBA(r=0.5, g=1.0, b=0.5, a=1.0)
        self._planned_path_pub = self.create_publisher(
            Marker, "/roboplan_trajectory/path", latched_qos
        )

        # Publish any planning scene obstacles as markers, exactly once.
        self._obstacle_marker_pub = self.create_publisher(
            MarkerArray, "roboplan_scene/obstacles", latched_qos
        )
        if self._obstacles:
            self._obstacle_marker_pub.publish(obstacle_marker_array(self._obstacles))

        # Setup an action client for trajectory execution
        self._execute_client = ActionClient(
            self, FollowJointTrajectory, action_server_name
        )

        # Setup an action client for commanding a parallel gripper controller
        self._gripper_client = ParallelGripperClient(
            self,
            gripper_action_name,
            self._gripper_joint,
            self._gripper_open_position,
            self._gripper_closed_position,
        )

        # Target pose and planned trajectories
        self._target_q = None
        self._planned_traj = None

        # Setup Trigger Services
        self.create_service(Trigger, "~/plan", self._on_plan)
        self.create_service(Trigger, "~/preview", self._on_preview)
        self.create_service(Trigger, "~/execute", self._on_execute)
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_service(Trigger, "~/open_gripper", self._on_open_gripper)
        self.create_service(Trigger, "~/close_gripper", self._on_close_gripper)

        # Reset and notify
        self._reset()
        self.get_logger().info("Ready. Move the interactive marker to set a target.")
        self.get_logger().info(
            "Call services: ~/plan, ~/preview, ~/execute, ~/reset, "
            "~/open_gripper, ~/close_gripper"
        )

    def _get_hardware_positions(self):
        """
        Returns the latest joint positions, clamped to the joint limits.

        Simulated (or real) hardware can report positions slightly outside the
        limits, which would otherwise make planning reject the configuration.
        """
        joint_config = fromJointState(
            self._js_subscriber.last_joint_state, self._scene, self._conversion_map
        )
        return self._scene.clampToValidConfiguration(joint_config.positions)

    def _on_ik_feedback(self, feedback):
        # Sync with the latest hardware state so that joints outside the
        # planning group (e.g., the gripper fingers) are up to date in the
        # scene, since the IK solution is visualized at the full configuration.
        self._latest_joint_positions = self._get_hardware_positions()
        self._scene.setJointPositions(self._latest_joint_positions)
        self._ik_marker.set_seed_configuration(self._latest_joint_positions)
        q = self._ik_marker.process_feedback(feedback)
        if q is not None:
            self._target_q = q
            self._ik_marker_pub.publish(
                self._ik_visualizer.markers_from_configuration(q)
            )

    def _plan(self):
        if self._target_q is None:
            return False, "No target set. Move the interactive marker first."

        self._latest_joint_positions = self._get_hardware_positions()
        self._scene.setJointPositions(self._latest_joint_positions)

        start = JointConfiguration()
        start.positions = self._latest_joint_positions[self._q_indices]

        goal = JointConfiguration()
        goal.positions = self._target_q[self._q_indices]

        self.get_logger().info("Planning...")
        plan_start_time = time.time()

        try:
            start_time = time.time()
            path = self._rrt.plan(start, goal)
            self.get_logger().info(
                f"  Finished planning in {time.time() - start_time} seconds."
            )
        except RuntimeError as e:
            self.get_logger().error(str(e))
            path = None

        if path is None:
            return False, "Planning failed."

        if self._include_shortcutting:
            self.get_logger().info("Shortcutting...")
            start_time = time.time()
            path = self._shortcutter.shortcut(path)
            self.get_logger().info(
                f"  Finished shortcutting in {time.time() - start_time} seconds."
            )

        self.get_logger().info("Generating trajectory...")
        start_time = time.time()
        self._planned_traj = self._toppra.generate(
            path,
            TOPPRAOptions(
                self._traj_dt,
                mode=SplineFittingMode.Adaptive,
                max_adaptive_iterations=5,
            ),
        )
        self.get_logger().info(
            f"  Finished generating trajectory in {time.time() - start_time} seconds."
        )

        self.get_logger().info(
            f"Total planning time: {time.time() - plan_start_time} seconds."
        )

        # Visualize the planned end-effector trajectory.
        self._planned_path_pub.publish(
            markerFromJointTrajectory(
                self._scene,
                self._planned_traj,
                [self._tip_link],
                frame_id="world",
                ns="planned_trajectory",
                color=self._planned_path_color,
            )
        )

        return (
            True,
            f"Planned trajectory with {len(self._planned_traj.positions)} points",
        )

    def _preview(self):
        if self._planned_traj is None:
            return False, "No trajectory to preview. Plan first."

        self.get_logger().info("Previewing trajectory...")
        self._player.play(
            self._planned_traj,
            self._traj_dt,
            on_complete=lambda: self.get_logger().info("Preview complete."),
        )
        return True, "Playback started."

    def _execute(self):
        if self._planned_traj is None:
            return False, "No trajectory to execute. Plan first."

        if not self._execute_client.wait_for_server(timeout_sec=2.0):
            return False, "Action server not available."

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = toJointTrajectory(self._planned_traj)

        self.get_logger().info("Sending trajectory for execution...")
        future = self._execute_client.send_goal_async(
            goal, feedback_callback=self._execute_feedback
        )
        future.add_done_callback(self._execute_goal_response)

        return True, "Trajectory sent for execution."

    def _execute_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Trajectory execution rejected.")
            return

        self.get_logger().info("Trajectory accepted, executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._execute_result)

    def _execute_feedback(self, feedback_msg):
        pass

    def _execute_result(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Trajectory execution complete.")
        else:
            self.get_logger().error(
                f"Trajectory execution failed with error code: {result.error_code}"
            )

    def _reset(self):
        """Clears all plans and resets to a hardware state."""
        if self._js_subscriber.last_joint_state is None:
            raise RuntimeError("No joint states received, cannot reset to hw state.")

        # Reset joint positions to the latest joint state
        self._latest_joint_positions = self._get_hardware_positions()

        # Update the IK marker's seed to the current state
        self._ik_marker.set_seed_configuration(self._latest_joint_positions)

        # Compute FK for the current state to get the marker pose
        fk = self._scene.forwardKinematics(
            self._latest_joint_positions, self._tip_link, self._base_link
        )
        pose = se3ToPose(fk)

        # Update the IK to the current pose
        self._ik_server.setPose("ik_target", pose)
        self._ik_server.applyChanges()
        self._ik_marker_pub.publish(
            self._ik_visualizer.markers_from_configuration(self._latest_joint_positions)
        )

        # Clear the planned trajectory and target
        self._target_q = None
        self._planned_traj = None
        self._traj_marker_pub.publish(self._traj_visualizer.clear_markers())
        delete_marker = Marker()
        delete_marker.header.frame_id = "world"
        delete_marker.action = Marker.DELETEALL
        self._planned_path_pub.publish(delete_marker)

    # Menu callbacks
    def _on_plan_menu(self, feedback):
        _, msg = self._plan()
        self.get_logger().info(msg)

    def _on_preview_menu(self, feedback):
        _, msg = self._preview()
        self.get_logger().info(msg)

    def _on_execute_menu(self, feedback):
        _, msg = self._execute()
        self.get_logger().info(msg)

    def _on_reset_menu(self, feedback):
        self._reset()
        self.get_logger().info("Reset node to current state.")

    def _on_open_gripper_menu(self, feedback):
        _, msg = self._gripper_client.open()
        self.get_logger().info(msg)

    def _on_close_gripper_menu(self, feedback):
        _, msg = self._gripper_client.close()
        self.get_logger().info(msg)

    # Trigger service callbacks
    def _on_plan(self, request, response):
        response.success, response.message = self._plan()
        return response

    def _on_preview(self, request, response):
        response.success, response.message = self._preview()
        return response

    def _on_execute(self, request, response):
        response.success, response.message = self._execute()
        return response

    def _on_reset(self, request, response):
        self._reset()
        response.success = True
        response.message = "Reset node to current state."
        self.get_logger().info(response.message)
        return response

    def _on_open_gripper(self, request, response):
        response.success, response.message = self._gripper_client.open()
        return response

    def _on_close_gripper(self, request, response):
        response.success, response.message = self._gripper_client.close()
        return response

    def destroy_node(self):
        self._player.stop()
        self._js_subscriber.shutdown()
        self._marker_executor.shutdown()
        self._marker_thread.join(timeout=0.25)
        self._marker_node.destroy_node()

        # Manually remove self referenced nanobind objects before destruction.
        # https://nanobind.readthedocs.io/en/latest/refleaks.html
        self._ik_marker = None

        super().destroy_node()


if __name__ == "__main__":
    rclpy.init()
    run_node(PlanAndExecuteNode())
