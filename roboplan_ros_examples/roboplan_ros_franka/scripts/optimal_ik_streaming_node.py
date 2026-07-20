#!/usr/bin/env python3

"""
Example node demonstrating Cartesian servoing with OInK (Optimal Inverse Kinematics):
  1. Drag the interactive marker to set a target pose
  2. A background control loop continuously runs OInK one step per tick
  3. The result is published directly as a joint command

No planning, preview, or trajectory generation — just direct IK tracking.
Use the iMarker dropdown menu to start / pause / reset tracking.

Intended as an example _only_. Consumers are expected to use this as a
reference, rather than hardened application code.
"""

import os
import time
import threading
import xacro
import numpy as np

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import MarkerArray
from interactive_markers import InteractiveMarkerServer, MenuHandler
from std_srvs.srv import Trigger

from roboplan.core import CartesianConfiguration, Scene
from roboplan.filters import SE3LowPassFilter
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionLimit,
    SelfCollisionBarrier,
    SelfCollisionBarrierOptions,
    VelocityLimit,
)
from roboplan_ros.visualization import RoboplanIKMarker
from roboplan_ros.cpp import buildConversionMap, fromJointState, se3ToPose
from roboplan_ros_examples import run_node, spin_executor
from roboplan_ros_examples.utils import (
    LATCHED_QOS,
    JointStateSubscriber,
    ParallelGripperClient,
    add_box_obstacles,
    obstacle_marker_array,
)


class CartesianServoNode(Node):
    """
    Cartesian servoing node using OInK.

    Drag the interactive marker to set a target end-effector pose.
    A background control loop continuously runs OInK one step per tick,
    integrates the result, and publishes the joint command directly.
    """

    def __init__(self):
        super().__init__("cartesian_servo_node")

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
            "joint_command_topic", "/fr3_arm_controller/joint_trajectory"
        )

        # Gripper params, for commanding a parallel gripper controller
        self.declare_parameter(
            "gripper_action_name", "/fr3_gripper_controller/gripper_cmd"
        )
        self.declare_parameter("gripper_joint", "fr3_finger_joint1")
        self.declare_parameter("gripper_open_position", 0.04)
        self.declare_parameter("gripper_closed_position", 0.0)

        # Optional YAML file with box obstacles to add to the scene
        self.declare_parameter("obstacles_config_file", "")

        # Whether to add a collision avoidance barrier to the solver.
        self.declare_parameter("use_collision_barrier", False)

        # IK params
        self.declare_parameter("joint_group", "fr3_arm")
        self.declare_parameter("base_link", "fr3_link0")
        self.declare_parameter("tip_link", "fr3_hand")
        self.declare_parameter("task_gain", 1.0)
        self.declare_parameter("lm_damping", 0.01)
        self.declare_parameter("regularization", 1e-6)
        self.declare_parameter("position_cost", 1.0)
        self.declare_parameter("orientation_cost", 0.1)
        self.declare_parameter("control_freq", 100.0)
        self.declare_parameter("reference_filter_tau", 0.1)
        self.declare_parameter("command_duration_ms", 100)

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
        joint_command_topic = self.get_parameter("joint_command_topic").value

        gripper_action_name = self.get_parameter("gripper_action_name").value
        gripper_joint = self.get_parameter("gripper_joint").value
        gripper_open_position = self.get_parameter("gripper_open_position").value
        gripper_closed_position = self.get_parameter("gripper_closed_position").value

        use_collision_barrier = self.get_parameter("use_collision_barrier").value

        self._joint_group = self.get_parameter("joint_group").value
        self._base_link = self.get_parameter("base_link").value
        self._tip_link = self.get_parameter("tip_link").value
        task_gain = self.get_parameter("task_gain").value
        lm_damping = self.get_parameter("lm_damping").value
        self._regularization = self.get_parameter("regularization").value
        position_cost = self.get_parameter("position_cost").value
        orientation_cost = self.get_parameter("orientation_cost").value
        control_freq = self.get_parameter("control_freq").value
        self._reference_filter_tau = self.get_parameter("reference_filter_tau").value
        command_duration_ms = self.get_parameter("command_duration_ms").value
        self._command_duration_msg = Duration(
            seconds=command_duration_ms / 1000.0
        ).to_msg()

        # Control loop time step for Cartesian tracking
        self._dt = 1.0 / control_freq
        self._period = Duration(seconds=self._dt)

        # Get robot description files and setup the scene
        pkg_share_dir = get_package_share_directory(robot_description_package)
        models_dir = os.path.join(pkg_share_dir, robot_descriptions_model_path)
        urdf_xml = xacro.process_file(os.path.join(models_dir, urdf_filename)).toxml()
        srdf_xml = xacro.process_file(os.path.join(models_dir, srdf_filename)).toxml()
        yaml_config_path = os.path.join(models_dir, yaml_config_filename)
        package_paths = [pkg_share_dir]
        self._scene = Scene(
            name="cartesian_servo_scene",
            urdf=urdf_xml,
            srdf=srdf_xml,
            package_paths=package_paths,
            yaml_config_path=yaml_config_path,
        )

        # Optionally add obstacles (e.g., a tabletop) to the scene.
        self._obstacles = []
        obstacles_config_file = self.get_parameter("obstacles_config_file").value
        if obstacles_config_file:
            self._obstacles = add_box_obstacles(self._scene, obstacles_config_file)
            self.get_logger().info(
                f"Added {len(self._obstacles)} obstacle(s) to the scene."
            )

        # Start paused so the robot doesn't move until the user is ready and the
        # iMarker has been initialized. Otherwise danger.
        self._paused = True

        # Subscribe to joint states to keep the scene in sync with hardware.
        self._js_subscriber = JointStateSubscriber(topic=joint_state_topic)

        # Wait for joint states
        while self._js_subscriber.last_joint_state is None:
            self.get_logger().info("Waiting for joint positions...")
            time.sleep(1.0)

        # Once we have joint states, build the required joint state to roboplan scene
        # conversion map
        joint_group_info = self._scene.getJointGroupInfo(self._joint_group)
        self._q_indices = joint_group_info.q_indices
        self._joint_names = joint_group_info.joint_names
        self._conversion_map = buildConversionMap(
            self._scene, self._js_subscriber.last_joint_state
        )

        # Set up the solver
        self._oink = Oink(self._scene, self._joint_group)
        self._num_variables = len(self._oink.v_indices)

        # Frame task for end-effector tracking (priority 1)
        goal = CartesianConfiguration()
        goal.base_frame = self._base_link
        goal.tip_frame = self._tip_link

        task_options = FrameTaskOptions(
            position_cost=position_cost,
            orientation_cost=orientation_cost,
            task_gain=task_gain,
            lm_damping=lm_damping,
        )
        self._frame_task = FrameTask(self._oink, self._scene, goal, task_options)

        # Configuration task to regularize toward starting pose (priority 2).
        # Projected into the nullspace of the frame task so it never sacrifices
        # end-effector tracking — only uses redundant degrees of freedom.
        q_home = np.array(self._scene.getCurrentJointPositions())
        joint_weights = np.full(self._num_variables, 0.05)
        self._config_task = ConfigurationTask(
            self._oink,
            q_home[self._oink.q_indices],
            joint_weights,
            ConfigurationTaskOptions(task_gain=1.0, lm_damping=0.0, priority=2),
        )

        self._tasks = [self._frame_task, self._config_task]

        # Constraints: joint position and velocity limits
        position_limit = PositionLimit(self._oink, gain=1.0)
        v_max = np.hstack(
            [
                self._scene.getJointInfo(name).limits.max_velocity
                for name in self._joint_names
            ]
        )
        velocity_limit = VelocityLimit(self._oink, self._dt, v_max)
        self._constraints = [position_limit, velocity_limit]

        self._barriers = []
        if use_collision_barrier:
            barrier_options = SelfCollisionBarrierOptions(
                n_collision_pairs=5, d_min=0.02
            )
            self._barriers.append(
                SelfCollisionBarrier(self._oink, self._scene, self._dt, barrier_options)
            )

        # Thread-safe access to scene and target
        self._lock = threading.Lock()

        # Reference filter for smooth target tracking
        q_full = self._scene.getCurrentJointPositions()
        initial_pose = self._scene.forwardKinematics(
            q_full, self._tip_link, self._base_link
        )
        self._raw_target = initial_pose.copy()
        self._reference_filter = SE3LowPassFilter(tau=self._reference_filter_tau)
        self._reference_filter.reset(initial_pose)

        self._delta_q = np.zeros(self._num_variables)
        self._delta_q_full = np.zeros(len(q_full))

        # This is a little odd because we don't actually want to do the solving
        # while dragging the ik marker. Really we just want to know the target pose
        # so that OinK can do the work of computing joint commands in the _control
        # loop_. Still, the marker does some nice things for us so we include it here
        # and just save the target pose away.
        def store_target(target_pose, _):
            self._raw_target = target_pose.copy()
            return None

        self._ik_marker = RoboplanIKMarker(
            scene=self._scene,
            base_link=self._base_link,
            tip_link=self._tip_link,
            ik_solve_fn=store_target,
        )

        # Interactive marker server
        self._marker_node = Node("imarker_server_node")
        self._ik_server = InteractiveMarkerServer(self._marker_node, "roboplan_ik")
        self._ik_server.insert(
            self._ik_marker.construct_imarker(),
            feedback_callback=self._on_ik_feedback,
        )
        self._ik_server.applyChanges()

        # Add menu items to reset, start, and pause the marker streaming
        menu = MenuHandler()
        menu.insert("Start", callback=self._on_start_menu)
        menu.insert("Pause", callback=self._on_pause_menu)
        menu.insert("Reset", callback=self._on_reset_menu)
        menu.insert("Open Gripper", callback=self._on_open_gripper_menu)
        menu.insert("Close Gripper", callback=self._on_close_gripper_menu)
        menu.apply(self._ik_server, "ik_target")
        self._ik_server.applyChanges()

        self._marker_executor = SingleThreadedExecutor()
        self._marker_executor.add_node(self._marker_node)
        self._marker_thread = threading.Thread(
            target=spin_executor, daemon=True, args=(self._marker_executor,)
        )
        self._marker_thread.start()

        # Joint command publisher
        self._cmd_pub = self.create_publisher(JointTrajectory, joint_command_topic, 10)

        # Publish any scene obstacles as markers, exactly once
        self._obstacle_marker_pub = self.create_publisher(
            MarkerArray, "roboplan_scene/obstacles", LATCHED_QOS
        )
        if self._obstacles:
            self._obstacle_marker_pub.publish(obstacle_marker_array(self._obstacles))

        # Setup an action client for commanding a parallel gripper controller
        self._gripper_client = ParallelGripperClient(
            self,
            gripper_action_name,
            gripper_joint,
            gripper_open_position,
            gripper_closed_position,
        )

        # Trigger services
        self.create_service(Trigger, "~/reset", self._on_reset)
        self.create_service(Trigger, "~/open_gripper", self._on_open_gripper)
        self.create_service(Trigger, "~/close_gripper", self._on_close_gripper)

        # Start control loop
        self._running = True
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        # Reset and notify
        self._reset()
        self.get_logger().info(
            "Ready. Drag the interactive marker, then right-click > Start to begin servoing."
        )

    def _on_ik_feedback(self, feedback):
        """Pass feedback through the marker. solve_fn stores the target;
        the control loop tracks it."""
        self._ik_marker.set_seed_configuration(self._latest_joint_positions)
        self._ik_marker.process_feedback(feedback)

    def _control_loop(self):
        """Continuously run one OInK step per tick while not paused"""
        while self._running:
            loop_start = self.get_clock().now()

            if not self._paused:
                with self._lock:
                    q_current = np.array(self._scene.getCurrentJointPositions())
                    self._scene.forwardKinematics(q_current, self._tip_link)

                    if self._reference_filter_tau > 0:
                        filtered = self._reference_filter.update(
                            self._raw_target, self._dt
                        )
                        self._frame_task.setTargetFrameTransform(filtered)
                    else:
                        self._frame_task.setTargetFrameTransform(self._raw_target)

                    try:
                        self._oink.solveIk(
                            self._scene,
                            self._tasks,
                            self._constraints,
                            self._barriers,
                            self._delta_q,
                            self._regularization,
                        )
                    except RuntimeError as e:
                        self._delta_q[:] = 0.0
                        self.get_logger().warn(
                            f"IK solver failed: {e}", throttle_duration_sec=1.0
                        )

                    self._delta_q_full[:] = 0.0
                    self._delta_q_full[self._oink.v_indices] = self._delta_q
                    q_current = self._scene.integrate(q_current, self._delta_q_full)

                    self._scene.setJointPositions(q_current)
                    self._scene.forwardKinematics(q_current, self._tip_link)
                    self._latest_joint_positions = q_current

                self._publish_joint_command(q_current)

            self.get_clock().sleep_until(loop_start + self._period)

    def _publish_joint_command(self, q):
        """Publish a single-point JointTrajectory to command the robot."""
        msg = JointTrajectory()
        msg.joint_names = list(self._joint_names)
        point = JointTrajectoryPoint()
        point.positions = q[self._q_indices].tolist()
        point.time_from_start = self._command_duration_msg
        msg.points = [point]
        self._cmd_pub.publish(msg)

    def _on_start_menu(self, _):
        self._reset()
        self._paused = False
        self.get_logger().info("Servoing started.")

    def _on_pause_menu(self, _):
        self._paused = True
        self.get_logger().info("Servoing paused.")

    def _on_reset_menu(self, _):
        self._reset()
        self.get_logger().info("Reset marker to current hardware state.")

    def _on_open_gripper_menu(self, _):
        _, msg = self._gripper_client.open()
        self.get_logger().info(msg)

    def _on_close_gripper_menu(self, _):
        _, msg = self._gripper_client.close()
        self.get_logger().info(msg)

    def _on_open_gripper(self, _, response):
        response.success, response.message = self._gripper_client.open()
        return response

    def _on_close_gripper(self, _, response):
        response.success, response.message = self._gripper_client.close()
        return response

    def _on_reset(self, _, response):
        self._reset()
        response.success = True
        response.message = "Reset marker to current hardware state."
        self.get_logger().info(response.message)
        return response

    def _reset(self):
        """Reset the marker and control state to the current hardware state."""
        if self._js_subscriber.last_joint_state is None:
            raise RuntimeError("No joint states received, cannot reset to hw state.")

        # Pause it
        self._on_pause_menu(None)

        # Clamp to the joint limits, since (simulated) hardware can report
        # positions slightly outside them.
        joint_config = fromJointState(
            self._js_subscriber.last_joint_state, self._scene, self._conversion_map
        )
        self._latest_joint_positions = self._scene.clampToValidConfiguration(
            joint_config.positions
        )

        with self._lock:
            self._scene.setJointPositions(self._latest_joint_positions)
            self._config_task.setTargetConfiguration(
                self._latest_joint_positions[self._oink.q_indices]
            )
            self._scene.forwardKinematics(self._latest_joint_positions, self._tip_link)
            initial_pose = self._scene.forwardKinematics(
                self._latest_joint_positions, self._tip_link, self._base_link
            )
            self._raw_target = initial_pose.copy()
            if self._reference_filter_tau > 0:
                self._reference_filter.reset(initial_pose)

        self._ik_marker.set_seed_configuration(self._latest_joint_positions)
        pose = se3ToPose(initial_pose)
        self._ik_server.setPose("ik_target", pose)
        self._ik_server.applyChanges()

    def destroy_node(self):
        # Stop the control loop first so it releases the lock
        self._running = False
        self._control_thread.join(timeout=1.0)

        # Shut down executors and join their threads
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
    run_node(CartesianServoNode())
