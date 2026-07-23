"""
Utility classes for roboplan_ros's example packages.
"""

import threading

import numpy as np
import yaml

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

# ParallelGripperCommand is only available in Jazzy and newer.
try:
    from control_msgs.action import ParallelGripperCommand
except ImportError:
    ParallelGripperCommand = None

from roboplan.core import Box
from roboplan_ros_examples import spin_executor


VOLATILE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)

LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class JointStateSubscriber:
    """
    Subscribes and manages a spinner to the specified joint states topic.

    JointStates are often published at the control loop rate, and a subscriber's
    callback functions can eat time in an executor's spinner. This class handles
    standing up the required infrastructure to subscribe to a JointState topic,
    spinning the subscriber, and providing access to the latest message on the
    topic.
    """

    def __init__(self, node_name="joint_state_listener", topic="/joint_states"):
        self.last_joint_state = None

        def joint_state_cb(msg):
            self.last_joint_state = msg

        self._js_node = Node(node_name)
        self._js_sub = self._js_node.create_subscription(
            JointState,
            topic,
            joint_state_cb,
            VOLATILE_QOS,
        )

        self._js_executor = SingleThreadedExecutor()
        self._js_executor.add_node(self._js_node)
        self._js_thread = threading.Thread(
            target=spin_executor, daemon=True, args=(self._js_executor,)
        )
        self._js_thread.start()

    def shutdown(self):
        self._js_node.destroy_node()
        self._js_executor.shutdown()
        self._js_thread.join()
        self._js_node = None


class ParallelGripperClient:
    """
    Wraps an action client for commanding a parallel gripper controller with
    open/close helpers.
    """

    def __init__(self, node, action_name, joint_name, open_position, closed_position):
        self._joint_name = joint_name
        self._open_position = open_position
        self._closed_position = closed_position
        self._client = None
        if ParallelGripperCommand is not None:
            self._client = ActionClient(node, ParallelGripperCommand, action_name)

    def open(self):
        """Opens the gripper, returning a (success, message) tuple."""
        return self._command(self._open_position)

    def close(self):
        """Closes the gripper, returning a (success, message) tuple."""
        return self._command(self._closed_position)

    def _command(self, position):
        if self._client is None:
            return (
                False,
                "ParallelGripperCommand action is not available in this "
                "control_msgs version.",
            )

        if not self._client.wait_for_server(timeout_sec=2.0):
            return False, "Gripper action server not available."

        goal = ParallelGripperCommand.Goal()
        goal.command.name = [self._joint_name]
        goal.command.position = [position]
        self._client.send_goal_async(goal)
        return True, f"Commanded gripper to position {position}."


def add_box_obstacles(scene, config_file):
    """
    Adds box obstacles from a YAML config file to the scene's collision model.

    Returns the parsed list of obstacle dictionaries.
    """
    with open(config_file) as f:
        config = yaml.safe_load(f) or {}
    obstacles = config.get("obstacles", [])
    for obstacle in obstacles:
        tform = np.eye(4)
        tform[:3, 3] = obstacle["xyz"]
        scene.addBoxGeometry(
            obstacle["name"],
            obstacle["parent_frame"],
            Box(*obstacle["size"]),
            tform,
            np.array(obstacle["color"]),
        )
    return obstacles


def obstacle_marker_array(obstacles, ns="roboplan_obstacles"):
    """Builds a MarkerArray visualizing box obstacles from add_box_obstacles."""
    marker_array = MarkerArray()
    for idx, obstacle in enumerate(obstacles):
        marker = Marker()
        marker.header.frame_id = obstacle["parent_frame"]
        marker.ns = ns
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        (
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
        ) = obstacle["xyz"]
        marker.scale.x, marker.scale.y, marker.scale.z = obstacle["size"]
        r, g, b, a = obstacle["color"]
        marker.color = ColorRGBA(r=r, g=g, b=b, a=a)
        marker_array.markers.append(marker)
    return marker_array
