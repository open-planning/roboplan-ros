"""
A collection of small helper functions and utilities for roboplan_ros's example packages.
"""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.executors import (
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import JointState


VOLATILE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def run_node(node):
    """Spins a node using a MultiThreadedExecutor and handles exceptions shutdowns."""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        del node
        rclpy.try_shutdown()


def spin_executor(executor):
    """Helper function to spin an executor."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass


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
