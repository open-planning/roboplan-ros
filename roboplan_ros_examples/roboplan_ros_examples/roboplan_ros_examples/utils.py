"""
Utility classes for roboplan_ros's example packages.
"""

import threading

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from roboplan_ros_examples import spin_executor


VOLATILE_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
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
