"""
A collection of small helper functions for roboplan_ros's example packages.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor


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
        rclpy.try_shutdown()


def spin_executor(executor):
    """Helper function to spin an executor."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
