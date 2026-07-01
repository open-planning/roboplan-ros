"""
A collection of helper functions and tools for roboplan_ros's example packages.
"""

from rclpy.executors import ExternalShutdownException


def spin_executor(executor):
    """Helper function to spin an executor."""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
