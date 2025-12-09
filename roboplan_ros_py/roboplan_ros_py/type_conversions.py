import numpy as np

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

from roboplan.core import JointConfiguration


def to_duration(time_sec: float) -> Duration:
    """
    Converts a double timestamp to an equivalent ROS Duration.

    Args:
        time_sec: Timestamp in seconds.

    Returns:
        ROS 2 Duration with seconds and nanoseconds.
    """
    duration = Duration()
    duration.sec = int(time_sec)
    duration.nanosec = int((time_sec - duration.sec) * 1e9)
    return duration


def from_duration(duration: Duration) -> float:
    """
    Converts a ROS 2 Duration with seconds and nanoseconds to an equivalent double timestamp.

    Args:
        duration: ROS 2 Duration with seconds and nanoseconds.

    Returns:
        An equivalent double timestamp.
    """
    return duration.sec + duration.nanosec * 1e-9


def to_joint_state(joint_configuration: JointConfiguration) -> JointState:
    """
    Converts a roboplan.JointConfiguration object to a ROS 2 JointState message.

    Args:
        joint_configuration: The roboplan JointConfiguration to convert.

    Returns:
        An equivalent ROS 2 JointState message
    """
    joint_state = JointState()
    joint_state.name = joint_configuration.joint_names
    joint_state.position = joint_configuration.positions.tolist()
    joint_state.velocity = joint_configuration.velocities.tolist()

    # Effort != acceleration but including and kind of abusing the interface.
    joint_state.effort = joint_configuration.accelerations.tolist()
    return joint_state


def from_joint_state(joint_state: JointState):
    """
    Convert the provided ROS 2 JointState message to an equivalent
    roboplan.JointConfiguration.

    Args:
        joint_state: The ROS 2 JointState message to convert.

    Returns:
        An equivalent roboplan.JointConfiguration.
    """
    joint_configuration = JointConfiguration()
    joint_configuration.joint_names = joint_state.name
    joint_configuration.positions = np.array(joint_state.position)
    joint_configuration.velocities = np.array(joint_state.velocity)

    # Effort != acceleration but including and kind of abusing the interface.
    joint_configuration.accelerations = np.array(joint_state.effort)
    return joint_configuration
