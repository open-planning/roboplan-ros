import unittest
import roboplan_ros_py

# Ok this is dumb because obviously the ROS python types don't match their corresponding
# c++ types, so we end up with an error.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TestRoboplanRosPy(unittest.TestCase):

    def test_type_conversions(self):
        # Construct a Trajectory with a single point
        ros_traj = JointTrajectory()
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        roboplan_ros_py.fromJointTrajectory(ros_traj)
