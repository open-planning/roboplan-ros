import threading
import time

import numpy as np
import pytest
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.time_source import TimeSource
from roboplan.core import JointTrajectory, Scene, loadTextFile, loadUrdfSceneDescription
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan_ros.visualization import RoboplanVisualizer
from visualization_msgs.msg import Marker

from roboplan_ros_py.trajectory_publisher import TrajectoryPublisher


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def sim_clock() -> Clock:
    """A ROS-time clock that only advances via set_ros_time_override()."""
    clock = Clock(clock_type=ClockType.ROS_TIME)
    time_source = TimeSource()
    time_source.ros_time_is_active = True
    time_source.attach_clock(clock)
    clock.set_ros_time_override(Time(seconds=0, clock_type=ClockType.ROS_TIME))
    return clock


@pytest.fixture
def scene() -> Scene:
    models_dir = get_package_models_dir() / "ur_robot_model"
    urdf_path = models_dir / "ur5_gripper.urdf"
    srdf_path = models_dir / "ur5_gripper.srdf"
    scene = Scene(
        "test_scene",
        loadUrdfSceneDescription(urdf_path, [get_package_share_dir()]),
    )
    scene.importSrdf(loadTextFile(srdf_path))
    return scene


@pytest.fixture
def visualizer(scene: Scene) -> RoboplanVisualizer:
    urdf_path = get_package_models_dir() / "ur_robot_model" / "ur5_gripper.urdf"
    return RoboplanVisualizer(
        scene=scene, urdf_xml=urdf_path.read_text(), group_name="arm"
    )


@pytest.fixture
def make_traj_publisher(scene, visualizer):
    """Builds a TrajectoryPublisher for the UR5 arm and returns it with its publisher."""
    q_indices = scene.getJointGroupInfo("arm").q_indices

    def _make(clock=None):
        pub = RecordingPublisher()
        return TrajectoryPublisher(scene, visualizer, pub, q_indices, clock), pub

    return _make


def make_trajectory(num_waypoints):
    trajectory = JointTrajectory()
    trajectory.positions = [np.full(6, 0.1 * i) for i in range(num_waypoints)]
    return trajectory


def poses(msg):
    return [
        (m.pose.position.x, m.pose.position.y, m.pose.position.z) for m in msg.markers
    ]


def wait_for(predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < end, "timed out"
        time.sleep(0.001)


def test_wall_clock_playback(make_traj_publisher, scene, visualizer):
    traj_publisher, pub = make_traj_publisher()
    done = threading.Event()
    trajectory = make_trajectory(3)

    traj_publisher.play(trajectory, dt=0.01, on_complete=done.set)
    assert done.wait(2.0)

    # Intermediate waypoints may be skipped on a loaded machine, but playback
    # always starts by clearing stale markers and ends at the goal.
    assert 2 <= len(pub.messages) <= 3
    assert pub.messages[0].markers[0].action == Marker.DELETEALL
    assert all(m.action == Marker.ADD for m in pub.messages[-1].markers)

    q_goal = np.array(scene.getCurrentJointPositions())
    q_goal[scene.getJointGroupInfo("arm").q_indices] = trajectory.positions[-1]
    assert poses(pub.messages[-1]) == poses(
        visualizer.markers_from_configuration(q_goal)
    )


def test_sim_clock_playback_follows_clock(make_traj_publisher, sim_clock):
    traj_publisher, pub = make_traj_publisher(sim_clock)
    done = threading.Event()

    traj_publisher.play(make_trajectory(3), dt=0.1, on_complete=done.set)
    wait_for(lambda: len(pub.messages) == 1)

    # Playback is paced by the sim clock, not the wall clock.
    time.sleep(0.05)
    assert len(pub.messages) == 1

    sim_clock.set_ros_time_override(Time(seconds=0.1, clock_type=ClockType.ROS_TIME))
    wait_for(lambda: len(pub.messages) == 2)

    # A large jump skips to the waypoint that is due, ending on the last one.
    sim_clock.set_ros_time_override(Time(seconds=1.0, clock_type=ClockType.ROS_TIME))
    assert done.wait(2.0)
    assert len(pub.messages) == 3


def test_stop_interrupts_paused_clock(make_traj_publisher, sim_clock):
    traj_publisher, pub = make_traj_publisher(sim_clock)
    done = threading.Event()

    traj_publisher.play(make_trajectory(2), dt=1.0, on_complete=done.set)
    wait_for(lambda: len(pub.messages) == 1)

    traj_publisher.stop()
    assert traj_publisher._thread is None
    assert not done.is_set()
    assert len(pub.messages) == 1
