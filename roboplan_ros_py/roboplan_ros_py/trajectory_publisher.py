import threading
import numpy as np


class TrajectoryPublisher:
    """
    Threaded trajectory playback via ROS marker visualization.

    Publishes trajectory waypoints as marker arrays at a fixed rate,
    allowing visual preview of planned motions without requiring a
    ROS node for timer management.
    """

    def __init__(self, scene, visualizer, marker_pub, q_indices):
        """
        Construct a trajectory publisher.

        Manages threaded playback of roboplan trajectories, publishing
        visualization markers at each waypoint. Does not require a ROS
        node, but consumers must bring their own publisher.

        Args:
            scene: RoboPlan Scene object.
            visualizer: RoboplanVisualizer for generating marker arrays.
            marker_pub: ROS publisher for MarkerArray messages.
            q_indices: Joint group indices into the full configuration vector.
        """
        self._scene = scene
        self._visualizer = visualizer
        self._marker_pub = marker_pub
        self._q_indices = q_indices

        self._thread = None
        self._stop_event = threading.Event()
        self._on_complete = None

    def play(self, trajectory, dt=0.01, on_complete=None):
        """
        Start publishing trajectory waypoints as markers.

        Stops any active playback before starting. Waypoints are published
        at a fixed interval on a background thread.

        Args:
            trajectory: A roboplan JointTrajectory.
            dt: Time in seconds between published waypoints.
            on_complete: Optional callback invoked when playback finishes.
        """
        self.stop()
        self._stop_event.clear()
        self._on_complete = on_complete

        positions = trajectory.positions
        q_full = np.array(self._scene.getCurrentJointPositions())

        def _run():
            for pos in positions:
                if self._stop_event.is_set():
                    return
                q_full[self._q_indices] = pos
                self._marker_pub.publish(
                    self._visualizer.markers_from_configuration(q_full)
                )
                self._stop_event.wait(dt)
            if self._on_complete and not self._stop_event.is_set():
                self._on_complete()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
