import threading
import time

import numpy as np

# Polling interval when pacing against a ROS clock, which may be paused or scaled.
MAX_WAIT_TIME = 0.005


class TrajectoryPublisher:
    """
    Threaded trajectory playback via ROS marker visualization.

    Publishes trajectory waypoints as marker arrays at a fixed rate,
    allowing visual preview of planned motions without requiring a
    ROS node for timer management.
    """

    def __init__(self, scene, visualizer, marker_pub, q_indices, clock=None):
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
            clock: Optional ROS clock to pace playback against, for example
                to respect simulation time. Defaults to the wall clock.
        """
        self._scene = scene
        self._visualizer = visualizer
        self._marker_pub = marker_pub
        self._q_indices = q_indices
        self._clock = clock

        self._thread = None
        self._stop_event = threading.Event()
        self._on_complete = None

    def play(self, trajectory, dt=0.01, on_complete=None):
        """
        Start publishing trajectory waypoints as markers.

        Stops any active playback before starting. Waypoints are published
        at a fixed interval on a background thread.

        The first message also clears every marker previously published on
        the topic, so a preview never leaves geometry from an earlier one
        (e.g. from a different joint group) behind at a stale pose.

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

        def now():
            if self._clock is None:
                return time.monotonic()
            return self._clock.now().nanoseconds * 1e-9

        def _run():
            start_time = now()
            last_index = len(positions) - 1
            index = 0
            while index <= last_index:
                if self._stop_event.is_set():
                    return
                q_full[self._q_indices] = positions[index]
                markers = self._visualizer.markers_from_configuration(q_full)
                if index == 0:
                    # Clearing in the same message as the first waypoint avoids
                    # a blank frame between the delete and the redraw.
                    markers.markers = (
                        self._visualizer.clear_markers().markers + markers.markers
                    )
                self._marker_pub.publish(markers)
                if index == last_index:
                    break

                # Wait for the next waypoint's deadline. If publishing fell
                # behind, skip ahead to the waypoint that is due now, but always
                # end on the final waypoint so the preview settles at the goal.
                next_index = index + 1
                remaining = start_time + next_index * dt - now()
                if remaining <= 0.0:
                    next_index = int((now() - start_time) / dt)
                while remaining > 0.0 and not self._stop_event.is_set():
                    timeout = (
                        remaining
                        if self._clock is None
                        else min(remaining, MAX_WAIT_TIME)
                    )
                    self._stop_event.wait(timeout)
                    remaining = start_time + next_index * dt - now()
                index = min(next_index, last_index)
            if self._on_complete and not self._stop_event.is_set():
                self._on_complete()

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
