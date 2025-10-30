#!/usr/bin/env python3

import numpy as np
import pinocchio as pin
from typing import Optional

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState

from roboplan import Scene
from roboplan_ros_py.type_conversions import se3_to_pose


class RoboplanVisualizer:
    """
    Utility for visualizing a robot configuration using markers in RViz.
    Does not use TF or robot_state_publisher - computes forward kinematics
    directly from the Scene and publishes visualization markers.
    """

    def __init__(
        self,
        node: Node,
        scene: Scene,
        urdf_xml: str,
        package_paths: list = [],
        color: Optional[ColorRGBA] = None,
        update_rate: Optional[float] = None,
        namespace: str = "/roboplan",
    ):
        """
        Initialize the robot visualizer.

        Args:
            node: ROS node instance.
            scene: A fully configured RoboPlan scene.
            urdf_xml: URDF XML string (already processed with xacro if needed).
            package_paths: List of package paths for mesh resolution.
            namespace: Namespace for the marker publisher and joint state subscriber.
            color: Default color for the robot visualization, if set will override defaults.
            update_rate: If provided, subscribes to joint_states and auto-updates.
                         If None, call visualize_configuration() manually.
        """
        self.node = node
        self.scene = scene
        self.namespace = namespace
        self.color = color

        # Build visual geometry model from URDF. I think if/when the scene model has python bindings
        # this will be unnecessary.
        self._build_visual_model(urdf_xml, package_paths or [])

        # Create marker publisher
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.marker_pub = node.create_publisher(
            MarkerArray, f"{namespace}/markers", qos
        )

        # If update rate provided, subscribe to joint states and update at a fixed rate
        self._joint_state_sub = None
        self._timer = None
        self._latest_joint_positions = None
        if update_rate is not None:
            self._joint_state_sub = node.create_subscription(
                JointState,
                f"{namespace}/joint_states",
                self._joint_state_callback,
                qos,
            )
            self._timer = node.create_timer(1.0 / update_rate, self._timer_callback)

        self.node.get_logger().info(
            f"Constructed RoboplanVisualizer in namespace: {namespace}"
        )

    def _build_visual_model(self, urdf_xml: str, package_paths: list):
        """
        Build visual geometry model from URDF using Pinocchio.

        Like many things this could be removed when Pinocchio has python bindings.
        """
        self.pin_model = pin.buildModelFromXML(urdf_xml)
        self.visual_model = pin.buildGeomFromUrdfString(
            self.pin_model,
            urdf_xml,
            pin.GeometryType.VISUAL,
            package_dirs=package_paths,
        )
        self.visual_data = self.visual_model.createData()

    def _joint_state_callback(self, msg: JointState):
        self._latest_joint_positions = np.array(msg.position)

    def _timer_callback(self):
        if self._latest_joint_positions is not None:
            self.visualize_configuration(self._latest_joint_positions)

    def visualize_configuration(self, q: np.ndarray):
        """
        Visualize a robot configuration using markers.

        Args:
            q: Joint positions (full robot state)
        """
        marker_array = MarkerArray()

        # Update geometry placements based on joint configuration
        pin.updateGeometryPlacements(
            self.pin_model,
            pin.Data(self.pin_model),
            self.visual_model,
            self.visual_data,
            q,
        )
        for idx, geom_obj in enumerate(self.visual_model.geometryObjects):
            try:
                placement = self.visual_data.oMg[idx]
                marker = self._create_geometry_marker(idx, geom_obj, placement)
                if marker:
                    marker_array.markers.append(marker)

            except Exception as e:
                self.node.get_logger().debug(
                    f"Could not create marker for geometry {geom_obj.name}: {e}"
                )
                continue

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    def _create_geometry_marker(
        self, marker_id: int, geom_obj, placement
    ) -> Optional[Marker]:
        """
        Create a marker for a Pinocchio geometry object.

        Args:
            marker_id: Marker ID
            geom_obj: Pinocchio GeometryObject
            placement: SE3 placement of the geometry

        Returns:
            Marker or None
        """
        marker = Marker()
        marker.header.frame_id = "world"  # or your base frame
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = f"{self.namespace}/{geom_obj.name}"
        marker.id = marker_id
        marker.action = Marker.ADD

        # Convert SE3 placement to pose
        marker.pose = se3_to_pose(placement.homogeneous)

        # Handle different geometry types
        # TODO: Something more intelligent with pin's types?
        geometry = geom_obj.geometry
        if isinstance(geometry, pin.hppfcl.Box):
            marker.type = Marker.CUBE
            marker.scale.x = float(geometry.halfSide[0] * 2)
            marker.scale.y = float(geometry.halfSide[1] * 2)
            marker.scale.z = float(geometry.halfSide[2] * 2)

        elif isinstance(geometry, pin.hppfcl.Sphere):
            marker.type = Marker.SPHERE
            r = float(geometry.radius * 2)
            marker.scale.x = marker.scale.y = marker.scale.z = r

        elif isinstance(geometry, pin.hppfcl.Cylinder):
            marker.type = Marker.CYLINDER
            marker.scale.x = float(geometry.radius * 2)
            marker.scale.y = float(geometry.radius * 2)
            marker.scale.z = float(geometry.halfLength * 2)

        elif isinstance(geometry, (pin.hppfcl.Convex, pin.hppfcl.BVHModelBase)):
            # For mesh geometries
            marker.type = Marker.MESH_RESOURCE

            # Ensure proper URI format because RViz gets angry at absolute paths
            if hasattr(geom_obj, "meshPath") and geom_obj.meshPath:
                mesh_path = geom_obj.meshPath

                if not mesh_path.startswith("package://") and not mesh_path.startswith(
                    "file://"
                ):
                    marker.mesh_resource = f"file://{mesh_path}"
                else:
                    marker.mesh_resource = mesh_path
            else:
                # If there isn't a mesh path we can't do anything?
                return None

            if hasattr(geom_obj, "meshScale"):
                marker.scale.x = float(geom_obj.meshScale[0])
                marker.scale.y = float(geom_obj.meshScale[1])
                marker.scale.z = float(geom_obj.meshScale[2])
            else:
                marker.scale.x = marker.scale.y = marker.scale.z = 1.0

        else:
            # Unknown geometry type, skip
            self.node.get_logger().warn(
                f"Unknown geometry type for {geom_obj.name}: {type(geometry)}"
            )
            return None

        # Use embedded colorings for meshes
        if marker.type == Marker.MESH_RESOURCE:
            marker.mesh_use_embedded_materials = True
        # If the geometry has a color user it
        if hasattr(geom_obj, "meshColor") and len(geom_obj.meshColor) >= 4:
            marker.color.r = float(geom_obj.meshColor[0])
            marker.color.g = float(geom_obj.meshColor[1])
            marker.color.b = float(geom_obj.meshColor[2])
            marker.color.a = float(geom_obj.meshColor[3])
        # Apply colorings if set
        if self.color:
            marker.color = self.color

        return marker

    def clear_markers(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def set_color(self, color: ColorRGBA):
        self.color = color
