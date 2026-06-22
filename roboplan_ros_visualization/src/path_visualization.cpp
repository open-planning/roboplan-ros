#include <Eigen/Dense>

#include <roboplan/core/path_utils.hpp>

#include <roboplan_ros_visualization/path_visualization.hpp>

namespace roboplan_ros_visualization {

namespace {

/// @brief Builds a ROS Point from the translation of a homogeneous transform.
geometry_msgs::msg::Point rosPointFromTransform(const Eigen::Matrix4d& tform) {
  geometry_msgs::msg::Point point;
  point.x = tform(0, 3);
  point.y = tform(1, 3);
  point.z = tform(2, 3);
  return point;
}

}  // namespace

visualization_msgs::msg::Marker
markerFromJointTrajectory(const roboplan::Scene& scene, const roboplan::JointTrajectory& trajectory,
                          const std::vector<std::string>& frame_names, const std::string& frame_id,
                          const std::string& ns,
                          const std::optional<std_msgs::msg::ColorRGBA>& color, double line_width) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = line_width;
  marker.pose.orientation.w = 1.0;

  // Default to opaque white when no color is supplied.
  if (color) {
    marker.color = *color;
  } else {
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0;
  }

  // Need at least two waypoints to draw a segment.
  if (trajectory.positions.size() < 2) {
    return marker;
  }

  // The trajectory stores group positions in the order of trajectory.joint_names.
  // Embed each waypoint into a full configuration vector, seeded from the scene's current state,
  // before running forward kinematics.
  const Eigen::VectorXi q_indices = scene.getJointPositionIndices(trajectory.joint_names);
  Eigen::VectorXd q = scene.getCurrentJointPositions();
  std::vector<Eigen::VectorXd> q_vec;
  q_vec.reserve(trajectory.positions.size());
  for (const auto& positions : trajectory.positions) {
    q(q_indices) = positions;
    q_vec.push_back(q);
  }

  // Trace each requested frame independently and emit its waypoints as LINE_LIST segment pairs.
  for (const auto& frame_name : frame_names) {
    const std::vector<Eigen::Matrix4d> frame_path =
        roboplan::computeFramePath(scene, q_vec, frame_name);
    for (std::size_t idx = 0; idx + 1 < frame_path.size(); ++idx) {
      marker.points.push_back(rosPointFromTransform(frame_path.at(idx)));
      marker.points.push_back(rosPointFromTransform(frame_path.at(idx + 1)));
    }
  }

  return marker;
}

}  // namespace roboplan_ros_visualization
