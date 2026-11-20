#include <roboplan_ros_cpp/type_conversions.hpp>

#include <cmath>
#include <numbers>
#include <optional>
#include <unordered_set>
#include <utility>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace roboplan_ros_cpp {

namespace {

using roboplan::JointType;

/// @brief Below this norm, a quaternion is taken to be zero, which is no rotation.
constexpr double kMinQuaternionNorm = 1e-9;

/// @brief Below this norm, the x axis of a planar joint's rotation is taken to have no component
/// in the xy plane, so the rotation has no angle about z.
constexpr double kMinPlanarAxisNorm = 1e-9;

/// @brief Whether a joint has several degrees of freedom: planar and floating joints.
bool isMultiDof(const roboplan::JointInfo& info) {
  return info.type == JointType::PLANAR || info.type == JointType::FLOATING;
}

/// @brief Checks that point i has `expected` values for a field, or none if it is optional.
tl::expected<void, std::string> checkPointSize(size_t i, const std::string& what, size_t size,
                                               size_t expected, bool optional) {
  if (size == expected || (optional && size == 0)) {
    return {};
  }
  return tl::make_unexpected("Point " + std::to_string(i) + " has " + std::to_string(size) + " " +
                             what + ", expected " + std::to_string(expected));
}

/// @brief Checks the joints and the point sizes of a RoboPlan trajectory.
tl::expected<void, std::string> checkRoboPlanTrajectory(const roboplan::JointTrajectory& trajectory,
                                                        const roboplan::Scene& scene) {
  // The numbers of positions and velocities of the joints; mimic joints have none.
  size_t nq = 0;
  size_t nv = 0;
  for (const auto& joint_name : trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (!joint_info->mimic_info) {
      nq += joint_info->num_position_dofs;
      nv += joint_info->num_velocity_dofs;
    }
  }
  // Positions, velocities and accelerations each have no values, or one per time.
  const auto num_times = trajectory.times.size();
  if (!trajectory.positions.empty() && trajectory.positions.size() != num_times) {
    return tl::make_unexpected("The trajectory has " + std::to_string(num_times) + " times but " +
                               std::to_string(trajectory.positions.size()) +
                               " positions, expected one per time or none");
  }
  if (!trajectory.velocities.empty() && trajectory.velocities.size() != num_times) {
    return tl::make_unexpected("The trajectory has " + std::to_string(num_times) + " times but " +
                               std::to_string(trajectory.velocities.size()) +
                               " velocities, expected one per time or none");
  }
  if (!trajectory.accelerations.empty() && trajectory.accelerations.size() != num_times) {
    return tl::make_unexpected("The trajectory has " + std::to_string(num_times) + " times but " +
                               std::to_string(trajectory.accelerations.size()) +
                               " accelerations, expected one per time or none");
  }
  for (size_t i = 0; i < trajectory.positions.size(); ++i) {
    if (auto ok = checkPointSize(i, "positions", trajectory.positions[i].size(), nq, false); !ok) {
      return ok;
    }
  }
  for (size_t i = 0; i < trajectory.velocities.size(); ++i) {
    if (auto ok = checkPointSize(i, "velocities", trajectory.velocities[i].size(), nv, false);
        !ok) {
      return ok;
    }
  }
  for (size_t i = 0; i < trajectory.accelerations.size(); ++i) {
    if (auto ok = checkPointSize(i, "accelerations", trajectory.accelerations[i].size(), nv, false);
        !ok) {
      return ok;
    }
  }
  return {};
}

/// @brief Returns angle plus the multiple of 2 pi that brings it closest to previous, if any.
double unwrapAngle(double angle, const std::optional<double>& previous) {
  if (!previous) {
    return angle;
  }
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  return angle + kTwoPi * std::round((*previous - angle) / kTwoPi);
}

/// @brief A planar joint's (vx, vy, omega) as a twist.
geometry_msgs::msg::Twist planarTwist(const Eigen::VectorXd& values, size_t v_idx) {
  geometry_msgs::msg::Twist twist;
  twist.linear.x = values(v_idx);
  twist.linear.y = values(v_idx + 1);
  twist.angular.z = values(v_idx + 2);
  return twist;
}

/// @brief Returns a ROS quaternion normalized, or nothing if it is zero.
std::optional<Eigen::Quaterniond> normalizedRotation(const geometry_msgs::msg::Quaternion& msg) {
  Eigen::Quaterniond rotation;
  tf2::fromMsg(msg, rotation);
  if (rotation.norm() < kMinQuaternionNorm) {
    return std::nullopt;
  }
  return rotation.normalized();
}

/// @brief rotation as a ROS quaternion, with the sign closest to previous so that interpolating
/// between points takes the short way. Stores it in previous.
geometry_msgs::msg::Quaternion continuousQuaternion(Eigen::Quaterniond rotation,
                                                    std::optional<Eigen::Quaterniond>& previous) {
  if (previous && previous->dot(rotation) < 0.0) {
    rotation.coeffs() *= -1.0;
  }
  previous = rotation;
  return tf2::toMsg(rotation);
}

/// @brief Appends the joints of a JointTrajectory to trajectory, after the joints it already has.
/// Planar and floating joints are an error. If trajectory has no times yet, they are the
/// message's. A message without joints adds nothing.
tl::expected<void, std::string>
appendJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                      const roboplan::Scene& scene, roboplan::JointTrajectory& trajectory) {
  if (ros_trajectory.joint_names.empty()) {
    return {};
  }

  // Check the joints, and count the RoboPlan positions and velocities of these and of the joints
  // already in trajectory, whose values come first.
  size_t q_start = 0;
  size_t v_start = 0;
  std::unordered_set<std::string> seen;
  for (const auto& joint_name : trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    q_start += joint_info.num_position_dofs;
    v_start += joint_info.num_velocity_dofs;
    seen.insert(joint_name);
  }
  size_t nq = 0;
  size_t nv = 0;
  for (const auto& joint_name : ros_trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (isMultiDof(*joint_info)) {
      return tl::make_unexpected(
          "Joint '" + joint_name +
          "' has several degrees of freedom: a JointTrajectory cannot represent it, "
          "use fromRosTrajectories");
    }
    if (!seen.insert(joint_name).second) {
      return tl::make_unexpected("Joint '" + joint_name + "' appears twice");
    }
    if (!joint_info->mimic_info) {
      nq += joint_info->num_position_dofs;
      nv += joint_info->num_velocity_dofs;
    }
  }

  const auto num_joints = ros_trajectory.joint_names.size();
  const auto num_points = ros_trajectory.points.size();
  for (size_t i = 0; i < num_points; ++i) {
    const auto& point = ros_trajectory.points[i];
    for (const auto& ok :
         {checkPointSize(i, "positions", point.positions.size(), num_joints, false),
          checkPointSize(i, "velocities", point.velocities.size(), num_joints, true),
          checkPointSize(i, "accelerations", point.accelerations.size(), num_joints, true)}) {
      if (!ok) {
        return tl::make_unexpected(ok.error());
      }
    }
  }

  // Make room for the values.
  if (trajectory.times.empty()) {
    for (const auto& point : ros_trajectory.points) {
      trajectory.times.push_back(fromDuration(point.time_from_start));
    }
    trajectory.positions.resize(num_points);
    trajectory.velocities.resize(num_points);
    trajectory.accelerations.resize(num_points);
  }
  for (size_t i = 0; i < num_points; ++i) {
    const auto& point = ros_trajectory.points[i];
    trajectory.positions[i].conservativeResize(q_start + nq);
    if (!point.velocities.empty()) {
      trajectory.velocities[i].conservativeResize(v_start + nv);
    }
    if (!point.accelerations.empty()) {
      trajectory.accelerations[i].conservativeResize(v_start + nv);
    }
  }

  size_t q_idx = q_start;
  size_t v_idx = v_start;
  for (size_t j = 0; j < num_joints; ++j) {
    const auto& joint_name = ros_trajectory.joint_names[j];
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      // Mimic joints are left out: RoboPlan derives them from the joints they mimic.
      continue;
    }
    trajectory.joint_names.push_back(joint_name);
    for (size_t i = 0; i < num_points; ++i) {
      const auto& point = ros_trajectory.points[i];
      auto& positions = trajectory.positions[i];
      auto& velocities = trajectory.velocities[i];
      auto& accelerations = trajectory.accelerations[i];
      switch (joint_info.type) {
      case JointType::REVOLUTE:
      case JointType::PRISMATIC:
        positions(q_idx) = point.positions[j];
        if (!point.velocities.empty()) {
          velocities(v_idx) = point.velocities[j];
        }
        if (!point.accelerations.empty()) {
          accelerations(v_idx) = point.accelerations[j];
        }
        break;
      case JointType::CONTINUOUS:
        positions(q_idx) = std::cos(point.positions[j]);
        positions(q_idx + 1) = std::sin(point.positions[j]);
        if (!point.velocities.empty()) {
          velocities(v_idx) = point.velocities[j];
        }
        if (!point.accelerations.empty()) {
          accelerations(v_idx) = point.accelerations[j];
        }
        break;
      default:
        return tl::make_unexpected("Joint '" + joint_name + "' has an unknown type");
      }
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }
  return {};
}

/// @brief Appends the joints of a MultiDOFJointTrajectory to trajectory, after the joints it
/// already has. Joints other than planar and floating ones are an error. If trajectory has no
/// times yet, they are the message's. A message without joints adds nothing.
tl::expected<void, std::string>
appendMultiDOFJointTrajectory(const trajectory_msgs::msg::MultiDOFJointTrajectory& ros_trajectory,
                              const roboplan::Scene& scene, roboplan::JointTrajectory& trajectory) {
  if (ros_trajectory.joint_names.empty()) {
    return {};
  }

  // Check the joints, and count the RoboPlan positions and velocities of these and of the joints
  // already in trajectory, whose values come first.
  size_t q_start = 0;
  size_t v_start = 0;
  std::unordered_set<std::string> seen;
  for (const auto& joint_name : trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    q_start += joint_info.num_position_dofs;
    v_start += joint_info.num_velocity_dofs;
    seen.insert(joint_name);
  }
  size_t nq = 0;
  size_t nv = 0;
  for (const auto& joint_name : ros_trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (!isMultiDof(*joint_info)) {
      return tl::make_unexpected(
          "Joint '" + joint_name +
          "' has a single degree of freedom: it belongs in the JointTrajectory");
    }
    if (!seen.insert(joint_name).second) {
      return tl::make_unexpected("Joint '" + joint_name + "' appears twice");
    }
    nq += joint_info->num_position_dofs;
    nv += joint_info->num_velocity_dofs;
  }

  const auto num_joints = ros_trajectory.joint_names.size();
  const auto num_points = ros_trajectory.points.size();
  for (size_t i = 0; i < num_points; ++i) {
    const auto& point = ros_trajectory.points[i];
    for (const auto& ok :
         {checkPointSize(i, "transforms", point.transforms.size(), num_joints, false),
          checkPointSize(i, "velocity twists", point.velocities.size(), num_joints, true),
          checkPointSize(i, "acceleration twists", point.accelerations.size(), num_joints, true)}) {
      if (!ok) {
        return tl::make_unexpected(ok.error());
      }
    }
  }

  // Make room for the values.
  if (trajectory.times.empty()) {
    for (const auto& point : ros_trajectory.points) {
      trajectory.times.push_back(fromDuration(point.time_from_start));
    }
    trajectory.positions.resize(num_points);
    trajectory.velocities.resize(num_points);
    trajectory.accelerations.resize(num_points);
  }
  for (size_t i = 0; i < num_points; ++i) {
    const auto& point = ros_trajectory.points[i];
    trajectory.positions[i].conservativeResize(q_start + nq);
    if (!point.velocities.empty()) {
      trajectory.velocities[i].conservativeResize(v_start + nv);
    }
    if (!point.accelerations.empty()) {
      trajectory.accelerations[i].conservativeResize(v_start + nv);
    }
  }

  size_t q_idx = q_start;
  size_t v_idx = v_start;
  for (size_t j = 0; j < num_joints; ++j) {
    const auto& joint_name = ros_trajectory.joint_names[j];
    const auto joint_info = scene.getJointInfo(joint_name).value();
    trajectory.joint_names.push_back(joint_name);
    for (size_t i = 0; i < num_points; ++i) {
      const auto& point = ros_trajectory.points[i];
      const auto& transform = point.transforms[j];
      auto& positions = trajectory.positions[i];
      auto& velocities = trajectory.velocities[i];
      auto& accelerations = trajectory.accelerations[i];
      const auto rotation = normalizedRotation(transform.rotation);
      if (!rotation) {
        return tl::make_unexpected("Point " + std::to_string(i) +
                                   " has a zero quaternion for joint '" + joint_name + "'");
      }
      switch (joint_info.type) {
      case JointType::PLANAR: {
        // Keep only the rotation about z.
        const Eigen::Matrix3d matrix = rotation->toRotationMatrix();
        const double norm = std::hypot(matrix(0, 0), matrix(1, 0));
        if (norm < kMinPlanarAxisNorm) {
          return tl::make_unexpected("Point " + std::to_string(i) +
                                     " has no rotation about z for joint '" + joint_name + "'");
        }
        positions.segment<4>(q_idx) << transform.translation.x, transform.translation.y,
            matrix(0, 0) / norm, matrix(1, 0) / norm;
        if (!point.velocities.empty()) {
          const auto& twist = point.velocities[j];
          velocities.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
        }
        if (!point.accelerations.empty()) {
          const auto& twist = point.accelerations[j];
          accelerations.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
        }
        break;
      }
      case JointType::FLOATING: {
        positions.segment<7>(q_idx) << transform.translation.x, transform.translation.y,
            transform.translation.z, rotation->x(), rotation->y(), rotation->z(), rotation->w();
        Eigen::Matrix<double, 6, 1> twist;
        if (!point.velocities.empty()) {
          tf2::fromMsg(point.velocities[j], twist);
          velocities.segment<6>(v_idx) = twist;
        }
        if (!point.accelerations.empty()) {
          tf2::fromMsg(point.accelerations[j], twist);
          accelerations.segment<6>(v_idx) = twist;
        }
        break;
      }
      default:
        return tl::make_unexpected("Joint '" + joint_name + "' has an unknown type");
      }
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }
  return {};
}

/// @brief The single degree of freedom joints of a RoboPlan trajectory, checked by
/// checkRoboPlanTrajectory, as a JointTrajectory. Its other joints are skipped.
tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
toJointTrajectoryMessage(const roboplan::JointTrajectory& trajectory,
                         const roboplan::Scene& scene) {
  const auto& positions = trajectory.positions;
  const auto& velocities = trajectory.velocities;
  const auto& accelerations = trajectory.accelerations;
  trajectory_msgs::msg::JointTrajectory ros_trajectory;
  ros_trajectory.points.resize(trajectory.times.size());
  for (size_t i = 0; i < trajectory.times.size(); ++i) {
    ros_trajectory.points[i].time_from_start = toDuration(trajectory.times[i]);
  }

  // The values of a point, or no values if the trajectory has none of that kind.
  const Eigen::VectorXd no_values;
  size_t q_idx = 0;
  size_t v_idx = 0;
  for (const auto& joint_name : trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    if (isMultiDof(joint_info)) {
      q_idx += joint_info.num_position_dofs;
      v_idx += joint_info.num_velocity_dofs;
      continue;
    }
    ros_trajectory.joint_names.push_back(joint_name);
    // The previous point's angle, to unwrap continuous joints.
    std::optional<double> previous_angle;
    for (size_t i = 0; i < trajectory.times.size(); ++i) {
      const auto& q = positions.empty() ? no_values : positions[i];
      const auto& v = velocities.empty() ? no_values : velocities[i];
      const auto& a = accelerations.empty() ? no_values : accelerations[i];
      auto& point = ros_trajectory.points[i];
      switch (joint_info.type) {
      case JointType::REVOLUTE:
      case JointType::PRISMATIC:
        if (!positions.empty()) {
          point.positions.push_back(q(q_idx));
        }
        if (!velocities.empty()) {
          point.velocities.push_back(v(v_idx));
        }
        if (!accelerations.empty()) {
          point.accelerations.push_back(a(v_idx));
        }
        break;
      case JointType::CONTINUOUS:
        if (!positions.empty()) {
          const auto angle = unwrapAngle(std::atan2(q(q_idx + 1), q(q_idx)), previous_angle);
          previous_angle = angle;
          point.positions.push_back(angle);
        }
        if (!velocities.empty()) {
          point.velocities.push_back(v(v_idx));
        }
        if (!accelerations.empty()) {
          point.accelerations.push_back(a(v_idx));
        }
        break;
      default:
        return tl::make_unexpected("Joint '" + joint_name + "' has an unknown type");
      }
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }

  // A message without joints has no points either.
  if (ros_trajectory.joint_names.empty()) {
    ros_trajectory.points.clear();
  }
  return ros_trajectory;
}

/// @brief The planar and floating joints of a RoboPlan trajectory, checked by
/// checkRoboPlanTrajectory, as a MultiDOFJointTrajectory. Its other joints are skipped.
tl::expected<trajectory_msgs::msg::MultiDOFJointTrajectory, std::string>
toMultiDOFJointTrajectoryMessage(const roboplan::JointTrajectory& trajectory,
                                 const roboplan::Scene& scene) {
  const auto& positions = trajectory.positions;
  const auto& velocities = trajectory.velocities;
  const auto& accelerations = trajectory.accelerations;
  trajectory_msgs::msg::MultiDOFJointTrajectory ros_trajectory;
  ros_trajectory.points.resize(trajectory.times.size());
  for (size_t i = 0; i < trajectory.times.size(); ++i) {
    ros_trajectory.points[i].time_from_start = toDuration(trajectory.times[i]);
  }

  // The values of a point, or no values if the trajectory has none of that kind.
  const Eigen::VectorXd no_values;
  size_t q_idx = 0;
  size_t v_idx = 0;
  for (const auto& joint_name : trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    if (!isMultiDof(joint_info)) {
      q_idx += joint_info.num_position_dofs;
      v_idx += joint_info.num_velocity_dofs;
      continue;
    }
    ros_trajectory.joint_names.push_back(joint_name);
    // The previous point's rotation, to keep the quaternion's sign.
    std::optional<Eigen::Quaterniond> previous_rotation;
    for (size_t i = 0; i < trajectory.times.size(); ++i) {
      const auto& q = positions.empty() ? no_values : positions[i];
      const auto& v = velocities.empty() ? no_values : velocities[i];
      const auto& a = accelerations.empty() ? no_values : accelerations[i];
      auto& point = ros_trajectory.points[i];
      switch (joint_info.type) {
      case JointType::PLANAR:
        if (!positions.empty()) {
          geometry_msgs::msg::Transform transform;
          transform.translation.x = q(q_idx);
          transform.translation.y = q(q_idx + 1);
          const auto angle = std::atan2(q(q_idx + 3), q(q_idx + 2));
          transform.rotation = continuousQuaternion(
              Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())),
              previous_rotation);
          point.transforms.push_back(transform);
        }
        if (!velocities.empty()) {
          point.velocities.push_back(planarTwist(v, v_idx));
        }
        if (!accelerations.empty()) {
          point.accelerations.push_back(planarTwist(a, v_idx));
        }
        break;
      case JointType::FLOATING:
        if (!positions.empty()) {
          geometry_msgs::msg::Transform transform;
          transform.translation.x = q(q_idx);
          transform.translation.y = q(q_idx + 1);
          transform.translation.z = q(q_idx + 2);
          transform.rotation = continuousQuaternion(
              Eigen::Quaterniond(q(q_idx + 6), q(q_idx + 3), q(q_idx + 4), q(q_idx + 5)),
              previous_rotation);
          point.transforms.push_back(transform);
        }
        if (!velocities.empty()) {
          point.velocities.push_back(tf2::toMsg(Eigen::Matrix<double, 6, 1>(v.segment<6>(v_idx))));
        }
        if (!accelerations.empty()) {
          point.accelerations.push_back(
              tf2::toMsg(Eigen::Matrix<double, 6, 1>(a.segment<6>(v_idx))));
        }
        break;
      default:
        return tl::make_unexpected("Joint '" + joint_name + "' has an unknown type");
      }
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }

  // A message without joints has no points either.
  if (ros_trajectory.joint_names.empty()) {
    ros_trajectory.points.clear();
  }
  return ros_trajectory;
}

}  // namespace

tl::expected<JointStateConverterMap, std::string>
buildConversionMap(const roboplan::Scene& scene, const sensor_msgs::msg::JointState& joint_state) {
  // Setup a mapping joint names to their index in the joint state
  const auto& ros_joint_names = joint_state.name;
  std::unordered_map<std::string, size_t> ros_map;
  ros_map.reserve(ros_joint_names.size());
  for (size_t i = 0; i < ros_joint_names.size(); ++i) {
    ros_map[ros_joint_names[i]] = i;
  }

  // Pull out actuated joint state sizes
  const auto& model = scene.getModel();
  JointStateConverterMap conversion_map;
  conversion_map.nq = model.nq;
  conversion_map.nv = model.nv;
  conversion_map.mappings.reserve(joint_state.name.size());

  // For each actuated joint name, determine type and its location in the Scene's configuration,
  // and put it into the mapping.
  for (const auto& joint_name : joint_state.name) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected("Failed to get joint info for: " + joint_name);
    }
    const auto& info = joint_info.value();

    // TODO: This might not actually be worth failing for
    auto it = ros_map.find(joint_name);
    if (it == ros_map.end()) {
      return tl::make_unexpected("Actuated joint: " + joint_name + ", not in the ROS message!");
    }

    // Grab the joint info and store the relevant information
    const auto joint_id = model.getJointId(joint_name);
    const auto q_idx = model.idx_qs.at(joint_id);
    const auto v_idx = model.idx_vs.at(joint_id);
    JointStateConverterMap::JointMapping mapping{.joint_name = joint_name,
                                                 .ros_index = it->second,
                                                 .q_start = static_cast<size_t>(q_idx),
                                                 .v_start = static_cast<size_t>(v_idx),
                                                 .type = info.type};
    conversion_map.mappings.push_back(mapping);
  }

  return conversion_map;
}

tl::expected<sensor_msgs::msg::JointState, std::string>
toJointState(const roboplan::JointConfiguration& config, const roboplan::Scene& scene) {
  const auto& model = scene.getModel();

  sensor_msgs::msg::JointState msg;
  msg.name.reserve(scene.getJointNames().size());
  msg.position.reserve(scene.getJointNames().size());
  msg.velocity.reserve(scene.getJointNames().size());
  msg.effort.reserve(scene.getJointNames().size());

  for (const auto& joint_name : scene.getJointNames()) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected("Failed to get joint info: " + joint_name);
    }
    const auto& info = joint_info.value();

    const auto joint_id = model.getJointId(joint_name);
    const auto q_idx = model.idx_qs.at(joint_id);
    const auto v_idx = model.idx_vs.at(joint_id);

    msg.name.push_back(joint_name);
    if (info.type == JointType::CONTINUOUS) {
      const auto theta = std::atan2(config.positions(q_idx + 1), config.positions(q_idx));
      msg.position.push_back(theta);
    } else {
      msg.position.push_back(config.positions(q_idx));
    }

    msg.velocity.push_back(v_idx < config.velocities.size() ? config.velocities(v_idx) : 0.0);
    msg.effort.push_back(v_idx < config.accelerations.size() ? config.accelerations(v_idx) : 0.0);
  }

  return msg;
}

tl::expected<roboplan::JointConfiguration, std::string>
fromJointState(const sensor_msgs::msg::JointState& joint_state, const roboplan::Scene& scene,
               const JointStateConverterMap& conversion_map,
               const std::optional<Eigen::VectorXd>& q_reference) {
  if (q_reference && static_cast<size_t>(q_reference->size()) != conversion_map.nq) {
    return tl::make_unexpected("q_reference has " + std::to_string(q_reference->size()) +
                               " positions, the scene has " + std::to_string(conversion_map.nq));
  }

  roboplan::JointConfiguration config;
  config.joint_names = scene.getJointNames();
  // Joints that are not in the message keep their reference positions.
  config.positions = q_reference ? *q_reference : scene.getCurrentJointPositions();
  config.velocities = Eigen::VectorXd::Zero(conversion_map.nv);
  config.accelerations = Eigen::VectorXd::Zero(conversion_map.nv);

  // Process actuated joints using pre-computed indices
  for (const auto& mapping : conversion_map.mappings) {
    const size_t ros_idx = mapping.ros_index;

    if (ros_idx >= joint_state.position.size()) {
      return tl::make_unexpected("Missing position data for joint '" + mapping.joint_name + "'");
    }

    if (mapping.type == JointType::CONTINUOUS) {
      const double theta = joint_state.position.at(ros_idx);
      config.positions(mapping.q_start) = std::cos(theta);
      config.positions(mapping.q_start + 1) = std::sin(theta);
      if (ros_idx < joint_state.velocity.size()) {
        config.velocities(mapping.v_start) = joint_state.velocity.at(ros_idx);
      }
      if (ros_idx < joint_state.effort.size()) {
        config.accelerations(mapping.v_start) = joint_state.effort.at(ros_idx);
      }
    } else {
      config.positions(mapping.q_start) = joint_state.position.at(ros_idx);

      if (ros_idx < joint_state.velocity.size()) {
        config.velocities(mapping.v_start) = joint_state.velocity.at(ros_idx);
      }
      if (ros_idx < joint_state.effort.size()) {
        config.accelerations(mapping.v_start) = joint_state.effort.at(ros_idx);
      }
    }
  }

  return config;
}

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene) {
  if (auto ok = checkRoboPlanTrajectory(roboplan_trajectory, scene); !ok) {
    return tl::make_unexpected(ok.error());
  }
  for (const auto& joint_name : roboplan_trajectory.joint_names) {
    if (isMultiDof(scene.getJointInfo(joint_name).value())) {
      return tl::make_unexpected("Joint '" + joint_name +
                                 "' has several degrees of freedom: a JointTrajectory cannot "
                                 "represent it, use toRosTrajectories");
    }
  }
  return toJointTrajectoryMessage(roboplan_trajectory, scene);
}

tl::expected<RosTrajectories, std::string>
toRosTrajectories(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene) {
  if (auto ok = checkRoboPlanTrajectory(roboplan_trajectory, scene); !ok) {
    return tl::make_unexpected(ok.error());
  }
  auto ros_trajectory = toJointTrajectoryMessage(roboplan_trajectory, scene);
  if (!ros_trajectory) {
    return tl::make_unexpected(ros_trajectory.error());
  }
  auto multi_dof_trajectory = toMultiDOFJointTrajectoryMessage(roboplan_trajectory, scene);
  if (!multi_dof_trajectory) {
    return tl::make_unexpected(multi_dof_trajectory.error());
  }
  return RosTrajectories{std::move(*ros_trajectory), std::move(*multi_dof_trajectory)};
}

tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const roboplan::Scene& scene) {
  roboplan::JointTrajectory trajectory;
  if (auto ok = appendJointTrajectory(ros_trajectory, scene, trajectory); !ok) {
    return tl::make_unexpected(ok.error());
  }
  return trajectory;
}

tl::expected<roboplan::JointTrajectory, std::string>
fromRosTrajectories(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene) {
  // When both messages have joints, they must have the same points at the same times, and
  // velocities and accelerations for all the joints or for none.
  const bool has_single_dof_joints = !ros_trajectory.joint_names.empty();
  const bool has_multi_dof_joints = !multi_dof_trajectory.joint_names.empty();
  if (has_single_dof_joints && has_multi_dof_joints) {
    const auto num_points = ros_trajectory.points.size();
    if (multi_dof_trajectory.points.size() != num_points) {
      return tl::make_unexpected("The JointTrajectory has " + std::to_string(num_points) +
                                 " points, the MultiDOFJointTrajectory " +
                                 std::to_string(multi_dof_trajectory.points.size()));
    }
    for (size_t i = 0; i < num_points; ++i) {
      const auto& point = ros_trajectory.points[i];
      const auto& multi_dof_point = multi_dof_trajectory.points[i];
      if (point.time_from_start != multi_dof_point.time_from_start) {
        return tl::make_unexpected(
            "Point " + std::to_string(i) +
            " has different times in the JointTrajectory and the MultiDOFJointTrajectory");
      }
      if (point.velocities.empty() != multi_dof_point.velocities.empty()) {
        return tl::make_unexpected("Point " + std::to_string(i) +
                                   " has velocities for some joints only");
      }
      if (point.accelerations.empty() != multi_dof_point.accelerations.empty()) {
        return tl::make_unexpected("Point " + std::to_string(i) +
                                   " has accelerations for some joints only");
      }
    }
  }

  roboplan::JointTrajectory trajectory;
  if (auto ok = appendJointTrajectory(ros_trajectory, scene, trajectory); !ok) {
    return tl::make_unexpected(ok.error());
  }
  if (auto ok = appendMultiDOFJointTrajectory(multi_dof_trajectory, scene, trajectory); !ok) {
    return tl::make_unexpected(ok.error());
  }
  return trajectory;
}

geometry_msgs::msg::TransformStamped
toTransformStamped(const roboplan::CartesianConfiguration& cartesian_configuration) {
  Eigen::Isometry3d isometry(cartesian_configuration.tform);
  geometry_msgs::msg::TransformStamped ret = tf2::eigenToTransform(isometry);
  ret.header.frame_id = cartesian_configuration.base_frame;
  ret.child_frame_id = cartesian_configuration.tip_frame;
  return ret;
}

roboplan::CartesianConfiguration
fromTransformStamped(const geometry_msgs::msg::TransformStamped& transform_stamped) {
  roboplan::CartesianConfiguration config;
  config.base_frame = transform_stamped.header.frame_id;
  config.tip_frame = transform_stamped.child_frame_id;
  Eigen::Isometry3d isometry = tf2::transformToEigen(transform_stamped.transform);
  config.tform = isometry.matrix();
  return config;
}

Eigen::Matrix4d poseToSE3(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d isometry;
  tf2::fromMsg(pose, isometry);
  return isometry.matrix();
}

geometry_msgs::msg::Pose se3ToPose(const Eigen::Matrix4d& transform) {
  Eigen::Isometry3d isometry(transform);
  return tf2::toMsg(isometry);
}

}  // namespace roboplan_ros_cpp
