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
  const auto check_field = [&](const std::string& what, const std::vector<Eigen::VectorXd>& values,
                               size_t expected) -> tl::expected<void, std::string> {
    if (!values.empty() && values.size() != trajectory.times.size()) {
      return tl::make_unexpected("It has " + std::to_string(trajectory.times.size()) +
                                 " times but " + std::to_string(values.size()) + " " + what +
                                 ", expected one per time or none");
    }
    for (size_t i = 0; i < values.size(); ++i) {
      if (auto ok = checkPointSize(i, what, values[i].size(), expected, false); !ok) {
        return ok;
      }
    }
    return {};
  };
  if (auto ok = check_field("positions", trajectory.positions, nq); !ok) {
    return ok;
  }
  if (auto ok = check_field("velocities", trajectory.velocities, nv); !ok) {
    return ok;
  }
  return check_field("accelerations", trajectory.accelerations, nv);
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

/// @brief The shared implementation of fromJointTrajectory and fromRosTrajectories.
tl::expected<roboplan::JointTrajectory, std::string>
fromRosMessages(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                const roboplan::Scene& scene) {
  // Records a joint whose message was checked, and counts its RoboPlan positions and velocities.
  roboplan::JointTrajectory joint_traj;
  size_t nq = 0;
  size_t nv = 0;
  std::unordered_set<std::string> seen;
  const auto add_joint =
      [&](const std::string& joint_name,
          const roboplan::JointInfo& joint_info) -> tl::expected<void, std::string> {
    if (!seen.insert(joint_name).second) {
      return tl::make_unexpected("Joint '" + joint_name + "' appears twice");
    }
    if (joint_info.mimic_info) {
      // Mimic joints are left out: RoboPlan derives them from the joints they mimic.
      return {};
    }
    nq += joint_info.num_position_dofs;
    nv += joint_info.num_velocity_dofs;
    joint_traj.joint_names.push_back(joint_name);
    return {};
  };
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
    if (auto ok = add_joint(joint_name, *joint_info); !ok) {
      return tl::make_unexpected(ok.error());
    }
  }
  for (const auto& joint_name : multi_dof_trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    if (!isMultiDof(*joint_info)) {
      return tl::make_unexpected(
          "Joint '" + joint_name +
          "' has a single degree of freedom: it belongs in the JointTrajectory");
    }
    if (auto ok = add_joint(joint_name, *joint_info); !ok) {
      return tl::make_unexpected(ok.error());
    }
  }

  // A message without joints is ignored; without joints at all, the trajectory has no points.
  const bool has_single_dof_joints = !ros_trajectory.joint_names.empty();
  const bool has_multi_dof_joints = !multi_dof_trajectory.joint_names.empty();
  const auto num_points = has_single_dof_joints  ? ros_trajectory.points.size()
                          : has_multi_dof_joints ? multi_dof_trajectory.points.size()
                                                 : 0;
  if (has_single_dof_joints && has_multi_dof_joints &&
      multi_dof_trajectory.points.size() != num_points) {
    return tl::make_unexpected("The JointTrajectory has " + std::to_string(num_points) +
                               " points, the MultiDOFJointTrajectory " +
                               std::to_string(multi_dof_trajectory.points.size()));
  }

  const auto num_single_dof_joints = ros_trajectory.joint_names.size();
  const auto num_multi_dof_joints = multi_dof_trajectory.joint_names.size();

  joint_traj.times.reserve(num_points);
  joint_traj.positions.reserve(num_points);
  joint_traj.velocities.reserve(num_points);
  joint_traj.accelerations.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    // Both messages must have the same time, and velocities and accelerations for all the joints
    // or for none.
    std::optional<builtin_interfaces::msg::Duration> time;
    std::optional<bool> has_velocities;
    std::optional<bool> has_accelerations;
    if (has_single_dof_joints) {
      const auto& point = ros_trajectory.points[i];
      for (const auto& ok :
           {checkPointSize(i, "positions", point.positions.size(), num_single_dof_joints, false),
            checkPointSize(i, "velocities", point.velocities.size(), num_single_dof_joints, true),
            checkPointSize(i, "accelerations", point.accelerations.size(), num_single_dof_joints,
                           true)}) {
        if (!ok) {
          return tl::make_unexpected(ok.error());
        }
      }
      time = point.time_from_start;
      has_velocities = !point.velocities.empty();
      has_accelerations = !point.accelerations.empty();
    }
    if (has_multi_dof_joints) {
      const auto& point = multi_dof_trajectory.points[i];
      if (time && *time != point.time_from_start) {
        return tl::make_unexpected(
            "Point " + std::to_string(i) +
            " has different times in the JointTrajectory and the MultiDOFJointTrajectory");
      }
      for (const auto& ok :
           {checkPointSize(i, "transforms", point.transforms.size(), num_multi_dof_joints, false),
            checkPointSize(i, "velocity twists", point.velocities.size(), num_multi_dof_joints,
                           true),
            checkPointSize(i, "acceleration twists", point.accelerations.size(),
                           num_multi_dof_joints, true)}) {
        if (!ok) {
          return tl::make_unexpected(ok.error());
        }
      }
      if (has_velocities && *has_velocities == point.velocities.empty()) {
        return tl::make_unexpected("Point " + std::to_string(i) +
                                   " has velocities for some joints only");
      }
      if (has_accelerations && *has_accelerations == point.accelerations.empty()) {
        return tl::make_unexpected("Point " + std::to_string(i) +
                                   " has accelerations for some joints only");
      }
      time = point.time_from_start;
      has_velocities = !point.velocities.empty();
      has_accelerations = !point.accelerations.empty();
    }
    joint_traj.times.push_back(fromDuration(*time));
    joint_traj.positions.emplace_back(nq);
    joint_traj.velocities.emplace_back(*has_velocities ? nv : 0);
    joint_traj.accelerations.emplace_back(*has_accelerations ? nv : 0);
  }

  // Fills in the values of joint j of its message, after those of the previous joints.
  size_t q_idx = 0;
  size_t v_idx = 0;
  const auto fill_joint = [&](const std::string& joint_name,
                              size_t j) -> tl::expected<void, std::string> {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      // Mimic joints have no values of their own.
      return {};
    }
    for (size_t i = 0; i < num_points; ++i) {
      auto& positions = joint_traj.positions[i];
      auto& velocities = joint_traj.velocities[i];
      auto& accelerations = joint_traj.accelerations[i];
      switch (joint_info.type) {
      case JointType::REVOLUTE:
      case JointType::PRISMATIC: {
        const auto& point = ros_trajectory.points[i];
        positions(q_idx) = point.positions[j];
        if (velocities.size() > 0) {
          velocities(v_idx) = point.velocities[j];
        }
        if (accelerations.size() > 0) {
          accelerations(v_idx) = point.accelerations[j];
        }
        break;
      }
      case JointType::CONTINUOUS: {
        const auto& point = ros_trajectory.points[i];
        positions(q_idx) = std::cos(point.positions[j]);
        positions(q_idx + 1) = std::sin(point.positions[j]);
        if (velocities.size() > 0) {
          velocities(v_idx) = point.velocities[j];
        }
        if (accelerations.size() > 0) {
          accelerations(v_idx) = point.accelerations[j];
        }
        break;
      }
      case JointType::PLANAR: {
        const auto& point = multi_dof_trajectory.points[i];
        const auto& transform = point.transforms[j];
        const auto rotation = normalizedRotation(transform.rotation);
        if (!rotation) {
          return tl::make_unexpected("Point " + std::to_string(i) +
                                     " has a zero quaternion for joint '" + joint_name + "'");
        }
        // Keep only the rotation about z.
        const Eigen::Matrix3d matrix = rotation->toRotationMatrix();
        const double norm = std::hypot(matrix(0, 0), matrix(1, 0));
        if (norm < kMinPlanarAxisNorm) {
          return tl::make_unexpected("Point " + std::to_string(i) +
                                     " has no rotation about z for joint '" + joint_name + "'");
        }
        positions.segment<4>(q_idx) << transform.translation.x, transform.translation.y,
            matrix(0, 0) / norm, matrix(1, 0) / norm;
        if (velocities.size() > 0) {
          const auto& twist = point.velocities[j];
          velocities.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
        }
        if (accelerations.size() > 0) {
          const auto& twist = point.accelerations[j];
          accelerations.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
        }
        break;
      }
      case JointType::FLOATING: {
        const auto& point = multi_dof_trajectory.points[i];
        const auto& transform = point.transforms[j];
        const auto rotation = normalizedRotation(transform.rotation);
        if (!rotation) {
          return tl::make_unexpected("Point " + std::to_string(i) +
                                     " has a zero quaternion for joint '" + joint_name + "'");
        }
        positions.segment<7>(q_idx) << transform.translation.x, transform.translation.y,
            transform.translation.z, rotation->x(), rotation->y(), rotation->z(), rotation->w();
        Eigen::Matrix<double, 6, 1> twist;
        if (velocities.size() > 0) {
          tf2::fromMsg(point.velocities[j], twist);
          velocities.segment<6>(v_idx) = twist;
        }
        if (accelerations.size() > 0) {
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
    return {};
  };
  for (size_t j = 0; j < ros_trajectory.joint_names.size(); ++j) {
    if (auto ok = fill_joint(ros_trajectory.joint_names[j], j); !ok) {
      return tl::make_unexpected(ok.error());
    }
  }
  for (size_t j = 0; j < multi_dof_trajectory.joint_names.size(); ++j) {
    if (auto ok = fill_joint(multi_dof_trajectory.joint_names[j], j); !ok) {
      return tl::make_unexpected(ok.error());
    }
  }
  return joint_traj;
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

namespace {

/// @brief The shared implementation of toJointTrajectory and toRosTrajectories. If
/// allow_multi_dof is false, as for toJointTrajectory, planar and floating joints are an error.
tl::expected<RosTrajectories, std::string>
toRosMessages(const roboplan::JointTrajectory& roboplan_trajectory, const roboplan::Scene& scene,
              bool allow_multi_dof) {
  if (auto ok = checkRoboPlanTrajectory(roboplan_trajectory, scene); !ok) {
    return tl::make_unexpected(ok.error());
  }
  // checkRoboPlanTrajectory made sure each of these has no values, or one per time.
  const auto& positions = roboplan_trajectory.positions;
  const auto& velocities = roboplan_trajectory.velocities;
  const auto& accelerations = roboplan_trajectory.accelerations;
  RosTrajectories ros_trajectories;
  auto& [joint_traj, multi_dof_traj] = ros_trajectories;
  joint_traj.points.resize(roboplan_trajectory.times.size());
  multi_dof_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < roboplan_trajectory.times.size(); ++i) {
    joint_traj.points[i].time_from_start = toDuration(roboplan_trajectory.times[i]);
    multi_dof_traj.points[i].time_from_start = toDuration(roboplan_trajectory.times[i]);
  }

  // The values of a point, or no values if the trajectory has none of that kind.
  const Eigen::VectorXd no_values;
  size_t q_idx = 0;
  size_t v_idx = 0;
  for (const auto& joint_name : roboplan_trajectory.joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    if (isMultiDof(joint_info) && !allow_multi_dof) {
      return tl::make_unexpected("Joint '" + joint_name +
                                 "' has several degrees of freedom: a JointTrajectory cannot "
                                 "represent it, use toRosTrajectories");
    }
    if (isMultiDof(joint_info)) {
      multi_dof_traj.joint_names.push_back(joint_name);
    } else {
      joint_traj.joint_names.push_back(joint_name);
    }
    // The previous point's values, to unwrap angles and keep quaternion signs.
    std::optional<double> previous_angle;
    std::optional<Eigen::Quaterniond> previous_rotation;
    for (size_t i = 0; i < roboplan_trajectory.times.size(); ++i) {
      const auto& q = positions.empty() ? no_values : positions[i];
      const auto& v = velocities.empty() ? no_values : velocities[i];
      const auto& a = accelerations.empty() ? no_values : accelerations[i];
      switch (joint_info.type) {
      case JointType::REVOLUTE:
      case JointType::PRISMATIC: {
        auto& point = joint_traj.points[i];
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
      }
      case JointType::CONTINUOUS: {
        auto& point = joint_traj.points[i];
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
      }
      case JointType::PLANAR: {
        auto& point = multi_dof_traj.points[i];
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
      }
      case JointType::FLOATING: {
        auto& point = multi_dof_traj.points[i];
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
      }
      default:
        return tl::make_unexpected("Joint '" + joint_name + "' has an unknown type");
      }
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }

  // A message without joints has no points either.
  if (joint_traj.joint_names.empty()) {
    joint_traj.points.clear();
  }
  if (multi_dof_traj.joint_names.empty()) {
    multi_dof_traj.points.clear();
  }
  return ros_trajectories;
}

}  // namespace

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene) {
  auto ros_trajectories = toRosMessages(roboplan_trajectory, scene, /*allow_multi_dof=*/false);
  if (!ros_trajectories) {
    return tl::make_unexpected("Failed to convert the trajectory: " + ros_trajectories.error());
  }
  return std::move(ros_trajectories->first);
}

tl::expected<RosTrajectories, std::string>
toRosTrajectories(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene) {
  auto ros_trajectories = toRosMessages(roboplan_trajectory, scene, /*allow_multi_dof=*/true);
  if (!ros_trajectories) {
    return tl::make_unexpected("Failed to convert the trajectory: " + ros_trajectories.error());
  }
  return ros_trajectories;
}

tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const roboplan::Scene& scene) {
  auto joint_traj =
      fromRosMessages(ros_trajectory, trajectory_msgs::msg::MultiDOFJointTrajectory(), scene);
  if (!joint_traj) {
    return tl::make_unexpected("Failed to convert the trajectory: " + joint_traj.error());
  }
  return joint_traj;
}

tl::expected<roboplan::JointTrajectory, std::string>
fromRosTrajectories(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene) {
  auto joint_traj = fromRosMessages(ros_trajectory, multi_dof_trajectory, scene);
  if (!joint_traj) {
    return tl::make_unexpected("Failed to convert the trajectory: " + joint_traj.error());
  }
  return joint_traj;
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
