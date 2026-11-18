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

/// @brief Whether a joint goes in a MultiDOFJointTrajectory: planar and floating joints, not mimic
/// joints.
bool isMultiDof(const roboplan::JointInfo& info) {
  return !info.mimic_info &&
         (info.type == roboplan::JointType::PLANAR || info.type == roboplan::JointType::FLOATING);
}

/// @brief The number of positions and velocities (or accelerations) that some joints take in a
/// RoboPlan trajectory. Mimic joints take none.
tl::expected<std::pair<size_t, size_t>, std::string>
trajectoryDofs(const roboplan::Scene& scene, const std::vector<std::string>& joint_names) {
  size_t nq = 0;
  size_t nv = 0;
  for (const auto& joint_name : joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected("Failed to convert the trajectory: " + joint_info.error());
    }
    if (!joint_info->mimic_info) {
      nq += joint_info->num_position_dofs;
      nv += joint_info->num_velocity_dofs;
    }
  }
  return std::make_pair(nq, nv);
}

/// @brief Checks that point i has the expected number of values for a field, or none if the field
/// is optional.
tl::expected<void, std::string> checkPointSize(size_t i, const std::string& what, size_t size,
                                               size_t expected, bool optional) {
  if (size == expected || (optional && size == 0)) {
    return {};
  }
  return tl::make_unexpected("Failed to convert the trajectory: Point " + std::to_string(i) +
                             " has " + std::to_string(size) + " " + what + ", expected " +
                             std::to_string(expected));
}

/// @brief Checks that the joints of a RoboPlan trajectory are in the scene, and that its points
/// have the right number of values.
tl::expected<void, std::string> checkRoboPlanTrajectory(const roboplan::JointTrajectory& trajectory,
                                                        const roboplan::Scene& scene) {
  const auto dofs = trajectoryDofs(scene, trajectory.joint_names);
  if (!dofs) {
    return tl::make_unexpected(dofs.error());
  }
  const auto [nq, nv] = dofs.value();
  for (size_t i = 0; i < trajectory.times.size(); ++i) {
    // Values are only checked if the trajectory has them for this point.
    if (i < trajectory.positions.size()) {
      if (auto ok = checkPointSize(i, "positions", trajectory.positions[i].size(), nq, false);
          !ok) {
        return ok;
      }
    }
    if (i < trajectory.velocities.size()) {
      if (auto ok = checkPointSize(i, "velocities", trajectory.velocities[i].size(), nv, true);
          !ok) {
        return ok;
      }
    }
    if (i < trajectory.accelerations.size()) {
      if (auto ok =
              checkPointSize(i, "accelerations", trajectory.accelerations[i].size(), nv, true);
          !ok) {
        return ok;
      }
    }
  }
  return {};
}

/// @brief Returns point i's values, or nullptr if the trajectory has none for it.
const Eigen::VectorXd* pointValues(const std::vector<Eigen::VectorXd>& values, size_t i) {
  return i < values.size() && values[i].size() > 0 ? &values[i] : nullptr;
}

/// @brief Returns angle plus the multiple of 2 pi that brings it closest to previous, if any.
double unwrapAngle(double angle, const std::optional<double>& previous) {
  if (!previous) {
    return angle;
  }
  constexpr double kTwoPi = 2.0 * std::numbers::pi;
  return angle + kTwoPi * std::round((*previous - angle) / kTwoPi);
}

/// @brief Converts a planar joint's RoboPlan (vx, vy, omega) velocities (or accelerations) to a
/// twist, both in the joint's own frame.
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
  if (rotation.norm() < 1e-9) {
    return std::nullopt;
  }
  return rotation.normalized();
}

/// @brief Returns rotation as a ROS quaternion with the sign closest to previous, if any, and
/// stores it in previous.
/// @details q and -q are the same rotation: keeping the sign closest to the previous point makes
/// interpolating between consecutive points take the short way.
geometry_msgs::msg::Quaternion continuousQuaternion(Eigen::Quaterniond rotation,
                                                    std::optional<Eigen::Quaterniond>& previous) {
  if (previous && previous->dot(rotation) < 0.0) {
    rotation.coeffs() *= -1.0;
  }
  previous = rotation;
  return tf2::toMsg(rotation);
}

/// @brief Converts a ROS JointTrajectory and MultiDOFJointTrajectory pair to one RoboPlan
/// trajectory. The joints of ros_trajectory come first in their order, then those of
/// multi_dof_trajectory in theirs.
tl::expected<roboplan::JointTrajectory, std::string>
fromRosTrajectories(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene) {
  const auto error = [](const std::string& message) {
    return tl::make_unexpected("Failed to convert the trajectory: " + message);
  };

  // Check that each joint is in the right message and appears once, and count the positions and
  // velocities of the RoboPlan trajectory.
  roboplan::JointTrajectory joint_traj;
  size_t nq = 0;
  size_t nv = 0;
  std::unordered_set<std::string> seen;
  for (const bool multi_dof : {false, true}) {
    for (const auto& joint_name :
         multi_dof ? multi_dof_trajectory.joint_names : ros_trajectory.joint_names) {
      const auto joint_info = scene.getJointInfo(joint_name);
      if (!joint_info) {
        return error(joint_info.error());
      }
      if (isMultiDof(*joint_info) && !multi_dof) {
        return error("Joint '" + joint_name +
                     "' has several degrees of freedom: a JointTrajectory cannot represent it, "
                     "use a MultiDOFJointTrajectory");
      }
      if (!isMultiDof(*joint_info) && multi_dof) {
        return error("Joint '" + joint_name +
                     "' has a single degree of freedom: it belongs in the JointTrajectory");
      }
      if (!seen.insert(joint_name).second) {
        return error("Joint '" + joint_name + "' appears twice");
      }
      if (!joint_info->mimic_info) {
        nq += joint_info->num_position_dofs;
        nv += joint_info->num_velocity_dofs;
      }
      joint_traj.joint_names.push_back(joint_name);
    }
  }

  // A message without joints only matters if the other has none either.
  const bool use_multi_dof = !multi_dof_trajectory.joint_names.empty();
  const bool use_single = !ros_trajectory.joint_names.empty() || !use_multi_dof;
  const auto num_points =
      use_single ? ros_trajectory.points.size() : multi_dof_trajectory.points.size();
  if (use_single && use_multi_dof && multi_dof_trajectory.points.size() != num_points) {
    return error("The JointTrajectory has " + std::to_string(num_points) +
                 " points, the MultiDOFJointTrajectory " +
                 std::to_string(multi_dof_trajectory.points.size()));
  }

  const auto num_single = ros_trajectory.joint_names.size();
  const auto num_multi_dof = multi_dof_trajectory.joint_names.size();

  joint_traj.times.reserve(num_points);
  joint_traj.positions.reserve(num_points);
  joint_traj.velocities.reserve(num_points);
  joint_traj.accelerations.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const auto* point = use_single ? &ros_trajectory.points[i] : nullptr;
    const auto* multi_dof_point = use_multi_dof ? &multi_dof_trajectory.points[i] : nullptr;
    if (point && multi_dof_point && point->time_from_start != multi_dof_point->time_from_start) {
      return error("Point " + std::to_string(i) +
                   " has different times in the JointTrajectory and the MultiDOFJointTrajectory");
    }
    if (point) {
      for (const auto& ok :
           {checkPointSize(i, "positions", point->positions.size(), num_single, false),
            checkPointSize(i, "velocities", point->velocities.size(), num_single, true),
            checkPointSize(i, "accelerations", point->accelerations.size(), num_single, true)}) {
        if (!ok) {
          return tl::make_unexpected(ok.error());
        }
      }
    }
    if (multi_dof_point) {
      for (const auto& ok :
           {checkPointSize(i, "transforms", multi_dof_point->transforms.size(), num_multi_dof,
                           false),
            checkPointSize(i, "velocity twists", multi_dof_point->velocities.size(), num_multi_dof,
                           true),
            checkPointSize(i, "acceleration twists", multi_dof_point->accelerations.size(),
                           num_multi_dof, true)}) {
        if (!ok) {
          return tl::make_unexpected(ok.error());
        }
      }
    }

    // Velocities and accelerations must be given for all the joints or for none.
    const auto all_or_none = [&](bool single_has, bool multi_dof_has,
                                 const std::string& what) -> tl::expected<bool, std::string> {
      if (point && multi_dof_point && single_has != multi_dof_has) {
        return error("Point " + std::to_string(i) + " has " + what + " for some joints only");
      }
      return point ? single_has : multi_dof_has;
    };
    const auto has_velocities =
        all_or_none(point && !point->velocities.empty(),
                    multi_dof_point && !multi_dof_point->velocities.empty(), "velocities");
    if (!has_velocities) {
      return tl::make_unexpected(has_velocities.error());
    }
    const auto has_accelerations =
        all_or_none(point && !point->accelerations.empty(),
                    multi_dof_point && !multi_dof_point->accelerations.empty(), "accelerations");
    if (!has_accelerations) {
      return tl::make_unexpected(has_accelerations.error());
    }

    joint_traj.times.push_back(
        fromDuration(point ? point->time_from_start : multi_dof_point->time_from_start));
    joint_traj.positions.emplace_back(nq);
    joint_traj.velocities.emplace_back(*has_velocities ? nv : 0);
    joint_traj.accelerations.emplace_back(*has_accelerations ? nv : 0);
  }

  // Fill in the values one joint at a time, each one after the previous in RoboPlan.
  size_t q_idx = 0;
  size_t v_idx = 0;
  for (const bool multi_dof : {false, true}) {
    const auto& names = multi_dof ? multi_dof_trajectory.joint_names : ros_trajectory.joint_names;
    for (size_t j = 0; j < names.size(); ++j) {
      // The joints were checked above.
      const auto joint_info = scene.getJointInfo(names[j]).value();
      if (joint_info.mimic_info) {
        // Mimic joints have no values of their own: RoboPlan derives them.
        continue;
      }
      for (size_t i = 0; i < num_points; ++i) {
        const auto* point = use_single ? &ros_trajectory.points[i] : nullptr;
        const auto* multi_dof_point = use_multi_dof ? &multi_dof_trajectory.points[i] : nullptr;
        auto& positions = joint_traj.positions[i];
        auto& velocities = joint_traj.velocities[i];
        auto& accelerations = joint_traj.accelerations[i];
        switch (joint_info.type) {
        case roboplan::JointType::REVOLUTE:
        case roboplan::JointType::PRISMATIC:
          positions(q_idx) = point->positions[j];
          if (velocities.size() > 0) {
            velocities(v_idx) = point->velocities[j];
          }
          if (accelerations.size() > 0) {
            accelerations(v_idx) = point->accelerations[j];
          }
          break;
        case roboplan::JointType::CONTINUOUS:
          positions(q_idx) = std::cos(point->positions[j]);
          positions(q_idx + 1) = std::sin(point->positions[j]);
          if (velocities.size() > 0) {
            velocities(v_idx) = point->velocities[j];
          }
          if (accelerations.size() > 0) {
            accelerations(v_idx) = point->accelerations[j];
          }
          break;
        case roboplan::JointType::PLANAR: {
          const auto& transform = multi_dof_point->transforms[j];
          const auto rotation = normalizedRotation(transform.rotation);
          if (!rotation) {
            return error("Point " + std::to_string(i) + " has a zero quaternion for joint '" +
                         names[j] + "'");
          }
          // Only the rotation about z: the direction of the x axis in the xy plane.
          const Eigen::Matrix3d matrix = rotation->toRotationMatrix();
          const double norm = std::hypot(matrix(0, 0), matrix(1, 0));
          if (norm < 1e-9) {
            return error("Point " + std::to_string(i) + " has no rotation about z for joint '" +
                         names[j] + "'");
          }
          positions.segment<4>(q_idx) << transform.translation.x, transform.translation.y,
              matrix(0, 0) / norm, matrix(1, 0) / norm;
          // RoboPlan's (vx, vy, omega) are a twist's linear x and y and its angular z.
          if (velocities.size() > 0) {
            const auto& twist = multi_dof_point->velocities[j];
            velocities.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
          }
          if (accelerations.size() > 0) {
            const auto& twist = multi_dof_point->accelerations[j];
            accelerations.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
          }
          break;
        }
        case roboplan::JointType::FLOATING: {
          const auto& transform = multi_dof_point->transforms[j];
          const auto rotation = normalizedRotation(transform.rotation);
          if (!rotation) {
            return error("Point " + std::to_string(i) + " has a zero quaternion for joint '" +
                         names[j] + "'");
          }
          positions.segment<7>(q_idx) << transform.translation.x, transform.translation.y,
              transform.translation.z, rotation->x(), rotation->y(), rotation->z(), rotation->w();
          // RoboPlan's 6 velocities are a twist's linear then angular components.
          Eigen::Matrix<double, 6, 1> twist;
          if (velocities.size() > 0) {
            tf2::fromMsg(multi_dof_point->velocities[j], twist);
            velocities.segment<6>(v_idx) = twist;
          }
          if (accelerations.size() > 0) {
            tf2::fromMsg(multi_dof_point->accelerations[j], twist);
            accelerations.segment<6>(v_idx) = twist;
          }
          break;
        }
        default:
          return error("Joint '" + names[j] + "' has an unknown type");
        }
      }
      q_idx += joint_info.num_position_dofs;
      v_idx += joint_info.num_velocity_dofs;
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
    if (info.type == roboplan::JointType::CONTINUOUS) {
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

    if (mapping.type == roboplan::JointType::CONTINUOUS) {
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
  trajectory_msgs::msg::JointTrajectory ros_traj;
  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < ros_traj.points.size(); ++i) {
    ros_traj.points[i].time_from_start = toDuration(roboplan_trajectory.times[i]);
  }

  // One joint at a time, each one after the previous in RoboPlan. Mimic joints have no values of
  // their own, planar and floating joints go in a MultiDOFJointTrajectory.
  size_t q_idx = 0;
  size_t v_idx = 0;
  for (const auto& joint_name : roboplan_trajectory.joint_names) {
    // The joints were checked by checkRoboPlanTrajectory.
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    switch (joint_info.type) {
    case roboplan::JointType::REVOLUTE:
    case roboplan::JointType::PRISMATIC:
      ros_traj.joint_names.push_back(joint_name);
      for (size_t i = 0; i < ros_traj.points.size(); ++i) {
        auto& point = ros_traj.points[i];
        if (const auto* positions = pointValues(roboplan_trajectory.positions, i)) {
          point.positions.push_back((*positions)(q_idx));
        }
        if (const auto* velocities = pointValues(roboplan_trajectory.velocities, i)) {
          point.velocities.push_back((*velocities)(v_idx));
        }
        if (const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i)) {
          point.accelerations.push_back((*accelerations)(v_idx));
        }
      }
      break;
    case roboplan::JointType::CONTINUOUS: {
      ros_traj.joint_names.push_back(joint_name);
      std::optional<double> previous_angle;
      for (size_t i = 0; i < ros_traj.points.size(); ++i) {
        auto& point = ros_traj.points[i];
        if (const auto* positions = pointValues(roboplan_trajectory.positions, i)) {
          const auto angle =
              unwrapAngle(std::atan2((*positions)(q_idx + 1), (*positions)(q_idx)), previous_angle);
          previous_angle = angle;
          point.positions.push_back(angle);
        }
        if (const auto* velocities = pointValues(roboplan_trajectory.velocities, i)) {
          point.velocities.push_back((*velocities)(v_idx));
        }
        if (const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i)) {
          point.accelerations.push_back((*accelerations)(v_idx));
        }
      }
      break;
    }
    case roboplan::JointType::PLANAR:
    case roboplan::JointType::FLOATING:
      // These go in the MultiDOFJointTrajectory.
      break;
    default:
      return tl::make_unexpected("Failed to convert the trajectory: Joint '" + joint_name +
                                 "' has an unknown type");
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }
  return ros_traj;
}

tl::expected<trajectory_msgs::msg::MultiDOFJointTrajectory, std::string>
toMultiDOFJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                          const roboplan::Scene& scene) {
  if (auto ok = checkRoboPlanTrajectory(roboplan_trajectory, scene); !ok) {
    return tl::make_unexpected(ok.error());
  }
  trajectory_msgs::msg::MultiDOFJointTrajectory ros_traj;
  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < ros_traj.points.size(); ++i) {
    ros_traj.points[i].time_from_start = toDuration(roboplan_trajectory.times[i]);
  }

  // One joint at a time, each one after the previous in RoboPlan. Only planar and floating joints
  // go in a MultiDOFJointTrajectory.
  size_t q_idx = 0;
  size_t v_idx = 0;
  for (const auto& joint_name : roboplan_trajectory.joint_names) {
    // The joints were checked by checkRoboPlanTrajectory.
    const auto joint_info = scene.getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }
    switch (joint_info.type) {
    case roboplan::JointType::REVOLUTE:
    case roboplan::JointType::PRISMATIC:
    case roboplan::JointType::CONTINUOUS:
      // These go in the JointTrajectory.
      break;
    case roboplan::JointType::PLANAR: {
      ros_traj.joint_names.push_back(joint_name);
      std::optional<Eigen::Quaterniond> previous_rotation;
      for (size_t i = 0; i < ros_traj.points.size(); ++i) {
        auto& point = ros_traj.points[i];
        if (const auto* positions = pointValues(roboplan_trajectory.positions, i)) {
          const auto& q = *positions;
          geometry_msgs::msg::Transform transform;
          transform.translation.x = q(q_idx);
          transform.translation.y = q(q_idx + 1);
          const auto angle = std::atan2(q(q_idx + 3), q(q_idx + 2));
          transform.rotation = continuousQuaternion(
              Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())),
              previous_rotation);
          point.transforms.push_back(transform);
        }
        if (const auto* velocities = pointValues(roboplan_trajectory.velocities, i)) {
          point.velocities.push_back(planarTwist(*velocities, v_idx));
        }
        if (const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i)) {
          point.accelerations.push_back(planarTwist(*accelerations, v_idx));
        }
      }
      break;
    }
    case roboplan::JointType::FLOATING: {
      ros_traj.joint_names.push_back(joint_name);
      std::optional<Eigen::Quaterniond> previous_rotation;
      for (size_t i = 0; i < ros_traj.points.size(); ++i) {
        auto& point = ros_traj.points[i];
        if (const auto* positions = pointValues(roboplan_trajectory.positions, i)) {
          const auto& q = *positions;
          geometry_msgs::msg::Transform transform;
          transform.translation.x = q(q_idx);
          transform.translation.y = q(q_idx + 1);
          transform.translation.z = q(q_idx + 2);
          transform.rotation = continuousQuaternion(
              Eigen::Quaterniond(q(q_idx + 6), q(q_idx + 3), q(q_idx + 4), q(q_idx + 5)),
              previous_rotation);
          point.transforms.push_back(transform);
        }
        // RoboPlan's 6 velocities are a twist's linear then angular components.
        if (const auto* velocities = pointValues(roboplan_trajectory.velocities, i)) {
          point.velocities.push_back(
              tf2::toMsg(Eigen::Matrix<double, 6, 1>(velocities->segment<6>(v_idx))));
        }
        if (const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i)) {
          point.accelerations.push_back(
              tf2::toMsg(Eigen::Matrix<double, 6, 1>(accelerations->segment<6>(v_idx))));
        }
      }
      break;
    }
    default:
      return tl::make_unexpected("Failed to convert the trajectory: Joint '" + joint_name +
                                 "' has an unknown type");
    }
    q_idx += joint_info.num_position_dofs;
    v_idx += joint_info.num_velocity_dofs;
  }

  // Without planar or floating joints, the message is empty.
  if (ros_traj.joint_names.empty()) {
    ros_traj.points.clear();
  }
  return ros_traj;
}

tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const roboplan::Scene& scene) {
  return fromRosTrajectories(ros_trajectory, trajectory_msgs::msg::MultiDOFJointTrajectory(),
                             scene);
}

tl::expected<roboplan::JointTrajectory, std::string>
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory,
                    const trajectory_msgs::msg::MultiDOFJointTrajectory& multi_dof_trajectory,
                    const roboplan::Scene& scene) {
  return fromRosTrajectories(ros_trajectory, multi_dof_trajectory, scene);
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
