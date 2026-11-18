#include <roboplan_ros_cpp/type_conversions.hpp>

#include <cmath>
#include <numbers>
#include <optional>
#include <unordered_set>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace roboplan_ros_cpp {

namespace {

/// @brief A joint of a trajectory, with where its values are in RoboPlan.
struct TrajectoryJoint {
  std::string name;
  roboplan::JointInfo info;

  /// @brief Index of the joint's first position in RoboPlan, set by layOutJoints.
  size_t q_start = 0;

  /// @brief Index of the joint's first velocity (and acceleration) in RoboPlan, set by
  /// layOutJoints.
  size_t v_start = 0;

  /// @brief Whether the joint is a mimic joint, which has no values of its own in RoboPlan: they
  /// are derived from the joint it mimics.
  bool isMimic() const { return info.mimic_info.has_value(); }

  /// @brief Whether the joint goes in a JointTrajectory.
  bool isSingleDof() const {
    return !isMimic() && (info.type == roboplan::JointType::REVOLUTE ||
                          info.type == roboplan::JointType::PRISMATIC ||
                          info.type == roboplan::JointType::CONTINUOUS);
  }

  /// @brief Whether the joint goes in a MultiDOFJointTrajectory.
  bool isMultiDof() const {
    return !isMimic() &&
           (info.type == roboplan::JointType::PLANAR || info.type == roboplan::JointType::FLOATING);
  }
};

/// @brief Looks up the JointInfo of each joint of a trajectory.
/// @return The joints in the order of joint_names, or an error if one is unknown.
tl::expected<std::vector<TrajectoryJoint>, std::string>
trajectoryJoints(const roboplan::Scene& scene, const std::vector<std::string>& joint_names) {
  std::vector<TrajectoryJoint> joints;
  joints.reserve(joint_names.size());
  for (const auto& joint_name : joint_names) {
    const auto joint_info = scene.getJointInfo(joint_name);
    if (!joint_info) {
      return tl::make_unexpected(joint_info.error());
    }
    joints.push_back({joint_name, joint_info.value()});
  }
  return joints;
}

/// @brief Total number of RoboPlan positions and velocities of some trajectory joints.
struct TrajectorySize {
  size_t nq = 0;
  size_t nv = 0;
};

/// @brief Places the joints one after the other in RoboPlan's positions and velocities, setting
/// their q_start and v_start. Mimic joints take no room.
/// @return The total number of positions and velocities.
TrajectorySize layOutJoints(std::vector<TrajectoryJoint>& joints) {
  TrajectorySize size;
  for (auto& joint : joints) {
    joint.q_start = size.nq;
    joint.v_start = size.nv;
    if (!joint.isMimic()) {
      size.nq += joint.info.num_position_dofs;
      size.nv += joint.info.num_velocity_dofs;
    }
  }
  return size;
}

/// @brief The names of the joints for which keep returns true, in order.
std::vector<std::string> jointNames(const std::vector<TrajectoryJoint>& joints,
                                    bool (TrajectoryJoint::*keep)() const) {
  std::vector<std::string> names;
  for (const auto& joint : joints) {
    if ((joint.*keep)()) {
      names.push_back(joint.name);
    }
  }
  return names;
}

/// @brief The number of values a point has for one of its fields, and how many it should have.
struct PointField {
  std::string what;
  size_t size;
  size_t expected;

  /// @brief Whether the point may also have no values for the field.
  bool optional;
};

/// @brief Checks that each field of a point has the expected size, or is empty if it is optional.
tl::expected<void, std::string> checkPointSizes(size_t point_index,
                                                const std::vector<PointField>& fields) {
  for (const auto& field : fields) {
    if (field.size != field.expected && !(field.optional && field.size == 0)) {
      return tl::make_unexpected("Failed to convert the trajectory: Point " +
                                 std::to_string(point_index) + " has " +
                                 std::to_string(field.size) + " " + field.what + ", expected " +
                                 std::to_string(field.expected));
    }
  }
  return {};
}

/// @brief Checks the sizes of the values of point i of a RoboPlan trajectory.
tl::expected<void, std::string> checkRoboPlanPoint(const roboplan::JointTrajectory& trajectory,
                                                   size_t i, const TrajectorySize& size) {
  // Values are only checked if the trajectory has them for this point.
  std::vector<PointField> fields;
  if (i < trajectory.positions.size()) {
    fields.push_back(
        {"positions", static_cast<size_t>(trajectory.positions[i].size()), size.nq, false});
  }
  if (i < trajectory.velocities.size()) {
    fields.push_back(
        {"velocities", static_cast<size_t>(trajectory.velocities[i].size()), size.nv, true});
  }
  if (i < trajectory.accelerations.size()) {
    fields.push_back(
        {"accelerations", static_cast<size_t>(trajectory.accelerations[i].size()), size.nv, true});
  }
  return checkPointSizes(i, fields);
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

/// @brief Converts the RoboPlan velocities (or accelerations) of a planar or floating joint to a
/// twist, both in the joint's own frame.
geometry_msgs::msg::Twist toTwist(roboplan::JointType type, const Eigen::VectorXd& values,
                                  size_t v_idx) {
  geometry_msgs::msg::Twist twist;
  if (type == roboplan::JointType::PLANAR) {
    twist.linear.x = values(v_idx);
    twist.linear.y = values(v_idx + 1);
    twist.angular.z = values(v_idx + 2);
  } else {
    twist.linear.x = values(v_idx);
    twist.linear.y = values(v_idx + 1);
    twist.linear.z = values(v_idx + 2);
    twist.angular.x = values(v_idx + 3);
    twist.angular.y = values(v_idx + 4);
    twist.angular.z = values(v_idx + 5);
  }
  return twist;
}

/// @brief Inverse of toTwist: writes a twist's values for a planar or floating joint.
void fromTwist(roboplan::JointType type, const geometry_msgs::msg::Twist& twist,
               Eigen::VectorXd& values, size_t v_idx) {
  if (type == roboplan::JointType::PLANAR) {
    values.segment<3>(v_idx) << twist.linear.x, twist.linear.y, twist.angular.z;
  } else {
    values.segment<6>(v_idx) << twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x,
        twist.angular.y, twist.angular.z;
  }
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

  // The joints of both messages, with each joint's index in its own message.
  std::vector<TrajectoryJoint> joints;
  std::vector<size_t> message_indices;
  for (const bool multi_dof : {false, true}) {
    const auto& names = multi_dof ? multi_dof_trajectory.joint_names : ros_trajectory.joint_names;
    const auto message_joints = trajectoryJoints(scene, names);
    if (!message_joints) {
      return error(message_joints.error());
    }
    for (size_t j = 0; j < message_joints->size(); ++j) {
      const auto& joint = message_joints->at(j);
      if (joint.isMultiDof() && !multi_dof) {
        return error("Joint '" + joint.name +
                     "' has several degrees of freedom: a JointTrajectory cannot represent it, "
                     "use a MultiDOFJointTrajectory");
      }
      if (!joint.isMultiDof() && multi_dof) {
        return error("Joint '" + joint.name +
                     "' has a single degree of freedom: it belongs in the JointTrajectory");
      }
      joints.push_back(joint);
      message_indices.push_back(j);
    }
  }
  std::unordered_set<std::string> seen;
  for (const auto& joint : joints) {
    if (!seen.insert(joint.name).second) {
      return error("Joint '" + joint.name + "' appears twice");
    }
  }
  const auto size = layOutJoints(joints);

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

  roboplan::JointTrajectory joint_traj;
  for (const auto& joint : joints) {
    joint_traj.joint_names.push_back(joint.name);
  }
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
    std::vector<PointField> fields;
    if (point) {
      fields.push_back({"positions", point->positions.size(), num_single, false});
      fields.push_back({"velocities", point->velocities.size(), num_single, true});
      fields.push_back({"accelerations", point->accelerations.size(), num_single, true});
    }
    if (multi_dof_point) {
      fields.push_back({"transforms", multi_dof_point->transforms.size(), num_multi_dof, false});
      fields.push_back(
          {"velocity twists", multi_dof_point->velocities.size(), num_multi_dof, true});
      fields.push_back(
          {"acceleration twists", multi_dof_point->accelerations.size(), num_multi_dof, true});
    }
    if (auto ok = checkPointSizes(i, fields); !ok) {
      return tl::make_unexpected(ok.error());
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

    Eigen::VectorXd positions(size.nq);
    Eigen::VectorXd velocities(*has_velocities ? size.nv : 0);
    Eigen::VectorXd accelerations(*has_accelerations ? size.nv : 0);
    for (size_t k = 0; k < joints.size(); ++k) {
      const auto& joint = joints[k];
      const auto j = message_indices[k];
      const auto q_idx = joint.q_start;
      const auto v_idx = joint.v_start;
      if (joint.isMimic()) {
        // Mimic joints have no values of their own: RoboPlan derives them.
        continue;
      }
      switch (joint.info.type) {
      case roboplan::JointType::REVOLUTE:
      case roboplan::JointType::PRISMATIC:
      case roboplan::JointType::CONTINUOUS:
        if (joint.info.type == roboplan::JointType::CONTINUOUS) {
          positions(q_idx) = std::cos(point->positions[j]);
          positions(q_idx + 1) = std::sin(point->positions[j]);
        } else {
          positions(q_idx) = point->positions[j];
        }
        if (velocities.size() > 0) {
          velocities(v_idx) = point->velocities[j];
        }
        if (accelerations.size() > 0) {
          accelerations(v_idx) = point->accelerations[j];
        }
        break;
      case roboplan::JointType::PLANAR:
      case roboplan::JointType::FLOATING: {
        const auto& transform = multi_dof_point->transforms[j];
        Eigen::Quaterniond rotation;
        tf2::fromMsg(transform.rotation, rotation);
        if (rotation.norm() < 1e-9) {
          return error("Point " + std::to_string(i) + " has a zero quaternion for joint '" +
                       joint.name + "'");
        }
        rotation.normalize();
        if (joint.info.type == roboplan::JointType::PLANAR) {
          // Only the rotation about z: the direction of the x axis in the xy plane.
          const Eigen::Matrix3d matrix = rotation.toRotationMatrix();
          const double norm = std::hypot(matrix(0, 0), matrix(1, 0));
          if (norm < 1e-9) {
            return error("Point " + std::to_string(i) + " has no rotation about z for joint '" +
                         joint.name + "'");
          }
          positions.segment<4>(q_idx) << transform.translation.x, transform.translation.y,
              matrix(0, 0) / norm, matrix(1, 0) / norm;
        } else {
          positions.segment<7>(q_idx) << transform.translation.x, transform.translation.y,
              transform.translation.z, rotation.x(), rotation.y(), rotation.z(), rotation.w();
        }
        if (velocities.size() > 0) {
          fromTwist(joint.info.type, multi_dof_point->velocities[j], velocities, v_idx);
        }
        if (accelerations.size() > 0) {
          fromTwist(joint.info.type, multi_dof_point->accelerations[j], accelerations, v_idx);
        }
        break;
      }
      default:
        return error("Joint '" + joint.name + "' has an unknown type");
      }
    }
    joint_traj.times.push_back(
        fromDuration(point ? point->time_from_start : multi_dof_point->time_from_start));
    joint_traj.positions.push_back(positions);
    joint_traj.velocities.push_back(velocities);
    joint_traj.accelerations.push_back(accelerations);
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

trajectory_msgs::msg::JointTrajectory
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory) {

  trajectory_msgs::msg::JointTrajectory ros_traj;
  ros_traj.joint_names = roboplan_trajectory.joint_names;

  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < roboplan_trajectory.times.size(); ++i) {
    auto& point = ros_traj.points[i];
    double time_sec = roboplan_trajectory.times[i];
    point.time_from_start = toDuration(time_sec);

    if (i < roboplan_trajectory.positions.size()) {
      point.positions.resize(roboplan_trajectory.positions[i].size());
      for (int j = 0; j < roboplan_trajectory.positions[i].size(); ++j) {
        point.positions[j] = roboplan_trajectory.positions[i][j];
      }
    }

    if (i < roboplan_trajectory.velocities.size()) {
      point.velocities.resize(roboplan_trajectory.velocities[i].size());
      for (int j = 0; j < roboplan_trajectory.velocities[i].size(); ++j) {
        point.velocities[j] = roboplan_trajectory.velocities[i][j];
      }
    }

    if (i < roboplan_trajectory.accelerations.size()) {
      point.accelerations.resize(roboplan_trajectory.accelerations[i].size());
      for (int j = 0; j < roboplan_trajectory.accelerations[i].size(); ++j) {
        point.accelerations[j] = roboplan_trajectory.accelerations[i][j];
      }
    }
  }

  return ros_traj;
}

roboplan::JointTrajectory
fromJointTrajectory(const trajectory_msgs::msg::JointTrajectory& ros_trajectory) {

  roboplan::JointTrajectory joint_traj;

  joint_traj.joint_names = ros_trajectory.joint_names;

  joint_traj.times.reserve(ros_trajectory.points.size());
  joint_traj.positions.reserve(ros_trajectory.points.size());
  joint_traj.velocities.reserve(ros_trajectory.points.size());
  joint_traj.accelerations.reserve(ros_trajectory.points.size());

  for (const auto& point : ros_trajectory.points) {
    joint_traj.times.push_back(fromDuration(point.time_from_start));

    Eigen::VectorXd positions =
        Eigen::VectorXd::Map(point.positions.data(), point.positions.size());
    joint_traj.positions.push_back(positions);

    Eigen::VectorXd velocities =
        Eigen::VectorXd::Map(point.velocities.data(), point.velocities.size());
    joint_traj.velocities.push_back(velocities);

    Eigen::VectorXd accelerations =
        Eigen::VectorXd::Map(point.accelerations.data(), point.accelerations.size());
    joint_traj.accelerations.push_back(accelerations);
  }

  return joint_traj;
}

tl::expected<trajectory_msgs::msg::JointTrajectory, std::string>
toJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                  const roboplan::Scene& scene) {
  auto joints = trajectoryJoints(scene, roboplan_trajectory.joint_names);
  if (!joints) {
    return tl::make_unexpected("Failed to convert the trajectory: " + joints.error());
  }
  const auto size = layOutJoints(joints.value());

  // Mimic joints have no values of their own, planar and floating joints go in a
  // MultiDOFJointTrajectory.
  trajectory_msgs::msg::JointTrajectory ros_traj;
  ros_traj.joint_names = jointNames(joints.value(), &TrajectoryJoint::isSingleDof);

  std::vector<std::optional<double>> previous_angles(joints->size());
  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < ros_traj.points.size(); ++i) {
    if (auto ok = checkRoboPlanPoint(roboplan_trajectory, i, size); !ok) {
      return tl::make_unexpected(ok.error());
    }
    auto& point = ros_traj.points[i];
    point.time_from_start = toDuration(roboplan_trajectory.times[i]);
    const auto* positions = pointValues(roboplan_trajectory.positions, i);
    const auto* velocities = pointValues(roboplan_trajectory.velocities, i);
    const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i);
    for (size_t j = 0; j < joints->size(); ++j) {
      const auto& joint = joints->at(j);
      const auto q_idx = joint.q_start;
      const auto v_idx = joint.v_start;
      if (joint.isSingleDof()) {
        if (positions && joint.info.type == roboplan::JointType::CONTINUOUS) {
          const auto angle = unwrapAngle(std::atan2((*positions)(q_idx + 1), (*positions)(q_idx)),
                                         previous_angles[j]);
          previous_angles[j] = angle;
          point.positions.push_back(angle);
        } else if (positions) {
          point.positions.push_back((*positions)(q_idx));
        }
        if (velocities) {
          point.velocities.push_back((*velocities)(v_idx));
        }
        if (accelerations) {
          point.accelerations.push_back((*accelerations)(v_idx));
        }
      }
    }
  }
  return ros_traj;
}

tl::expected<trajectory_msgs::msg::MultiDOFJointTrajectory, std::string>
toMultiDOFJointTrajectory(const roboplan::JointTrajectory& roboplan_trajectory,
                          const roboplan::Scene& scene) {
  auto joints = trajectoryJoints(scene, roboplan_trajectory.joint_names);
  if (!joints) {
    return tl::make_unexpected("Failed to convert the trajectory: " + joints.error());
  }
  const auto size = layOutJoints(joints.value());

  trajectory_msgs::msg::MultiDOFJointTrajectory ros_traj;
  ros_traj.joint_names = jointNames(joints.value(), &TrajectoryJoint::isMultiDof);
  if (ros_traj.joint_names.empty()) {
    return ros_traj;
  }

  std::vector<std::optional<Eigen::Quaterniond>> previous_rotations(joints->size());
  ros_traj.points.resize(roboplan_trajectory.times.size());
  for (size_t i = 0; i < ros_traj.points.size(); ++i) {
    if (auto ok = checkRoboPlanPoint(roboplan_trajectory, i, size); !ok) {
      return tl::make_unexpected(ok.error());
    }
    auto& point = ros_traj.points[i];
    point.time_from_start = toDuration(roboplan_trajectory.times[i]);
    const auto* positions = pointValues(roboplan_trajectory.positions, i);
    const auto* velocities = pointValues(roboplan_trajectory.velocities, i);
    const auto* accelerations = pointValues(roboplan_trajectory.accelerations, i);
    for (size_t j = 0; j < joints->size(); ++j) {
      const auto& joint = joints->at(j);
      const auto q_idx = joint.q_start;
      const auto v_idx = joint.v_start;
      if (joint.isMultiDof()) {
        if (positions) {
          const auto& q = *positions;
          geometry_msgs::msg::Transform transform;
          Eigen::Quaterniond rotation;
          if (joint.info.type == roboplan::JointType::PLANAR) {
            transform.translation.x = q(q_idx);
            transform.translation.y = q(q_idx + 1);
            const auto angle = std::atan2(q(q_idx + 3), q(q_idx + 2));
            rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
          } else {
            transform.translation.x = q(q_idx);
            transform.translation.y = q(q_idx + 1);
            transform.translation.z = q(q_idx + 2);
            rotation = Eigen::Quaterniond(q(q_idx + 6), q(q_idx + 3), q(q_idx + 4), q(q_idx + 5));
          }
          // q and -q are the same rotation: keep the sign closest to the previous point, so that
          // interpolating between consecutive points takes the short way.
          if (previous_rotations[j] && previous_rotations[j]->dot(rotation) < 0.0) {
            rotation.coeffs() *= -1.0;
          }
          previous_rotations[j] = rotation;
          transform.rotation = tf2::toMsg(rotation);
          point.transforms.push_back(transform);
        }
        if (velocities) {
          point.velocities.push_back(toTwist(joint.info.type, *velocities, v_idx));
        }
        if (accelerations) {
          point.accelerations.push_back(toTwist(joint.info.type, *accelerations, v_idx));
        }
      }
    }
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
