#include "xarm_kinematics_plugin/xarm_kinematics_plugin.h"

#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>

namespace xarm_kinematics_plugin {

XArmKinematicsPlugin::XArmKinematicsPlugin() {
  jointNames_.reserve(JOINT_NUM);
  linkNames_.reserve(JOINT_NUM);
  lowerLimits_.reserve(JOINT_NUM);
  upperLimits_.reserve(JOINT_NUM);
}

XArmKinematicsPlugin::~XArmKinematicsPlugin() {}

bool XArmKinematicsPlugin::getPositionFK(
    const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::Pose> &poses) const {
  return false;
}

bool XArmKinematicsPlugin::getPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_,
                          consistency_limits, solution, solution_callback,
                          error_code, options);
}

bool XArmKinematicsPlugin::initialize(const std::string &robot_description,
                                      const std::string &group_name,
                                      const std::string &base_frame,
                                      const std::string &tip_frame,
                                      double search_discretization) {
  urdf::Model robotModel;
  robotModel.initParam(robot_description);

  auto link = robotModel.getLink(tip_frame);
  if (!link) {
    return false;
  }

  while (link && link->name != base_frame) {
    linkNames_.push_back(link->name);
    auto joint = link->parent_joint;
    if (!joint || joint->type == urdf::Joint::UNKNOWN) {
      link = link->getParent();
      continue;
    }

    jointNames_.push_back(joint->name);
    if (joint->type != urdf::Joint::CONTINUOUS) {
      if (joint->safety) {
        lowerLimits_.push_back(joint->safety->soft_lower_limit);
        upperLimits_.push_back(joint->safety->soft_upper_limit);
      } else {
        lowerLimits_.push_back(joint->limits->lower);
        upperLimits_.push_back(joint->limits->upper);
      }
    } else {
      lowerLimits_.push_back(-M_PI);
      upperLimits_.push_back(M_PI);
    }

    link = link->getParent();
  }

  return true;
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                          solution, solution_callback, error_code, options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  const IKCallbackFn solution_callback = 0;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                          solution, solution_callback, error_code, options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution, const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
                          solution, solution_callback, error_code, options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state, double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution, const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const {
  return true;
}

}  // namespace xarm_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(xarm_kinematics_plugin::XArmKinematicsPlugin,
                       kinematics::KinematicsBase);
