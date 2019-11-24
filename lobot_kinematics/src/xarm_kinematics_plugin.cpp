#include "xarm_kinematics_plugin/xarm_kinematics_plugin.h"

#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_list_macros.h>

namespace xarm_kinematics_plugin
{
XarmKinematicsPlugin::XarmKinematicsPlugin()
{
  joint_names_.reserve(JOINT_NUM);
  link_names_.reserve(JOINT_NUM);
  lower_limits_.reserve(JOINT_NUM);
  upper_limits_.reserve(JOINT_NUM);
}

XarmKinematicsPlugin::~XarmKinematicsPlugin()
{
}

bool XarmKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                         const std::vector<double>& joint_angles,
                                         std::vector<geometry_msgs::Pose>& poses) const
{
  return false;
}

bool XarmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                         std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                         const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits, solution, solution_callback,
                          error_code, options);
}

bool XarmKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                      const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                      double search_discretization)
{
  setValues(robot_model.getName(), group_name, base_frame, tip_frames, search_discretization);

  auto link_models = robot_model.getLinkModels();
  for (const auto& link : link_models)
  {
    if (link->getName() == tip_frames_.at(0))
    {
      break;
    }

    link_names_.push_back(link->getName());

    auto joint = link->getParentJointModel();
    if (!joint || joint->getType() == moveit::core::JointModel::JointType::UNKNOWN ||
        joint->getType() == moveit::core::JointModel::JointType::FIXED)
    {
      continue;
    }

    joint_names_.push_back(joint->getName());

    if (joint->getType() == moveit::core::JointModel::JointType::REVOLUTE ||
        joint->getType() == moveit::core::JointModel::JointType::PRISMATIC)
    {
      auto bounds = joint->getVariableBounds();
      lower_limits_.push_back(bounds.at(0).min_position_);
      upper_limits_.push_back(bounds.at(0).max_position_);
    }
    else
    {
      lower_limits_.push_back(-M_PI);
      upper_limits_.push_back(M_PI);
    }
  }

  return true;
}

bool XarmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool XarmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool XarmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool XarmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state, double timeout,
                                            const std::vector<double>& consistency_limits,
                                            std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                            moveit_msgs::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  // auto pose = ik_pose;  // Make a copy of the pose
  // if (!isPoseReachable(pose) && !revisePose(pose))
  // {
  //   ROS_ERROR_NAMED("xarm_kinematics_plugin", "The target pose is not reachable");
  //   return false;
  // }

  // tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  // tf::Matrix3x3 rotation_matrix(q);
  if (!solveIk(ik_pose, solution))
  {
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}

bool XarmKinematicsPlugin::isPoseReachable(const geometry_msgs::Pose& pose) const
{
  if (pose.position.z < 0)
  {
    return false;
  }

  double roll, pitch, yaw;
  quaternionToRpy(pose.orientation, roll, pitch, yaw);

  if (pose.position.x == 0 && abs(sin(yaw)) > 0.99)
  {
    return true;
  }

  constexpr double threshold = 0.01;
  auto y = pose.position.x * tan(yaw);
  if (y - threshold <= pose.position.y && pose.position.y <= y + threshold)
  {
    return true;
  }

  return false;
}

void XarmKinematicsPlugin::quaternionToRpy(const geometry_msgs::Quaternion& q, double& roll, double& pitch,
                                           double& yaw) const
{  // Roll, X axis
  roll = atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
  // Pitch, Y axis
  pitch = asin(-2 * (q.x * q.z - q.w * q.y));
  // Yaw, Z axis
  yaw = atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

bool XarmKinematicsPlugin::revisePose(geometry_msgs::Pose& pose) const
{
  double roll, pitch, yaw;
  quaternionToRpy(pose.orientation, roll, pitch, yaw);

  if (pose.position.x != 0)
  {
    yaw = atan2(pose.position.y, pose.position.x);
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  }
  else if (pose.position.y > 0)
  {
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, M_PI / 2);
  }
  else if (pose.position.y < 0)
  {
    pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, -M_PI / 2);
  }

  return isPoseReachable(pose);
}

bool XarmKinematicsPlugin::solveIk(const geometry_msgs::Pose& ik_pose, std::vector<double>& solution) const
{
  auto p = ik_pose.position;

  double roll, pitch, yaw;
  quaternionToRpy(ik_pose.orientation, roll, pitch, yaw);

  // Convert quaternion to rotation matrix
  tf::Matrix3x3 r;
  yaw = atan2(p.y, p.x);
  r.setRPY(roll, pitch, yaw);

  // Simplify the rotation matrix
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      if (abs(r[i][j]) < FLT_EPSILON)
      {
        r[i][j] = 0;
      }
    }
  }

  // Parameters
  constexpr double a1 = 0.003;
  constexpr double a2 = 0.096;
  constexpr double a3 = 0.096;
  constexpr double baseHeight = 0.072;  // Height of base relative to world
  constexpr double toolLength = 0.12;   // Length of terminal tool

  // Transformation from the tool frame to the last frame on the manipulator
  double nx = r[0][2], ny = r[1][2], nz = r[2][2];
  double ox = -r[0][1], oy = -r[1][1], oz = -r[2][1];
  double ax = r[0][0], ay = r[1][0], az = r[2][0];
  double px = p.x - toolLength * r[0][0];
  double py = p.y - toolLength * r[1][0];
  double pz = p.z - toolLength * r[2][0] - baseHeight;

  std::complex<double> theta3;
  const double theta3_numerator = 2 * a1 * a1 * a2 * a2 - a2 * a2 * a2 * a2 - a3 * a3 * a3 * a3 - px * px * px * px -
                                  py * py * py * py - pz * pz * pz * pz - a1 * a1 * a1 * a1 + 2 * a1 * a1 * a3 * a3 +
                                  2 * a2 * a2 * a3 * a3 + 2 * a1 * a1 * px * px + 2 * a2 * a2 * px * px +
                                  2 * a3 * a3 * px * px + 2 * a1 * a1 * py * py + 2 * a2 * a2 * py * py +
                                  2 * a3 * a3 * py * py - 2 * a1 * a1 * pz * pz + 2 * a2 * a2 * pz * pz +
                                  2 * a3 * a3 * pz * pz - 2 * px * px * py * py - 2 * px * px * pz * pz -
                                  2 * py * py * pz * pz + 8 * a1 * a2 * a3 * sqrt(px * px + py * py);
  if (theta3_numerator < 0)
  {
    r.setRPY(roll, pitch, 0);
    r *= tf::Matrix3x3(0, 0, 1, 0, -1, 0, 1, 0, 0);
    nx = r[0][2], ny = r[1][2], nz = r[2][2];
    ox = -r[0][1], oy = -r[1][1], oz = -r[2][1];
    ax = r[0][0], ay = r[1][0], az = r[2][0];
    px = p.x - toolLength * r[0][0];
    py = p.y - toolLength * r[1][0];
    pz = p.z - toolLength * r[2][0] - baseHeight;
  }
  if (p.y >= 0)
  {
    theta3 =
        2 * atan(sqrt((2 * a1 * a1 * a2 * a2 - a2 * a2 * a2 * a2 - a3 * a3 * a3 * a3 - px * px * px * px -
                       py * py * py * py - pz * pz * pz * pz - a1 * a1 * a1 * a1 + 2 * a1 * a1 * a3 * a3 +
                       2 * a2 * a2 * a3 * a3 + 2 * a1 * a1 * px * px + 2 * a2 * a2 * px * px + 2 * a3 * a3 * px * px +
                       2 * a1 * a1 * py * py + 2 * a2 * a2 * py * py + 2 * a3 * a3 * py * py - 2 * a1 * a1 * pz * pz +
                       2 * a2 * a2 * pz * pz + 2 * a3 * a3 * pz * pz - 2 * px * px * py * py - 2 * px * px * pz * pz -
                       2 * py * py * pz * pz + 8 * a1 * a2 * a3 * sqrt(px * px + py * py)) /
                      ((px * px + py * py) *
                           (-2 * a1 * a1 - 2 * a2 * a2 + 4 * a2 * a3 - 2 * a3 * a3 + px * px + py * py + 2 * pz * pz) -
                       4 * a2 * a3 * a3 * a3 - 4 * a2 * a2 * a2 * a3 + a1 * a1 * a1 * a1 + a2 * a2 * a2 * a2 +
                       a3 * a3 * a3 * a3 + pz * pz * pz * pz - 2 * a1 * a1 * a2 * a2 - 2 * a1 * a1 * a3 * a3 +
                       6 * a2 * a2 * a3 * a3 + 2 * a1 * a1 * pz * pz - 2 * a2 * a2 * pz * pz - 2 * a3 * a3 * pz * pz +
                       4 * a1 * a1 * a2 * a3 + 4 * a2 * a3 * pz * pz)));
  }
  else
  {
    theta3 =
        -2 * atan(sqrt(-(a1 * a1 * a1 * a1 + a2 * a2 * a2 * a2 + a3 * a3 * a3 * a3 + px * px * px * px +
                         py * py * py * py + pz * pz * pz * pz - 2 * a1 * a1 * a2 * a2 - 2 * a1 * a1 * a3 * a3 -
                         2 * a2 * a2 * a3 * a3 - 2 * a1 * a1 * px * px - 2 * a2 * a2 * px * px - 2 * a3 * a3 * px * px -
                         2 * a1 * a1 * py * py - 2 * a2 * a2 * py * py - 2 * a3 * a3 * py * py + 2 * a1 * a1 * pz * pz -
                         2 * a2 * a2 * pz * pz - 2 * a3 * a3 * pz * pz + 2 * px * px * py * py + 2 * px * px * pz * pz +
                         2 * py * py * pz * pz + 8 * a1 * a2 * a3 * sqrt(px * px + py * py)) /
                       ((px * px + py * py) *
                            (-2 * a1 * a1 - 2 * a2 * a2 + 4 * a2 * a3 - 2 * a3 * a3 + px * px + py * py + 2 * pz * pz) -
                        4 * a2 * a3 * a3 * a3 - 4 * a2 * a2 * a2 * a3 + a1 * a1 * a1 * a1 + a2 * a2 * a2 * a2 +
                        a3 * a3 * a3 * a3 + pz * pz * pz * pz - 2 * a1 * a1 * a2 * a2 - 2 * a1 * a1 * a3 * a3 +
                        6 * a2 * a2 * a3 * a3 + 2 * a1 * a1 * pz * pz - 2 * a2 * a2 * pz * pz - 2 * a3 * a3 * pz * pz +
                        4 * a1 * a1 * a2 * a3 + 4 * a2 * a3 * pz * pz)));
  }
  double t3 = real(theta3);
  double s3 = sin(t3);
  double c3 = cos(t3);

  std::complex<double> theta2 =
      (pz > 0) ?
          (-2 *
           atan((sqrt(-a1 * a1 * a1 * a1 + 2 * a1 * a1 * a2 * a2 + 4 * a1 * a1 * a2 * a3 * c3 +
                      2 * a1 * a1 * a3 * a3 * c3 * c3 + 2 * a1 * a1 * a3 * a3 * s3 * s3 + 2 * a1 * a1 * px * px +
                      2 * a1 * a1 * py * py + 2 * a1 * a1 * pz * pz - a2 * a2 * a2 * a2 - 4 * a2 * a2 * a2 * a3 * c3 -
                      6 * a2 * a2 * a3 * a3 * c3 * c3 - 2 * a2 * a2 * a3 * a3 * s3 * s3 + 2 * a2 * a2 * px * px +
                      2 * a2 * a2 * py * py + 2 * a2 * a2 * pz * pz - 4 * a2 * a3 * a3 * a3 * c3 * c3 * c3 -
                      4 * a2 * a3 * a3 * a3 * c3 * s3 * s3 + 4 * a2 * a3 * c3 * px * px + 4 * a2 * a3 * c3 * py * py +
                      4 * a2 * a3 * c3 * pz * pz - a3 * a3 * a3 * a3 * c3 * c3 * c3 * c3 -
                      2 * a3 * a3 * a3 * a3 * c3 * c3 * s3 * s3 - a3 * a3 * a3 * a3 * s3 * s3 * s3 * s3 +
                      2 * a3 * a3 * c3 * c3 * px * px + 2 * a3 * a3 * c3 * c3 * py * py +
                      2 * a3 * a3 * c3 * c3 * pz * pz + 2 * a3 * a3 * px * px * s3 * s3 +
                      2 * a3 * a3 * py * py * s3 * s3 + 2 * a3 * a3 * pz * pz * s3 * s3 - px * px * px * px -
                      2 * px * px * py * py - 2 * px * px * pz * pz - py * py * py * py - 2 * py * py * pz * pz -
                      pz * pz * pz * pz) +
                 2 * a1 * a3 * s3) /
                (-a1 * a1 + 2 * a1 * a2 + 2 * a1 * a3 * c3 - a2 * a2 - 2 * a2 * a3 * c3 - a3 * a3 * c3 * c3 -
                 a3 * a3 * s3 * s3 + px * px + py * py + pz * pz))) :
          (2 *
           atan((sqrt(-a1 * a1 * a1 * a1 + 2 * a1 * a1 * a2 * a2 + 4 * a1 * a1 * a2 * a3 * c3 +
                      2 * a1 * a1 * a3 * a3 * c3 * c3 + 2 * a1 * a1 * a3 * a3 * s3 * s3 + 2 * a1 * a1 * px * px +
                      2 * a1 * a1 * py * py + 2 * a1 * a1 * pz * pz - a2 * a2 * a2 * a2 - 4 * a2 * a2 * a2 * a3 * c3 -
                      6 * a2 * a2 * a3 * a3 * c3 * c3 - 2 * a2 * a2 * a3 * a3 * s3 * s3 + 2 * a2 * a2 * px * px +
                      2 * a2 * a2 * py * py + 2 * a2 * a2 * pz * pz - 4 * a2 * a3 * a3 * a3 * c3 * c3 * c3 -
                      4 * a2 * a3 * a3 * a3 * c3 * s3 * s3 + 4 * a2 * a3 * c3 * px * px + 4 * a2 * a3 * c3 * py * py +
                      4 * a2 * a3 * c3 * pz * pz - a3 * a3 * a3 * a3 * c3 * c3 * c3 * c3 -
                      2 * a3 * a3 * a3 * a3 * c3 * c3 * s3 * s3 - a3 * a3 * a3 * a3 * s3 * s3 * s3 * s3 +
                      2 * a3 * a3 * c3 * c3 * px * px + 2 * a3 * a3 * c3 * c3 * py * py +
                      2 * a3 * a3 * c3 * c3 * pz * pz + 2 * a3 * a3 * px * px * s3 * s3 +
                      2 * a3 * a3 * py * py * s3 * s3 + 2 * a3 * a3 * pz * pz * s3 * s3 - px * px * px * px -
                      2 * px * px * py * py - 2 * px * px * pz * pz - py * py * py * py - 2 * py * py * pz * pz -
                      pz * pz * pz * pz) -
                 2 * a1 * a3 * s3) /
                (-a1 * a1 + 2 * a1 * a2 + 2 * a1 * a3 * c3 - a2 * a2 - 2 * a2 * a3 * c3 - a3 * a3 * c3 * c3 -
                 a3 * a3 * s3 * s3 + px * px + py * py + pz * pz)));
  double t2 = real(theta2);
  double s2 = sin(t2);
  double c2 = cos(t2);

  std::complex<double> theta1;
  double theta1_numerator = a1 - px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3;
  if (abs(theta1_numerator) < FLT_EPSILON)
  {
    theta1 = 0;
  }
  else if (p.x == 0 && p.y == 0 && roll < 0)
  {
    theta1 = -2 * atan(sqrt(theta1_numerator / (a1 + px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3)));
  }
  else
  {
    theta1 = 2 * atan(sqrt(theta1_numerator / (a1 + px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3)));
  }
  double t1 = real(theta1);
  double s1 = sin(t1);
  double c1 = cos(t1);

  double t4 = atan2(
      (az * (c2 * s3 + c3 * s2)) / (c2 * c2 * c3 * c3 + c2 * c2 * s3 * s3 + c3 * c3 * s2 * s2 + s2 * s2 * s3 * s3) -
          (ax * (c1 * c2 * c3 - c1 * s2 * s3)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 + c1 * c1 * c3 * c3 * s2 * s2 +
               c1 * c1 * s2 * s2 * s3 * s3 + c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3) -
          (ay * (c2 * c3 * s1 - s1 * s2 * s3)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 + c1 * c1 * c3 * c3 * s2 * s2 +
               c1 * c1 * s2 * s2 * s3 * s3 + c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3),
      -(ax * (c1 * c2 * s3 + c1 * c3 * s2)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 + c1 * c1 * c3 * c3 * s2 * s2 +
               c1 * c1 * s2 * s2 * s3 * s3 + c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3) -
          (az * (c2 * c3 - s2 * s3)) / (c2 * c2 * c3 * c3 + c2 * c2 * s3 * s3 + c3 * c3 * s2 * s2 + s2 * s2 * s3 * s3) -
          (ay * s1 * (c2 * s3 + c3 * s2)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 + c1 * c1 * c3 * c3 * s2 * s2 +
               c1 * c1 * s2 * s2 * s3 * s3 + c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3));
  double s4 = sin(t4);
  double c4 = cos(t4);

  double t5 = atan2(((oz * (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4)) /
                         (c2 * c2 * c3 * c3 * c4 * c4 + c2 * c2 * c3 * c3 * s4 * s4 + c2 * c2 * c4 * c4 * s3 * s3 +
                          c2 * c2 * s3 * s3 * s4 * s4 + c3 * c3 * c4 * c4 * s2 * s2 + c3 * c3 * s2 * s2 * s4 * s4 +
                          c4 * c4 * s2 * s2 * s3 * s3 + s2 * s2 * s3 * s3 * s4 * s4) +
                     (oy * (c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4 + c4 * s1 * s2 * s3 - c2 * c3 * c4 * s1)) /
                         (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 + c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
                          c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 + c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
                          c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 + c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
                          c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 + c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
                          c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 + c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
                          c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 + c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
                          c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 + c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
                          c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 + s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4) +
                     (c1 * ox * (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3)) /
                         (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 + c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
                          c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 + c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
                          c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 + c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
                          c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 + c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
                          c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 + c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
                          c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 + c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
                          c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 + c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
                          c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 + s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4)),
                    (-(nz * (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4)) /
                         (c2 * c2 * c3 * c3 * c4 * c4 + c2 * c2 * c3 * c3 * s4 * s4 + c2 * c2 * c4 * c4 * s3 * s3 +
                          c2 * c2 * s3 * s3 * s4 * s4 + c3 * c3 * c4 * c4 * s2 * s2 + c3 * c3 * s2 * s2 * s4 * s4 +
                          c4 * c4 * s2 * s2 * s3 * s3 + s2 * s2 * s3 * s3 * s4 * s4) -
                     (ny * (c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4 + c4 * s1 * s2 * s3 - c2 * c3 * c4 * s1)) /
                         (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 + c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
                          c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 + c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
                          c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 + c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
                          c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 + c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
                          c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 + c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
                          c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 + c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
                          c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 + c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
                          c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 + s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4) -
                     (c1 * nx * (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3)) /
                         (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 + c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
                          c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 + c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
                          c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 + c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
                          c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 + c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
                          c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 + c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
                          c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 + c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
                          c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 + c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
                          c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 + s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4)));

  solution.resize(JOINT_NUM);
  solution[0] = t1;             // arm_joint1
  solution[1] = t2 + M_PI / 2;  // arm_joint2
  solution[2] = t3;             // arm_joint3
  solution[3] = t4 + M_PI / 2;  // arm_joint4
  solution[4] = asin(sin(t5));  // arm_joint5

  // Check the solution and joints' limits
  for (auto i = 0; i < JOINT_NUM; ++i)
  {
    if (std::isnan(solution[i]))
    {
      ROS_ERROR_NAMED("xarm_kinematics_plugin", "The value of arm_joint%d is not a number", i + 1);
      return false;
    }
    else if (solution[i] < lower_limits_[i] || solution[i] > upper_limits_[i])
    {
      ROS_ERROR_NAMED("xarm_kinematics_plugin", "The value of arm_joint%d (%f) is out of bounds", i + 1, solution[i]);
      // return false;
    }
  }

  return true;
}

}  // namespace xarm_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(xarm_kinematics_plugin::XarmKinematicsPlugin, kinematics::KinematicsBase);
