#include "xarm_kinematics_plugin/xarm_kinematics_plugin.h"

namespace xarm_kinematics_plugin {

bool XArmKinematicsPlugin::getPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state, std::vector<double>& solution,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;

  // Limit search to a single attempt by setting a timeout of zero
  return searchPositionIK(ik_pose, ik_seed_state, 0.0, solution, IKCallbackFn(),
                          error_code, consistency_limits, options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state, double timeout,
    std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution,
                          IKCallbackFn(), error_code, consistency_limits,
                          options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const {
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution,
                          IKCallbackFn(), error_code, consistency_limits,
                          options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state, double timeout,
    std::vector<double>& solution, const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const {
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution,
                          solution_callback, error_code, consistency_limits,
                          options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state, double timeout,
    const std::vector<double>& consistency_limits,
    std::vector<double>& solution, const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code,
    const kinematics::KinematicsQueryOptions& options) const {
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution,
                          solution_callback, error_code, consistency_limits,
                          options);
}

bool XArmKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose& ik_pose,
    const std::vector<double>& ik_seed_state, double timeout,
    std::vector<double>& solution, const IKCallbackFn& solution_callback,
    moveit_msgs::MoveItErrorCodes& error_code,
    const std::vector<double>& consistency_limits,
    const kinematics::KinematicsQueryOptions& options) const {
  ros::WallTime start_time = ros::WallTime::now();
  if (!initialized_) {
    ROS_ERROR_NAMED("xarm_kinematics_plugin",
                    "kinematics solver not initialized");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != dimension_) {
    ROS_ERROR_STREAM_NAMED("xarm_kinematics_plugin",
                           "Seed state must have size "
                               << dimension_ << " instead of size "
                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  geometry_msgs::Pose pose = ik_pose;
  if (!isPoseReachable(pose) && !revisePose(pose)) {
    return false;
  }

  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                   pose.orientation.w);
  tf::Matrix3x3 rotateMatrix(q);
  if (!solveIK(pose.position, rotateMatrix, solution)) {
    return false;
  }
}

bool XArmKinematicsPlugin::getPositionFK(
    const std::vector<std::string>& link_names,
    const std::vector<double>& joint_angles,
    std::vector<geometry_msgs::Pose>& poses) const {}

bool XArmKinematicsPlugin::initialize(
    const moveit::core::RobotModel& robot_model, const std::string& group_name,
    const std::string& base_frame, const std::vector<std::string>& tip_frames,
    double search_discretization) {}

const std::vector<std::string>& XArmKinematicsPlugin::getJointNames() const {}

const std::vector<std::string>& XArmKinematicsPlugin::getLinkNames() const {}

bool XArmKinematicsPlugin::solveIK(const geometry_msgs::Point& p,
                                   const tf::Matrix3x3& r,
                                   std::vector<double>& solution) const {
  constexpr double a1 = 0.003;
  constexpr double a2 = 0.096;
  constexpr double a3 = 0.096;
  constexpr double baseHeight = 0.072;  // Height of base relative to world
  constexpr double toolLength = 0.12;   // Length of terminal tool

  const double nx = r[0][2], ny = r[1][2], nz = r[2][2];
  const double ox = -r[0][1], oy = -r[1][1], oz = -r[2][1];
  const double ax = r[0][0], ay = r[1][0], az = r[2][0];
  const double px = p.x - toolLength * r[0][0];
  const double py = p.y - toolLength * r[1][0];
  const double pz = p.z - toolLength * r[2][0] - baseHeight;

  std::complex<double> theta3 =
      2 * atan(sqrt(
              (2 * pow(a1, 2) * pow(a2, 2) - pow(a2, 4) - pow(a3, 4) -
               pow(px, 4) - pow(py, 4) - pow(pz, 4) - pow(a1, 4) +
               2 * pow(a1, 2) * pow(a3, 2) + 2 * pow(a2, 2) * pow(a3, 2) +
               2 * pow(a1, 2) * pow(px, 2) + 2 * pow(a2, 2) * pow(px, 2) +
               2 * pow(a3, 2) * pow(px, 2) + 2 * pow(a1, 2) * pow(py, 2) +
               2 * pow(a2, 2) * pow(py, 2) + 2 * pow(a3, 2) * pow(py, 2) -
               2 * pow(a1, 2) * pow(pz, 2) + 2 * pow(a2, 2) * pow(pz, 2) +
               2 * pow(a3, 2) * pow(pz, 2) - 2 * pow(px, 2) * pow(py, 2) -
               2 * pow(px, 2) * pow(pz, 2) - 2 * pow(py, 2) * pow(pz, 2) +
               8 * a1 * a2 * a3 * sqrt(pow(px, 2) + pow(py, 2))) /
              ((pow(px, 2) + pow(py, 2)) *
                   (-2 * pow(a1, 2) - 2 * pow(a2, 2) + 4 * a2 * a3 -
                    2 * pow(a3, 2) + pow(px, 2) + pow(py, 2) + 2 * pow(pz, 2)) -
               4 * a2 * pow(a3, 3) - 4 * pow(a2, 3) * a3 + pow(a1, 4) +
               pow(a2, 4) + pow(a3, 4) + pow(pz, 4) -
               2 * pow(a1, 2) * pow(a2, 2) - 2 * pow(a1, 2) * pow(a3, 2) +
               6 * pow(a2, 2) * pow(a3, 2) + 2 * pow(a1, 2) * pow(pz, 2) -
               2 * pow(a2, 2) * pow(pz, 2) - 2 * pow(a3, 2) * pow(pz, 2) +
               4 * pow(a1, 2) * a2 * a3 + 4 * a2 * a3 * pow(pz, 2))));
  double t3 = real(theta3);
  double s3 = sin(t3);
  double c3 = cos(t3);

  std::complex<double> theta2 =
      (pz > 0) ? (-2 * atan((sqrt(-pow(a1, 4) + 2 * pow(a1, 2) * pow(a2, 2) +
                                  4 * pow(a1, 2) * a2 * a3 * c3 +
                                  2 * pow(a1, 2) * pow(a3, 2) * pow(c3, 2) +
                                  2 * pow(a1, 2) * pow(a3, 2) * pow(s3, 2) +
                                  2 * pow(a1, 2) * pow(px, 2) +
                                  2 * pow(a1, 2) * pow(py, 2) +
                                  2 * pow(a1, 2) * pow(pz, 2) - pow(a2, 4) -
                                  4 * pow(a2, 3) * a3 * c3 -
                                  6 * pow(a2, 2) * pow(a3, 2) * pow(c3, 2) -
                                  2 * pow(a2, 2) * pow(a3, 2) * pow(s3, 2) +
                                  2 * pow(a2, 2) * pow(px, 2) +
                                  2 * pow(a2, 2) * pow(py, 2) +
                                  2 * pow(a2, 2) * pow(pz, 2) -
                                  4 * a2 * pow(a3, 3) * pow(c3, 3) -
                                  4 * a2 * pow(a3, 3) * c3 * pow(s3, 2) +
                                  4 * a2 * a3 * c3 * pow(px, 2) +
                                  4 * a2 * a3 * c3 * pow(py, 2) +
                                  4 * a2 * a3 * c3 * pow(pz, 2) -
                                  pow(a3, 4) * pow(c3, 4) -
                                  2 * pow(a3, 4) * pow(c3, 2) * pow(s3, 2) -
                                  pow(a3, 4) * pow(s3, 4) +
                                  2 * pow(a3, 2) * pow(c3, 2) * pow(px, 2) +
                                  2 * pow(a3, 2) * pow(c3, 2) * pow(py, 2) +
                                  2 * pow(a3, 2) * pow(c3, 2) * pow(pz, 2) +
                                  2 * pow(a3, 2) * pow(px, 2) * pow(s3, 2) +
                                  2 * pow(a3, 2) * pow(py, 2) * pow(s3, 2) +
                                  2 * pow(a3, 2) * pow(pz, 2) * pow(s3, 2) -
                                  pow(px, 4) - 2 * pow(px, 2) * pow(py, 2) -
                                  2 * pow(px, 2) * pow(pz, 2) - pow(py, 4) -
                                  2 * pow(py, 2) * pow(pz, 2) - pow(pz, 4)) +
                             2 * a1 * a3 * s3) /
                            (-pow(a1, 2) + 2 * a1 * a2 + 2 * a1 * a3 * c3 -
                             pow(a2, 2) - 2 * a2 * a3 * c3 -
                             pow(a3, 2) * pow(c3, 2) - pow(a3, 2) * pow(s3, 2) +
                             pow(px, 2) + pow(py, 2) + pow(pz, 2))))
               : (2 * atan((sqrt(-pow(a1, 4) + 2 * pow(a1, 2) * pow(a2, 2) +
                                 4 * pow(a1, 2) * a2 * a3 * c3 +
                                 2 * pow(a1, 2) * pow(a3, 2) * pow(c3, 2) +
                                 2 * pow(a1, 2) * pow(a3, 2) * pow(s3, 2) +
                                 2 * pow(a1, 2) * pow(px, 2) +
                                 2 * pow(a1, 2) * pow(py, 2) +
                                 2 * pow(a1, 2) * pow(pz, 2) - pow(a2, 4) -
                                 4 * pow(a2, 3) * a3 * c3 -
                                 6 * pow(a2, 2) * pow(a3, 2) * pow(c3, 2) -
                                 2 * pow(a2, 2) * pow(a3, 2) * pow(s3, 2) +
                                 2 * pow(a2, 2) * pow(px, 2) +
                                 2 * pow(a2, 2) * pow(py, 2) +
                                 2 * pow(a2, 2) * pow(pz, 2) -
                                 4 * a2 * pow(a3, 3) * pow(c3, 3) -
                                 4 * a2 * pow(a3, 3) * c3 * pow(s3, 2) +
                                 4 * a2 * a3 * c3 * pow(px, 2) +
                                 4 * a2 * a3 * c3 * pow(py, 2) +
                                 4 * a2 * a3 * c3 * pow(pz, 2) -
                                 pow(a3, 4) * pow(c3, 4) -
                                 2 * pow(a3, 4) * pow(c3, 2) * pow(s3, 2) -
                                 pow(a3, 4) * pow(s3, 4) +
                                 2 * pow(a3, 2) * pow(c3, 2) * pow(px, 2) +
                                 2 * pow(a3, 2) * pow(c3, 2) * pow(py, 2) +
                                 2 * pow(a3, 2) * pow(c3, 2) * pow(pz, 2) +
                                 2 * pow(a3, 2) * pow(px, 2) * pow(s3, 2) +
                                 2 * pow(a3, 2) * pow(py, 2) * pow(s3, 2) +
                                 2 * pow(a3, 2) * pow(pz, 2) * pow(s3, 2) -
                                 pow(px, 4) - 2 * pow(px, 2) * pow(py, 2) -
                                 2 * pow(px, 2) * pow(pz, 2) - pow(py, 4) -
                                 2 * pow(py, 2) * pow(pz, 2) - pow(pz, 4)) -
                            2 * a1 * a3 * s3) /
                           (-pow(a1, 2) + 2 * a1 * a2 + 2 * a1 * a3 * c3 -
                            pow(a2, 2) - 2 * a2 * a3 * c3 -
                            pow(a3, 2) * pow(c3, 2) - pow(a3, 2) * pow(s3, 2) +
                            pow(px, 2) + pow(py, 2) + pow(pz, 2))));
  double t2 = real(theta2);
  double s2 = sin(t2);
  double c2 = cos(t2);

  double numerator = a1 - px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3;
  numerator =
      (abs(numerator) < static_cast<double>(FLT_EPSILON)) ? 0 : numerator;
  std::complex<double> theta1 =
      2 *
      atan(sqrt(numerator / (a1 + px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3)));
  double t1 = real(theta1);
  double s1 = sin(t1);
  double c1 = cos(t1);

  double t4 =
      atan2((az * (c2 * s3 + c3 * s2)) /
                    (pow(c2, 2) * pow(c3, 2) + pow(c2, 2) * pow(s3, 2) +
                     pow(c3, 2) * pow(s2, 2) + pow(s2, 2) * pow(s3, 2)) -
                (ax * (c1 * c2 * c3 - c1 * s2 * s3)) /
                    (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) +
                     pow(c1, 2) * pow(c2, 2) * pow(s3, 2) +
                     pow(c1, 2) * pow(c3, 2) * pow(s2, 2) +
                     pow(c1, 2) * pow(s2, 2) * pow(s3, 2) +
                     pow(c2, 2) * pow(c3, 2) * pow(s1, 2) +
                     pow(c2, 2) * pow(s1, 2) * pow(s3, 2) +
                     pow(c3, 2) * pow(s1, 2) * pow(s2, 2) +
                     pow(s1, 2) * pow(s2, 2) * pow(s3, 2)) -
                (ay * (c2 * c3 * s1 - s1 * s2 * s3)) /
                    (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) +
                     pow(c1, 2) * pow(c2, 2) * pow(s3, 2) +
                     pow(c1, 2) * pow(c3, 2) * pow(s2, 2) +
                     pow(c1, 2) * pow(s2, 2) * pow(s3, 2) +
                     pow(c2, 2) * pow(c3, 2) * pow(s1, 2) +
                     pow(c2, 2) * pow(s1, 2) * pow(s3, 2) +
                     pow(c3, 2) * pow(s1, 2) * pow(s2, 2) +
                     pow(s1, 2) * pow(s2, 2) * pow(s3, 2)),
            -(ax * (c1 * c2 * s3 + c1 * c3 * s2)) /
                    (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) +
                     pow(c1, 2) * pow(c2, 2) * pow(s3, 2) +
                     pow(c1, 2) * pow(c3, 2) * pow(s2, 2) +
                     pow(c1, 2) * pow(s2, 2) * pow(s3, 2) +
                     pow(c2, 2) * pow(c3, 2) * pow(s1, 2) +
                     pow(c2, 2) * pow(s1, 2) * pow(s3, 2) +
                     pow(c3, 2) * pow(s1, 2) * pow(s2, 2) +
                     pow(s1, 2) * pow(s2, 2) * pow(s3, 2)) -
                (az * (c2 * c3 - s2 * s3)) /
                    (pow(c2, 2) * pow(c3, 2) + pow(c2, 2) * pow(s3, 2) +
                     pow(c3, 2) * pow(s2, 2) + pow(s2, 2) * pow(s3, 2)) -
                (ay * s1 * (c2 * s3 + c3 * s2)) /
                    (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) +
                     pow(c1, 2) * pow(c2, 2) * pow(s3, 2) +
                     pow(c1, 2) * pow(c3, 2) * pow(s2, 2) +
                     pow(c1, 2) * pow(s2, 2) * pow(s3, 2) +
                     pow(c2, 2) * pow(c3, 2) * pow(s1, 2) +
                     pow(c2, 2) * pow(s1, 2) * pow(s3, 2) +
                     pow(c3, 2) * pow(s1, 2) * pow(s2, 2) +
                     pow(s1, 2) * pow(s2, 2) * pow(s3, 2)));
  double s4 = sin(t4);
  double c4 = cos(t4);

  double t5 = atan2(
      (oz * (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4)) /
              (pow(c2, 2) * pow(c3, 2) * pow(c4, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c4, 2) * pow(s3, 2) +
               pow(c2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c3, 2) * pow(c4, 2) * pow(s2, 2) +
               pow(c3, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c4, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(s2, 2) * pow(s3, 2) * pow(s4, 2)) +
          (oy * (c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4 + c4 * s1 * s2 * s3 -
                 c2 * c3 * c4 * s1)) /
              (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(c4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c4, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(c4, 2) * pow(s2, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c4, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(c4, 2) * pow(s1, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(s1, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c4, 2) * pow(s1, 2) * pow(s3, 2) +
               pow(c2, 2) * pow(s1, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c3, 2) * pow(c4, 2) * pow(s1, 2) * pow(s2, 2) +
               pow(c3, 2) * pow(s1, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c4, 2) * pow(s1, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(s1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2)) +
          (c1 * ox *
           (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3)) /
              (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(c4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c4, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(c4, 2) * pow(s2, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c4, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(c4, 2) * pow(s1, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(s1, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c4, 2) * pow(s1, 2) * pow(s3, 2) +
               pow(c2, 2) * pow(s1, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c3, 2) * pow(c4, 2) * pow(s1, 2) * pow(s2, 2) +
               pow(c3, 2) * pow(s1, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c4, 2) * pow(s1, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(s1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2)),
      -(nz * (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4)) /
              (pow(c2, 2) * pow(c3, 2) * pow(c4, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c4, 2) * pow(s3, 2) +
               pow(c2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c3, 2) * pow(c4, 2) * pow(s2, 2) +
               pow(c3, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c4, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(s2, 2) * pow(s3, 2) * pow(s4, 2)) -
          (ny * (c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4 + c4 * s1 * s2 * s3 -
                 c2 * c3 * c4 * s1)) /
              (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(c4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c4, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(c4, 2) * pow(s2, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c4, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(c4, 2) * pow(s1, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(s1, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c4, 2) * pow(s1, 2) * pow(s3, 2) +
               pow(c2, 2) * pow(s1, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c3, 2) * pow(c4, 2) * pow(s1, 2) * pow(s2, 2) +
               pow(c3, 2) * pow(s1, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c4, 2) * pow(s1, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(s1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2)) -
          (c1 * nx *
           (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3)) /
              (pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(c4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(c4, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(c2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(c4, 2) * pow(s2, 2) +
               pow(c1, 2) * pow(c3, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c1, 2) * pow(c4, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(c1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(c4, 2) * pow(s1, 2) +
               pow(c2, 2) * pow(c3, 2) * pow(s1, 2) * pow(s4, 2) +
               pow(c2, 2) * pow(c4, 2) * pow(s1, 2) * pow(s3, 2) +
               pow(c2, 2) * pow(s1, 2) * pow(s3, 2) * pow(s4, 2) +
               pow(c3, 2) * pow(c4, 2) * pow(s1, 2) * pow(s2, 2) +
               pow(c3, 2) * pow(s1, 2) * pow(s2, 2) * pow(s4, 2) +
               pow(c4, 2) * pow(s1, 2) * pow(s2, 2) * pow(s3, 2) +
               pow(s1, 2) * pow(s2, 2) * pow(s3, 2) * pow(s4, 2)));

  // Check joints' limit
  if (t1 < -2 * M_PI / 3 || t1 > 2 * M_PI / 3 || t2 < -M_PI || t2 > 0 ||
      t3 < -2 || t3 > 2 || t4 < -2 || t4 > 2 || t5 < -2 * M_PI / 3 ||
      t5 > 2 * M_PI / 3) {
    return false;
  }

  solution.resize(JOINT_NUM);
  solution[0] = t1;
  solution[1] = t2 + M_PI / 2;
  solution[2] = t3;
  solution[3] = t4;
  solution[4] = t5;

  return true;
}

bool XArmKinematicsPlugin::isPoseReachable(
    const geometry_msgs::Pose& pose) const {
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                   pose.orientation.w);
  tf::Matrix3x3 rotateMatrix(q);
  double roll, pitch, yaw;
  rotateMatrix.getRPY(roll, pitch, yaw);

  constexpr double threshold = 0.01;

  if (pose.position.x == 0 &&
      ((M_PI / 2 - threshold <= yaw && yaw <= M_PI / 2 + threshold) ||
       (-M_PI / 2 - threshold <= yaw && yaw <= -M_PI / 2 + threshold))) {
    return true;
  }

  auto y = pose.position.x * tan(yaw);
  if (y - threshold <= pose.position.y && pose.position.y <= y + threshold) {
    return true;
  }

  return false;
}

bool XArmKinematicsPlugin::revisePose(geometry_msgs::Pose& pose) const {
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                   pose.orientation.w);
  tf::Matrix3x3 rotateMatrix(q);
  double roll, pitch, yaw;
  rotateMatrix.getRPY(roll, pitch, yaw, 2);

  if (pose.position.x != 0) {
    yaw = atan(pose.position.y / pose.position.x);
    pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  }

  return isPoseReachable(pose);
}

}  // namespace xarm_kinematics_plugin

int main() { return 0; }
