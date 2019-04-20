#ifndef XARM_IK_H
#define XARM_IK_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <array>

namespace lobot_ik {

#define JOINT_NUM 5

class XArmIk {
 public:
  XArmIk() = default;

  bool SetPoseTarget(const geometry_msgs::Pose& p,
                     moveit::planning_interface::MoveGroupInterface& group);

 private:
  std::array<double, JOINT_NUM> jointValueVec_;

  bool IsPoseReachable(const geometry_msgs::Pose& pose);
  bool RevisePose(geometry_msgs::Pose& pose);
  void QuaternionToRPY(const geometry_msgs::Quaternion& q, double& roll,
                       double& pitch, double& yaw);
  bool SolveIk(const geometry_msgs::Point& p, tf::Matrix3x3& r);
};

inline bool XArmIk::IsPoseReachable(const geometry_msgs::Pose& pose) {
  if (pose.position.z < 0) {
    return false;
  }

  double roll, pitch, yaw;
  QuaternionToRPY(pose.orientation, roll, pitch, yaw);

  if (pose.position.x == 0 && abs(sin(yaw)) > 0.99) {
    return true;
  }

  constexpr double threshold = 0.01;
  auto y = pose.position.x * tan(yaw);
  if (y - threshold <= pose.position.y && pose.position.y <= y + threshold) {
    return true;
  }

  return false;
}

inline void XArmIk::QuaternionToRPY(const geometry_msgs::Quaternion& q,
                                    double& roll, double& pitch, double& yaw) {
  // Roll, X axis
  roll = atan2(2 * (q.y * q.z + q.w * q.x),
               q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
  // Pitch, Y axis
  pitch = asin(-2 * (q.x * q.z - q.w * q.y));
  // Yaw, Z axis
  yaw = atan2(2 * (q.x * q.y + q.w * q.z),
              q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
}

}  // namespace lobot_ik

#endif  // XARM_IK_H
