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
  bool SolveIk(const geometry_msgs::Point& p, tf::Matrix3x3& r);
};

inline bool XArmIk::IsPoseReachable(const geometry_msgs::Pose& pose) {
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                   pose.orientation.w);
  tf::Matrix3x3 rotateMatrix(q);
  double roll, pitch, yaw;
  rotateMatrix.getRPY(roll, pitch, yaw);

  constexpr double threshold = 0.01;

  if (pose.position.x == 0 && abs(sin(yaw)) > 0.99) {
    return true;
  }

  auto y = pose.position.x * tan(yaw);
  if (y - threshold <= pose.position.y && pose.position.y <= y + threshold) {
    return true;
  }

  return false;
}

}  // namespace lobot_ik

#endif  // XARM_IK_H
