#ifndef XARM_IK_H
#define XARM_IK_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <array>

#include "xarm_pose/xarm_pose.hpp"

#define JOINT_NUM 5

namespace lobot_ik {

class XArmIk {
 public:
  XArmIk() = default;

  bool SetTargetValue(XArmPose pose,
                      moveit::planning_interface::MoveGroupInterface& group);

 private:
  std::array<double, JOINT_NUM> jointValueVec_;

  bool SolveIk(const PosVec& p, const RotMat& r);
};

}  // namespace lobot_ik

#endif  // XARM_IK_H
