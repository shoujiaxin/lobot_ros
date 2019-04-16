#ifndef XARM_POSE_H
#define XARM_POSE_H

#include <iostream>

#include "xarm_position.hpp"
#include "xarm_quaternion.hpp"

#define PI 3.1415926

namespace lobot_ik {

class XArmPose {
  friend std::ostream &operator<<(std::ostream &os, const XArmPose &pose);

 public:
  XArmPose() = default;
  XArmPose(const XArmPosition &p, const XArmQuaternion &q)
      : position_(p), orientation_(q) {}

  PosVec GetPositionVector() { return position_.ToPositionVector(); }
  RotMat GetRotateMatrix() { return orientation_.ToRotateMatrix(); }
  bool IsReachable() const;
  void ReviseY();
  void SetOrientation(const XArmQuaternion &q) { orientation_ = q; }
  void SetPosition(const XArmPosition &p) { position_ = p; }

 private:
  XArmPosition position_;
  XArmQuaternion orientation_;
};

// Check reachability
bool XArmPose::IsReachable() const {
  auto euler = orientation_.ToRpy();
  constexpr double threshold = 0.01;

  if (position_.x_ == 0 &&
      ((PI / 2 - threshold <= euler[2] && euler[2] <= PI / 2 + threshold) ||
       (-PI / 2 - threshold <= euler[2] && euler[2] <= -PI / 2 + threshold))) {
    return true;
  }

  auto y = position_.x_ * tan(euler[2]);
  if (y - threshold <= position_.y_ && position_.y_ <= y + threshold) {
    return true;
  }

  return false;
}

// revise Y value of position
void XArmPose::ReviseY() {
  auto euler = orientation_.ToRpy();
  if (position_.x_ != 0) {
    position_.y_ = position_.x_ * tan(euler[2]);
  }
}

std::ostream &operator<<(std::ostream &os, const XArmPose &pose) {
  os << "Position: " << pose.position_ << std::endl
     << "Orientation: " << pose.orientation_;
  return os;
}

}  // namespace lobot_ik

#endif  // XARM_POSE_H
