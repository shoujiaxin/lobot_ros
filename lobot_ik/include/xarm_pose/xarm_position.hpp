#ifndef XARM_POSITION_H
#define XARM_POSITION_H

#include <array>
#include <iostream>

namespace lobot_ik {

using PosVec = std::array<double, 3>;

class XArmPosition {
  friend XArmPosition operator+(const XArmPosition& lhs,
                                const XArmPosition& rhs);
  friend std::ostream& operator<<(std::ostream& os, const XArmPosition& p);
  friend class XArmPose;

 public:
  XArmPosition() : x_(0), y_(0), z_(0){};
  XArmPosition(const double a, const double b, const double c)
      : x_(a), y_(b), z_(c) {}

  XArmPosition& operator+=(const XArmPosition& rhs);

  PosVec ToPositionVector() const { return {x_, y_, z_}; }

 private:
  double x_;
  double y_;
  double z_;
};

XArmPosition& XArmPosition::operator+=(const XArmPosition& rhs) {
  x_ += rhs.x_;
  y_ += rhs.y_;
  z_ += rhs.z_;
  return *this;
}

XArmPosition operator+(const XArmPosition& lhs, const XArmPosition& rhs) {
  auto sum = lhs;
  sum += rhs;
  return sum;
}
std::ostream& operator<<(std::ostream& os, const XArmPosition& p) {
  os << p.x_ << ", " << p.y_ << ", " << p.z_;
  return os;
}

}  // namespace lobot_ik

#endif  // XARM_POSITION_H
