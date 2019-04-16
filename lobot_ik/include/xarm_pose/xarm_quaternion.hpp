#ifndef XARM_QUATERNION_H
#define XARM_QUATERNION_H

#include <array>
#include <complex>
#include <iostream>

namespace lobot_ik {

using RotMat = std::array<std::array<double, 3>, 3>;

class XArmQuaternion {
  friend XArmQuaternion operator+(const XArmQuaternion& lhs,
                                  const XArmQuaternion& rhs);
  friend XArmQuaternion operator*(const XArmQuaternion& lhs,
                                  const XArmQuaternion& rhs);
  friend std::ostream& operator<<(std::ostream& os, const XArmQuaternion& q);
  friend class XArmPose;

 public:
  enum AngleType { RPY, XYZ, ZXZ, ZYX };

  XArmQuaternion() : w_(0), x_(0), y_(0), z_(0) {}
  XArmQuaternion(const double a, const double b, const double c, const double d)
      : w_(a), x_(b), y_(c), z_(d) {}
  XArmQuaternion(const RotMat& r);
  XArmQuaternion(AngleType type, const double a, const double b,
                 const double c);

  XArmQuaternion& operator+=(const XArmQuaternion& rhs);

  RotMat ToRotateMatrix() const;
  std::array<double, 3> ToRpy() const;

 private:
  double w_;
  double x_;
  double y_;
  double z_;
};

// Construct quaternion from rotate matrix
XArmQuaternion::XArmQuaternion(const RotMat& r) {
  w_ = sqrt(1 + r[0][0] + r[1][1] + r[2][2]) / 2;
  x_ = (r[2][1] - r[1][2]) / w_ / 4;
  y_ = (r[0][2] - r[2][0]) / w_ / 4;
  z_ = (r[1][0] - r[0][1]) / w_ / 4;
}

// Construct quaternion from RPY or Euler angle
XArmQuaternion::XArmQuaternion(AngleType type, const double a, const double b,
                               const double c) {
  switch (type) {
    case RPY:
      // The same with the ZYX Euler angle
      // R (a) -> X axis, P (b) -> Y axis, Y (c) -> Z axis
      w_ = cos(a / 2) * cos(b / 2) * cos(c / 2) +
           sin(a / 2) * sin(b / 2) * sin(c / 2);
      x_ = cos(b / 2) * cos(c / 2) * sin(a / 2) -
           cos(a / 2) * sin(b / 2) * sin(c / 2);
      y_ = cos(a / 2) * cos(c / 2) * sin(b / 2) +
           cos(b / 2) * sin(a / 2) * sin(c / 2);
      z_ = cos(a / 2) * cos(b / 2) * sin(c / 2) -
           cos(c / 2) * sin(a / 2) * sin(b / 2);
      break;

    case XYZ:
      // a -> X axis, b -> Y axis, c -> Z axis
      w_ = cos(a / 2) * cos(b / 2) * cos(c / 2) -
           sin(a / 2) * sin(b / 2) * sin(c / 2);
      x_ = cos(b / 2) * cos(c / 2) * sin(a / 2) +
           cos(a / 2) * sin(b / 2) * sin(c / 2);
      y_ = cos(a / 2) * cos(c / 2) * sin(b / 2) -
           cos(b / 2) * sin(a / 2) * sin(c / 2);
      z_ = cos(a / 2) * cos(b / 2) * sin(c / 2) +
           cos(c / 2) * sin(a / 2) * sin(b / 2);
      break;

    case ZXZ:
      // a -> Z axis, b -> X axis, c -> Z axis
      w_ = cos(a / 2) * cos(b / 2) * cos(c / 2) -
           cos(b / 2) * sin(a / 2) * sin(c / 2);
      x_ = cos(a / 2) * cos(c / 2) * sin(b / 2) +
           sin(a / 2) * sin(b / 2) * sin(c / 2);
      y_ = cos(c / 2) * sin(a / 2) * sin(b / 2) -
           cos(a / 2) * sin(b / 2) * sin(c / 2);
      z_ = cos(a / 2) * cos(b / 2) * sin(c / 2) +
           cos(b / 2) * cos(c / 2) * sin(a / 2);
      break;

    case ZYX:
      // a -> Z axis, b -> Y axis, c -> X axis
      w_ = cos(a / 2) * cos(b / 2) * cos(c / 2) +
           sin(a / 2) * sin(b / 2) * sin(c / 2);
      x_ = cos(a / 2) * cos(b / 2) * sin(c / 2) -
           cos(c / 2) * sin(a / 2) * sin(b / 2);
      y_ = cos(a / 2) * cos(c / 2) * sin(b / 2) +
           cos(b / 2) * sin(a / 2) * sin(c / 2);
      z_ = cos(b / 2) * cos(c / 2) * sin(a / 2) -
           cos(a / 2) * sin(b / 2) * sin(c / 2);
      break;

    default:
      break;
  }
}

XArmQuaternion& XArmQuaternion::operator+=(const XArmQuaternion& rhs) {
  w_ += rhs.w_;
  x_ += rhs.x_;
  y_ += rhs.y_;
  z_ += rhs.z_;
  return *this;
}

RotMat XArmQuaternion::ToRotateMatrix() const {
  RotMat r;
  r[0][0] = w_ * w_ + x_ * x_ - y_ * y_ - z_ * z_;
  r[0][1] = 2 * x_ * y_ - 2 * w_ * z_;
  r[0][2] = 2 * x_ * z_ + 2 * w_ * y_;
  r[1][0] = 2 * x_ * y_ + 2 * w_ * z_;
  r[1][1] = w_ * w_ - x_ * x_ + y_ * y_ - z_ * z_;
  r[1][2] = 2 * y_ * z_ - 2 * w_ * x_;
  r[2][0] = 2 * x_ * z_ - 2 * w_ * y_;
  r[2][1] = 2 * y_ * z_ + 2 * w_ * x_;
  r[2][2] = w_ * w_ - x_ * x_ - y_ * y_ + z_ * z_;
  return r;
}

std::array<double, 3> XArmQuaternion::ToRpy() const {
  std::array<double, 3> euler;
  // Roll, X axis
  euler[0] =
      atan2(2 * (y_ * z_ + w_ * x_), w_ * w_ - x_ * x_ - y_ * y_ + z_ * z_);
  // Pitch, Y axis
  euler[1] = asin(-2 * (x_ * z_ - w_ * y_));
  // Yaw, Z axis
  euler[2] =
      atan2(2 * (x_ * y_ + w_ * z_), w_ * w_ + x_ * x_ - y_ * y_ - z_ * z_);
  return euler;
}

XArmQuaternion operator+(const XArmQuaternion& lhs, const XArmQuaternion& rhs) {
  auto sum = lhs;
  sum += rhs;
  return sum;
}

XArmQuaternion operator*(const XArmQuaternion& lhs, const XArmQuaternion& rhs) {
  XArmQuaternion product;
  product.w_ =
      rhs.w_ * lhs.w_ - rhs.x_ * lhs.x_ - rhs.y_ * lhs.y_ - rhs.z_ * lhs.z_;
  product.x_ =
      rhs.w_ * lhs.x_ + rhs.x_ * lhs.w_ - rhs.y_ * lhs.z_ + rhs.z_ * lhs.y_;
  product.y_ =
      rhs.w_ * lhs.y_ + rhs.x_ * lhs.z_ + rhs.y_ * lhs.w_ - rhs.z_ * lhs.x_;
  product.z_ =
      rhs.w_ * lhs.z_ - rhs.x_ * lhs.y_ + rhs.y_ * lhs.x_ + rhs.z_ * lhs.w_;
  return product;
}

std::ostream& operator<<(std::ostream& os, const XArmQuaternion& q) {
  os << q.w_ << ", " << q.x_ << ", " << q.y_ << ", " << q.z_;
  return os;
}

}  // namespace lobot_ik

#endif  // XARM_QUATERNION_H
