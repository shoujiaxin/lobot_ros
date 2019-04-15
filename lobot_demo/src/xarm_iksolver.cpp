#include "xarm_iksolver/xarm_iksolver.h"
#include <complex>

namespace lobot_ik {

// Construct quaternion from rotate matrix
Quaternion::Quaternion(const RotMat& r) {
  w_ = sqrt(1 + r[0][0] + r[1][1] + r[2][2]) / 2;
  x_ = (r[2][1] - r[1][2]) / w_ / 4;
  y_ = (r[0][2] - r[2][0]) / w_ / 4;
  z_ = (r[1][0] - r[0][1]) / w_ / 4;
}

// Construct quaternion from RPY or Euler angle
Quaternion::Quaternion(AngleType type, const double a, const double b,
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

Quaternion& Quaternion::operator+=(const Quaternion& rhs) {
  w_ += rhs.w_;
  x_ += rhs.x_;
  y_ += rhs.y_;
  z_ += rhs.z_;
  return *this;
}

RotMat Quaternion::ToRotateMatrix() const {
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

Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) {
  Quaternion sum = lhs;
  sum += rhs;
  return sum;
}

Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) {
  Quaternion product;
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

Position& Position::operator+=(const Position& rhs) {
  x_ += rhs.x_;
  y_ += rhs.y_;
  z_ += rhs.z_;
  return *this;
}

Position operator+(const Position& lhs, const Position& rhs) {
  Position sum = lhs;
  sum += rhs;
  return sum;
}

bool XArmIkSolver::SetTargetValue(
    const Position& p, const Quaternion& q,
    moveit::planning_interface::MoveGroupInterface& mg) {
  if (!SolveIk({p.x_, p.y_, p.z_}, q.ToRotateMatrix())) {
    return false;
  }

  auto jointNames = mg.getJointNames();
  auto nameIt = jointNames.cbegin();
  auto valueIt = jointValueVec_.cbegin();
  while (nameIt != jointNames.cend()) {
    mg.setJointValueTarget(*nameIt, *valueIt);
    ++nameIt;
    ++valueIt;
  }

  return true;
}

bool XArmIkSolver::SolveIk(const PosVec& p, const RotMat& r) {
  constexpr double a1 = 3;
  constexpr double a2 = 96;
  constexpr double a3 = 96;
  constexpr double baseHeight = 72;   // Height of base relative to world
  constexpr double toolLength = 100;  // Length of terminal tool

  const double nx = r[0][2], ny = r[1][2], nz = r[2][2];
  const double ox = -r[0][1], oy = -r[1][1], oz = -r[2][1];
  const double ax = r[0][0], ay = r[1][0], az = r[2][0];
  const double px = p[0] - toolLength * r[0][0];
  const double py = p[1] - toolLength * r[1][0];
  const double pz = p[2] - toolLength * r[2][0] - baseHeight;

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

  if (t1 < -2 * PI / 3 || t1 > 2 * PI / 3 || t2 < -PI / 2 || t2 > PI / 2 ||
      t3 < -2 || t3 > 2 || t4 < -2 || t4 > 2 || t5 < -2 * PI / 3 ||
      t5 > 2 * PI / 3) {
    return false;
  }

  jointValueVec_[0] = t1;
  jointValueVec_[1] = t2;
  jointValueVec_[2] = t3;
  jointValueVec_[3] = t4;
  jointValueVec_[4] = t5;

  return true;
}

}  // namespace lobot_ik
