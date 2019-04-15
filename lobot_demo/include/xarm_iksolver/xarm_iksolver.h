#ifndef XARM_IKSOLVER_H
#define XARM_IKSOLVER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <array>
#include <string>

namespace lobot_ik {

#define PI 3.1415926

#define JOINT_NUM 5

using RotMat = std::array<std::array<double, 3>, 3>;
using PosVec = std::array<double, 3>;

class Quaternion {
  friend Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
  friend Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);

 public:
  enum AngleType { RPY, XYZ, ZXZ, ZYX };

  Quaternion() = default;
  Quaternion(const double a, const double b, const double c, const double d)
      : w_(a), x_(b), y_(c), z_(d) {}
  Quaternion(const RotMat& r);
  Quaternion(AngleType type, const double a, const double b, const double c);

  Quaternion& operator+=(const Quaternion& rhs);

  RotMat ToRotateMatrix() const;

 private:
  double w_;
  double x_;
  double y_;
  double z_;
};

class Position {
  friend Position operator+(const Position& lhs, const Position& rhs);
  friend class XArmIkSolver;

 public:
  Position() = default;
  Position(const double a, const double b, const double c)
      : x_(a), y_(b), z_(c) {}

  Position& operator+=(const Position& rhs);

 private:
  double x_;
  double y_;
  double z_;
};

class XArmIkSolver {
 public:
  XArmIkSolver() = default;
  ~XArmIkSolver();

  bool SetTargetValue(const Position& p, const Quaternion& q,
                      moveit::planning_interface::MoveGroupInterface& mg);

 private:
  std::array<double, JOINT_NUM> jointValueVec_;

  bool SolveIk(const PosVec& p, const RotMat& r);
};

}  // namespace lobot_ik

#endif  // XARM_IKSOLVER_H
