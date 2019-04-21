#include "xarm_ik/xarm_ik.h"

#include <iostream>

namespace lobot_ik {

bool XArmIk::SetPoseTarget(
    geometry_msgs::Pose pose,
    moveit::planning_interface::MoveGroupInterface& group) {
  // Make a copy of the pose
  if (!IsPoseReachable(pose) && !RevisePose(pose)) {
    ROS_ERROR_NAMED("xarm_ik", "The target pose is not reachable");
    return false;
  }

  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                   pose.orientation.w);
  tf::Matrix3x3 rotateMatrix(q);
  if (!SolveIk(pose.position, rotateMatrix)) {
    return false;
  }

  auto jointNames = group.getJointNames();
  auto nameIt = jointNames.cbegin();
  auto valueIt = jointValueVec_.cbegin();
  while (nameIt != jointNames.cend()) {
    std::cout << *nameIt << ": " << *valueIt << std::endl;
    group.setJointValueTarget(*nameIt, *valueIt);
    ++nameIt;
    ++valueIt;
  }

  return true;
}

bool XArmIk::SolveIk(const geometry_msgs::Point& p, tf::Matrix3x3& r) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (abs(r[i][j]) < FLT_EPSILON) {
        r[i][j] = 0;
      }
    }
  }

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

  std::complex<double> theta3;
  if (p.y >= 0) {
    theta3 =
        2 *
        atan(sqrt(
            (2 * a1 * a1 * a2 * a2 - a2 * a2 * a2 * a2 - a3 * a3 * a3 * a3 -
             px * px * px * px - py * py * py * py - pz * pz * pz * pz -
             a1 * a1 * a1 * a1 + 2 * a1 * a1 * a3 * a3 + 2 * a2 * a2 * a3 * a3 +
             2 * a1 * a1 * px * px + 2 * a2 * a2 * px * px +
             2 * a3 * a3 * px * px + 2 * a1 * a1 * py * py +
             2 * a2 * a2 * py * py + 2 * a3 * a3 * py * py -
             2 * a1 * a1 * pz * pz + 2 * a2 * a2 * pz * pz +
             2 * a3 * a3 * pz * pz - 2 * px * px * py * py -
             2 * px * px * pz * pz - 2 * py * py * pz * pz +
             8 * a1 * a2 * a3 * sqrt(px * px + py * py)) /
            ((px * px + py * py) *
                 (-2 * a1 * a1 - 2 * a2 * a2 + 4 * a2 * a3 - 2 * a3 * a3 +
                  px * px + py * py + 2 * pz * pz) -
             4 * a2 * a3 * a3 * a3 - 4 * a2 * a2 * a2 * a3 + a1 * a1 * a1 * a1 +
             a2 * a2 * a2 * a2 + a3 * a3 * a3 * a3 + pz * pz * pz * pz -
             2 * a1 * a1 * a2 * a2 - 2 * a1 * a1 * a3 * a3 +
             6 * a2 * a2 * a3 * a3 + 2 * a1 * a1 * pz * pz -
             2 * a2 * a2 * pz * pz - 2 * a3 * a3 * pz * pz +
             4 * a1 * a1 * a2 * a3 + 4 * a2 * a3 * pz * pz)));
  } else {
    theta3 =
        -2 *
        atan(sqrt(-(a1 * a1 * a1 * a1 + a2 * a2 * a2 * a2 + a3 * a3 * a3 * a3 +
                    px * px * px * px + py * py * py * py + pz * pz * pz * pz -
                    2 * a1 * a1 * a2 * a2 - 2 * a1 * a1 * a3 * a3 -
                    2 * a2 * a2 * a3 * a3 - 2 * a1 * a1 * px * px -
                    2 * a2 * a2 * px * px - 2 * a3 * a3 * px * px -
                    2 * a1 * a1 * py * py - 2 * a2 * a2 * py * py -
                    2 * a3 * a3 * py * py + 2 * a1 * a1 * pz * pz -
                    2 * a2 * a2 * pz * pz - 2 * a3 * a3 * pz * pz +
                    2 * px * px * py * py + 2 * px * px * pz * pz +
                    2 * py * py * pz * pz +
                    8 * a1 * a2 * a3 * sqrt(px * px + py * py)) /
                  ((px * px + py * py) *
                       (-2 * a1 * a1 - 2 * a2 * a2 + 4 * a2 * a3 - 2 * a3 * a3 +
                        px * px + py * py + 2 * pz * pz) -
                   4 * a2 * a3 * a3 * a3 - 4 * a2 * a2 * a2 * a3 +
                   a1 * a1 * a1 * a1 + a2 * a2 * a2 * a2 + a3 * a3 * a3 * a3 +
                   pz * pz * pz * pz - 2 * a1 * a1 * a2 * a2 -
                   2 * a1 * a1 * a3 * a3 + 6 * a2 * a2 * a3 * a3 +
                   2 * a1 * a1 * pz * pz - 2 * a2 * a2 * pz * pz -
                   2 * a3 * a3 * pz * pz + 4 * a1 * a1 * a2 * a3 +
                   4 * a2 * a3 * pz * pz)));
  }
  double t3 = real(theta3);
  double s3 = sin(t3);
  double c3 = cos(t3);

  std::complex<double> theta2 =
      (pz > 0)
          ? (-2 *
             atan(
                 (sqrt(-a1 * a1 * a1 * a1 + 2 * a1 * a1 * a2 * a2 +
                       4 * a1 * a1 * a2 * a3 * c3 +
                       2 * a1 * a1 * a3 * a3 * c3 * c3 +
                       2 * a1 * a1 * a3 * a3 * s3 * s3 + 2 * a1 * a1 * px * px +
                       2 * a1 * a1 * py * py + 2 * a1 * a1 * pz * pz -
                       a2 * a2 * a2 * a2 - 4 * a2 * a2 * a2 * a3 * c3 -
                       6 * a2 * a2 * a3 * a3 * c3 * c3 -
                       2 * a2 * a2 * a3 * a3 * s3 * s3 + 2 * a2 * a2 * px * px +
                       2 * a2 * a2 * py * py + 2 * a2 * a2 * pz * pz -
                       4 * a2 * a3 * a3 * a3 * c3 * c3 * c3 -
                       4 * a2 * a3 * a3 * a3 * c3 * s3 * s3 +
                       4 * a2 * a3 * c3 * px * px + 4 * a2 * a3 * c3 * py * py +
                       4 * a2 * a3 * c3 * pz * pz -
                       a3 * a3 * a3 * a3 * c3 * c3 * c3 * c3 -
                       2 * a3 * a3 * a3 * a3 * c3 * c3 * s3 * s3 -
                       a3 * a3 * a3 * a3 * s3 * s3 * s3 * s3 +
                       2 * a3 * a3 * c3 * c3 * px * px +
                       2 * a3 * a3 * c3 * c3 * py * py +
                       2 * a3 * a3 * c3 * c3 * pz * pz +
                       2 * a3 * a3 * px * px * s3 * s3 +
                       2 * a3 * a3 * py * py * s3 * s3 +
                       2 * a3 * a3 * pz * pz * s3 * s3 - px * px * px * px -
                       2 * px * px * py * py - 2 * px * px * pz * pz -
                       py * py * py * py - 2 * py * py * pz * pz -
                       pz * pz * pz * pz) +
                  2 * a1 * a3 * s3) /
                 (-a1 * a1 + 2 * a1 * a2 + 2 * a1 * a3 * c3 - a2 * a2 -
                  2 * a2 * a3 * c3 - a3 * a3 * c3 * c3 - a3 * a3 * s3 * s3 +
                  px * px + py * py + pz * pz)))
          : (2 * atan((sqrt(-a1 * a1 * a1 * a1 + 2 * a1 * a1 * a2 * a2 +
                            4 * a1 * a1 * a2 * a3 * c3 +
                            2 * a1 * a1 * a3 * a3 * c3 * c3 +
                            2 * a1 * a1 * a3 * a3 * s3 * s3 +
                            2 * a1 * a1 * px * px + 2 * a1 * a1 * py * py +
                            2 * a1 * a1 * pz * pz - a2 * a2 * a2 * a2 -
                            4 * a2 * a2 * a2 * a3 * c3 -
                            6 * a2 * a2 * a3 * a3 * c3 * c3 -
                            2 * a2 * a2 * a3 * a3 * s3 * s3 +
                            2 * a2 * a2 * px * px + 2 * a2 * a2 * py * py +
                            2 * a2 * a2 * pz * pz -
                            4 * a2 * a3 * a3 * a3 * c3 * c3 * c3 -
                            4 * a2 * a3 * a3 * a3 * c3 * s3 * s3 +
                            4 * a2 * a3 * c3 * px * px +
                            4 * a2 * a3 * c3 * py * py +
                            4 * a2 * a3 * c3 * pz * pz -
                            a3 * a3 * a3 * a3 * c3 * c3 * c3 * c3 -
                            2 * a3 * a3 * a3 * a3 * c3 * c3 * s3 * s3 -
                            a3 * a3 * a3 * a3 * s3 * s3 * s3 * s3 +
                            2 * a3 * a3 * c3 * c3 * px * px +
                            2 * a3 * a3 * c3 * c3 * py * py +
                            2 * a3 * a3 * c3 * c3 * pz * pz +
                            2 * a3 * a3 * px * px * s3 * s3 +
                            2 * a3 * a3 * py * py * s3 * s3 +
                            2 * a3 * a3 * pz * pz * s3 * s3 -
                            px * px * px * px - 2 * px * px * py * py -
                            2 * px * px * pz * pz - py * py * py * py -
                            2 * py * py * pz * pz - pz * pz * pz * pz) -
                       2 * a1 * a3 * s3) /
                      (-a1 * a1 + 2 * a1 * a2 + 2 * a1 * a3 * c3 - a2 * a2 -
                       2 * a2 * a3 * c3 - a3 * a3 * c3 * c3 -
                       a3 * a3 * s3 * s3 + px * px + py * py + pz * pz)));
  double t2 = real(theta2);
  double s2 = sin(t2);
  double c2 = cos(t2);

  double numerator = a1 - px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3;
  numerator = (abs(numerator) < FLT_EPSILON) ? 0 : numerator;
  std::complex<double> theta1 =
      2 *
      atan(sqrt(numerator / (a1 + px + a2 * c2 + a3 * c2 * c3 - a3 * s2 * s3)));
  double t1 = real(theta1);
  double s1 = sin(t1);
  double c1 = cos(t1);

  double t4 = atan2(
      (az * (c2 * s3 + c3 * s2)) / (c2 * c2 * c3 * c3 + c2 * c2 * s3 * s3 +
                                    c3 * c3 * s2 * s2 + s2 * s2 * s3 * s3) -
          (ax * (c1 * c2 * c3 - c1 * s2 * s3)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 +
               c1 * c1 * c3 * c3 * s2 * s2 + c1 * c1 * s2 * s2 * s3 * s3 +
               c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3) -
          (ay * (c2 * c3 * s1 - s1 * s2 * s3)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 +
               c1 * c1 * c3 * c3 * s2 * s2 + c1 * c1 * s2 * s2 * s3 * s3 +
               c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3),
      -(ax * (c1 * c2 * s3 + c1 * c3 * s2)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 +
               c1 * c1 * c3 * c3 * s2 * s2 + c1 * c1 * s2 * s2 * s3 * s3 +
               c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3) -
          (az * (c2 * c3 - s2 * s3)) / (c2 * c2 * c3 * c3 + c2 * c2 * s3 * s3 +
                                        c3 * c3 * s2 * s2 + s2 * s2 * s3 * s3) -
          (ay * s1 * (c2 * s3 + c3 * s2)) /
              (c1 * c1 * c2 * c2 * c3 * c3 + c1 * c1 * c2 * c2 * s3 * s3 +
               c1 * c1 * c3 * c3 * s2 * s2 + c1 * c1 * s2 * s2 * s3 * s3 +
               c2 * c2 * c3 * c3 * s1 * s1 + c2 * c2 * s1 * s1 * s3 * s3 +
               c3 * c3 * s1 * s1 * s2 * s2 + s1 * s1 * s2 * s2 * s3 * s3));
  double s4 = sin(t4);
  double c4 = cos(t4);

  double t5 = atan(
      ((oz * (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4)) /
           (c2 * c2 * c3 * c3 * c4 * c4 + c2 * c2 * c3 * c3 * s4 * s4 +
            c2 * c2 * c4 * c4 * s3 * s3 + c2 * c2 * s3 * s3 * s4 * s4 +
            c3 * c3 * c4 * c4 * s2 * s2 + c3 * c3 * s2 * s2 * s4 * s4 +
            c4 * c4 * s2 * s2 * s3 * s3 + s2 * s2 * s3 * s3 * s4 * s4) +
       (oy * (c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4 + c4 * s1 * s2 * s3 -
              c2 * c3 * c4 * s1)) /
           (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 +
            c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
            c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 +
            c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
            c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 +
            c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
            c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 +
            c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
            c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 +
            c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
            c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 +
            c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
            c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 +
            c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
            c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 +
            s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4) +
       (c1 * ox * (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3)) /
           (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 +
            c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
            c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 +
            c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
            c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 +
            c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
            c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 +
            c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
            c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 +
            c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
            c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 +
            c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
            c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 +
            c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
            c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 +
            s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4)) /
      (-(nz * (c2 * c3 * s4 + c2 * c4 * s3 + c3 * c4 * s2 - s2 * s3 * s4)) /
           (c2 * c2 * c3 * c3 * c4 * c4 + c2 * c2 * c3 * c3 * s4 * s4 +
            c2 * c2 * c4 * c4 * s3 * s3 + c2 * c2 * s3 * s3 * s4 * s4 +
            c3 * c3 * c4 * c4 * s2 * s2 + c3 * c3 * s2 * s2 * s4 * s4 +
            c4 * c4 * s2 * s2 * s3 * s3 + s2 * s2 * s3 * s3 * s4 * s4) -
       (ny * (c2 * s1 * s3 * s4 + c3 * s1 * s2 * s4 + c4 * s1 * s2 * s3 -
              c2 * c3 * c4 * s1)) /
           (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 +
            c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
            c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 +
            c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
            c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 +
            c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
            c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 +
            c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
            c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 +
            c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
            c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 +
            c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
            c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 +
            c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
            c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 +
            s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4) -
       (c1 * nx * (c2 * s3 * s4 - c2 * c3 * c4 + c3 * s2 * s4 + c4 * s2 * s3)) /
           (c1 * c1 * c2 * c2 * c3 * c3 * c4 * c4 +
            c1 * c1 * c2 * c2 * c3 * c3 * s4 * s4 +
            c1 * c1 * c2 * c2 * c4 * c4 * s3 * s3 +
            c1 * c1 * c2 * c2 * s3 * s3 * s4 * s4 +
            c1 * c1 * c3 * c3 * c4 * c4 * s2 * s2 +
            c1 * c1 * c3 * c3 * s2 * s2 * s4 * s4 +
            c1 * c1 * c4 * c4 * s2 * s2 * s3 * s3 +
            c1 * c1 * s2 * s2 * s3 * s3 * s4 * s4 +
            c2 * c2 * c3 * c3 * c4 * c4 * s1 * s1 +
            c2 * c2 * c3 * c3 * s1 * s1 * s4 * s4 +
            c2 * c2 * c4 * c4 * s1 * s1 * s3 * s3 +
            c2 * c2 * s1 * s1 * s3 * s3 * s4 * s4 +
            c3 * c3 * c4 * c4 * s1 * s1 * s2 * s2 +
            c3 * c3 * s1 * s1 * s2 * s2 * s4 * s4 +
            c4 * c4 * s1 * s1 * s2 * s2 * s3 * s3 +
            s1 * s1 * s2 * s2 * s3 * s3 * s4 * s4)));

  // Check joints' limit
  if (-M_PI < t1 && t1 < M_PI) {
    jointValueVec_[0] = t1;
  } else {
    ROS_ERROR_NAMED("xarm_ik", "The value of joint 1 is out of bounds");
    return false;
  }

  if (-M_PI < t2 && t2 < 0) {
    jointValueVec_[1] = t2 + M_PI / 2;
  } else {
    ROS_ERROR_NAMED("xarm_ik", "The value of joint 2 is out of bounds");
    return false;
  }

  if (-2 < t3 && t3 < 2) {
    jointValueVec_[2] = t3;
  } else {
    ROS_ERROR_NAMED("xarm_ik", "The value of joint 3 is out of bounds");
    return false;
  }

  if (-2 - M_PI < t4 && t4 < 2) {
    jointValueVec_[3] = t4 + M_PI / 2;
  } else {
    ROS_ERROR_NAMED("xarm_ik", "The value of joint 4 is out of bounds");
    return false;
  }

  if (-M_PI < t5 && t5 < M_PI) {
    jointValueVec_[4] = (abs(t5) < FLT_EPSILON) ? 0 : t5;
  } else {
    ROS_ERROR_NAMED("xarm_ik", "The value of joint 5 is out of bounds");
    return false;
  }

  return true;
}

bool XArmIk::RevisePose(geometry_msgs::Pose& pose) {
  double roll, pitch, yaw;
  QuaternionToRPY(pose.orientation, roll, pitch, yaw);

  if (pose.position.x != 0) {
    yaw = atan2(pose.position.y, pose.position.x);
    pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    std::cout << "Revise: " << yaw << std::endl;
  } else if (pose.position.y > 0) {
    pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, M_PI / 2);
  } else if (pose.position.y < 0) {
    pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, -M_PI / 2);
  }

  return IsPoseReachable(pose);
}

}  // namespace lobot_ik
