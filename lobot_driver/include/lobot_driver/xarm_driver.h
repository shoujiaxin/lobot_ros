#ifndef XARM_DRIVER_H
#define XARM_DRIVER_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <memory>
#include "hid/myhid.hpp"

class XArmDriver : public hardware_interface::RobotHW {
 public:
  XArmDriver(const unsigned short vendorId, const unsigned short productId);
  void init();
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

 private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[5];
  double pos[5];
  double vel[5];
  double eff[5];
  std::shared_ptr<MyHid> myHid;
};

#endif  // XARM_DRIVER_H
