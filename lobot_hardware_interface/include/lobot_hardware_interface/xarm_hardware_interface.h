#ifndef XARM_HARDWARE_INTERFACE_H
#define XARM_HARDWARE_INTERFACE_H

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <array>

#include "xarm_driver/xarm_driver.hpp"

#define JOINT_NUM 5

namespace lobot_hardware_interface {

class XArmHardwareInterface : public hardware_interface::RobotHW {
 public:
  XArmHardwareInterface();
  ~XArmHardwareInterface();

  void Update();

 private:
  ros::NodeHandle nh_;

  // Interfaces
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;

  boost::shared_ptr<controller_manager::ControllerManager> controllerManager_;

  // Shared memory
  std::array<double, JOINT_NUM> jointPosition_;
  std::array<double, JOINT_NUM> jointVelocity_;
  std::array<double, JOINT_NUM> jointEffort_;
  std::array<double, JOINT_NUM> jointPositionCmd_;

  // Driver
  XArmDriver xArmDriver_;

  void Read();
  void Write();
};

}  // namespace lobot_hardware_interface

// Get joints' current position
inline void lobot_hardware_interface::XArmHardwareInterface::Read() {}

// Send commands to control board
inline void lobot_hardware_interface::XArmHardwareInterface::Write() {}

inline void lobot_hardware_interface::XArmHardwareInterface::Update() {}

#endif  // XARM_HARDWARE_INTERFACE_H
