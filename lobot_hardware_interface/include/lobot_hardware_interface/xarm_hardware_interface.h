#ifndef XARM_HARDWARE_INTERFACE_H
#define XARM_HARDWARE_INTERFACE_H

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <array>

#include "xarm_driver/xarm_driver.hpp"

namespace lobot_hardware_interface {

class XArmHardwareInterface : public hardware_interface::RobotHW {
 public:
  XArmHardwareInterface(ros::NodeHandle& nh);
  ~XArmHardwareInterface();

  void Update(const ros::TimerEvent& e);

 private:
  ros::NodeHandle nh_;

  // Interfaces
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;

  controller_manager::ControllerManager controllerManager_;

  // Shared memory
  std::array<double, JOINT_NUM> jointPosition_{0};
  std::array<double, JOINT_NUM> jointVelocity_{0};
  std::array<double, JOINT_NUM> jointEffort_{0};
  std::array<double, JOINT_NUM> jointPositionCmd_{0};

  // Driver
  XArmDriver xArmDriver_;

  ros::Timer timer;

  void Read();
  void Write();
};

}  // namespace lobot_hardware_interface

// Get joints' current angles
inline void lobot_hardware_interface::XArmHardwareInterface::Read() {
  jointPosition_ = *xArmDriver_.GetJointState();
}

// Send commands to control board
inline void lobot_hardware_interface::XArmHardwareInterface::Write() {}

inline void lobot_hardware_interface::XArmHardwareInterface::Update(
    const ros::TimerEvent& e) {
  auto period = ros::Duration(e.current_real - e.last_real);
  Read();
  controllerManager_.update(ros::Time::now(), period);
  Write();
}

#endif  // XARM_HARDWARE_INTERFACE_H
