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

  void read(const ros::Time& time, const ros::Duration& period);
  void update(const ros::TimerEvent& e);
  void write(const ros::Time& time, const ros::Duration& period);

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
};

}  // namespace lobot_hardware_interface

// Get joints' current angles
inline void lobot_hardware_interface::XArmHardwareInterface::read(
    const ros::Time& time, const ros::Duration& period) {
  jointPosition_ = *xArmDriver_.GetJointState();
}

inline void lobot_hardware_interface::XArmHardwareInterface::update(
    const ros::TimerEvent& e) {
  auto currTime = ros::Time::now();
  auto period = ros::Duration(e.current_real - e.last_real);

  read(currTime, period);
  controllerManager_.update(currTime, period);
  write(currTime, period);
}

// Send commands to control board
inline void lobot_hardware_interface::XArmHardwareInterface::write(
    const ros::Time& time, const ros::Duration& period) {
  xArmDriver_.Execute(jointPositionCmd_, period);
}

#endif  // XARM_HARDWARE_INTERFACE_H
