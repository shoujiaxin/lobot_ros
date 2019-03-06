#include "lobot_driver/xarm_driver.h"
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <iostream>

XArmDriver::XArmDriver(const unsigned short vendorId,
                       const unsigned short productId) {
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_1(
      "/xarm/arm_joint1", &pos[0], &vel[0], &eff[0]);
  jnt_state_interface.registerHandle(state_handle_1);

  hardware_interface::JointStateHandle state_handle_2(
      "/xarm/arm_joint2", &pos[1], &vel[1], &eff[1]);
  jnt_state_interface.registerHandle(state_handle_2);

  hardware_interface::JointStateHandle state_handle_3(
      "/xarm/arm_joint3", &pos[2], &vel[2], &eff[2]);
  jnt_state_interface.registerHandle(state_handle_3);

  hardware_interface::JointStateHandle state_handle_4(
      "/xarm/arm_joint4", &pos[3], &vel[3], &eff[3]);
  jnt_state_interface.registerHandle(state_handle_4);

  hardware_interface::JointStateHandle state_handle_5(
      "/xarm/arm_joint5", &pos[4], &vel[4], &eff[4]);
  jnt_state_interface.registerHandle(state_handle_5);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle pos_handle_1(
      jnt_state_interface.getHandle("/xarm/arm_joint1"), &cmd[0]);
  jnt_pos_interface.registerHandle(pos_handle_1);

  hardware_interface::JointHandle pos_handle_2(
      jnt_state_interface.getHandle("/xarm/arm_joint2"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_2);

  hardware_interface::JointHandle pos_handle_3(
      jnt_state_interface.getHandle("/xarm/arm_joint3"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_3);

  hardware_interface::JointHandle pos_handle_4(
      jnt_state_interface.getHandle("/xarm/arm_joint4"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_4);

  hardware_interface::JointHandle pos_handle_5(
      jnt_state_interface.getHandle("/xarm/arm_joint5"), &cmd[1]);
  jnt_pos_interface.registerHandle(pos_handle_5);

  registerInterface(&jnt_pos_interface);

  myHid = std::make_shared<MyHid>(vendorId, productId);
}

void XArmDriver::init() {
  // connect the control board via USB-HID
  if (myHid->Open() != 0) {
    return;
  }
  std::clog << "xArm control board connected!" << std::endl;

  // initiate all joints
}

void XArmDriver::read(const ros::Time& time, const ros::Duration& period) {}

void XArmDriver::write(const ros::Time& time, const ros::Duration& period) {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_driver");

  XArmDriver xArmDriver(0x0483, 0x5750);
  controller_manager::ControllerManager cm(&xArmDriver);

  xArmDriver.init();
}
