#ifndef XARM_DRIVER_H
#define XARM_DRIVER_H

#include <ros/console.h>
#include <ros/ros.h>
#include <hid/myhid.hpp>

namespace lobot_hardware_interface {

class XArmDriver {
 public:
  XArmDriver();
  ~XArmDriver();

 private:
  MyHid myHid_;
};

}  // namespace lobot_hardware_interface

lobot_hardware_interface::XArmDriver::XArmDriver() {
  myHid_ = MyHid(0x0483, 0x5750);
  if (myHid_.Open() != 0) {
    ros::shutdown();
  }

  ROS_INFO("xArm control board connected!");
}

lobot_hardware_interface::XArmDriver::~XArmDriver() {}

#endif  // XARM_DRIVER_H
