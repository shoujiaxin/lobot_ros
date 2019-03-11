#include "lobot_hardware_interface/xarm_hardware_interface.h"

using namespace lobot_hardware_interface;

XArmHardwareInterface::XArmHardwareInterface() {}

XArmHardwareInterface::~XArmHardwareInterface() {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "example");
  XArmHardwareInterface temp;

  while (ros::ok()) {
    temp.Update();
    ros::Duration(1).sleep();
  }

  return 0;
}
