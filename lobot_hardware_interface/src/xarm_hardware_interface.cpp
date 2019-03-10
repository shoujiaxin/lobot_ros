#include "lobot_hardware_interface/xarm_hardware_interface.h"

using namespace lobot_hardware_interface;

XArmHardwareInterface::XArmHardwareInterface() {}

XArmHardwareInterface::~XArmHardwareInterface() {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "example");
  XArmHardwareInterface temp;

  ros::spin();

  return 0;
}
