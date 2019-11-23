#include "xarm_driver/xarm_driver.h"

namespace lobot_hardware_interface
{
XarmDriver::XarmDriver()
{
  my_hid_ = MyHid(0x0483, 0x5750);
  try
  {
    my_hid_.open();
  }
  catch (std::runtime_error err)
  {
    ROS_ERROR_NAMED("xarm_hardware_interface", err.what());
    ros::shutdown();
  }

  ROS_INFO_NAMED("xarm_hardware_interface", "xArm control board connected");

  init();
}

XarmDriver::~XarmDriver()
{
  my_hid_.close();
}

void XarmDriver::init()
{
  spinServos({ 1, 2, 3, 4, 5, 6 }, { 200, 500, 500, 500, 500, 500 });
  ros::Duration(2).sleep();
  ROS_INFO_NAMED("xarm_hardware_interface", "Arm joints initialized");
}

}  // namespace lobot_hardware_interface
