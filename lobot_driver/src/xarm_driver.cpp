#include "lobot_driver/xarm_driver.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

XArmDriver::XArmDriver(const unsigned short vendorId,
                       const unsigned short productId, ros::NodeHandle& nh)
    : as_(nh, "xarm/xarm_joint_controller/follow_joint_trajectory",
          boost::bind(&XArmDriver::write, this, _1), false),
      jntStatePub_(
          nh_.advertise<sensor_msgs::JointState>("xarm/joint_states", 1)),
      myHid(std::make_shared<MyHid>(vendorId, productId)) {
  // start follow_joint_trajectory action server
  as_.start();

  // connect and register the joint state interface
  hardware_interface::JointStateHandle stateHandle1("xarm/arm_joint1", &pos[0],
                                                    &vel[0], &eff[0]);
  jntStateInterface_.registerHandle(stateHandle1);
  hardware_interface::JointStateHandle stateHandle2("xarm/arm_joint2", &pos[1],
                                                    &vel[1], &eff[1]);
  jntStateInterface_.registerHandle(stateHandle2);
  hardware_interface::JointStateHandle stateHandle3("xarm/arm_joint3", &pos[2],
                                                    &vel[2], &eff[2]);
  jntStateInterface_.registerHandle(stateHandle3);
  hardware_interface::JointStateHandle stateHandle4("xarm/arm_joint4", &pos[3],
                                                    &vel[3], &eff[3]);
  jntStateInterface_.registerHandle(stateHandle4);
  hardware_interface::JointStateHandle stateHandle5("xarm/arm_joint5", &pos[4],
                                                    &vel[4], &eff[4]);
  jntStateInterface_.registerHandle(stateHandle5);
  registerInterface(&jntStateInterface_);

  // connect and register the joint position interface
  hardware_interface::JointHandle posHandle1(
      jntStateInterface_.getHandle("xarm/arm_joint1"), &cmd[0]);
  jntPosInterface_.registerHandle(posHandle1);
  hardware_interface::JointHandle posHandle2(
      jntStateInterface_.getHandle("xarm/arm_joint2"), &cmd[1]);
  jntPosInterface_.registerHandle(posHandle2);
  hardware_interface::JointHandle posHandle3(
      jntStateInterface_.getHandle("xarm/arm_joint3"), &cmd[1]);
  jntPosInterface_.registerHandle(posHandle3);
  hardware_interface::JointHandle posHandle4(
      jntStateInterface_.getHandle("xarm/arm_joint4"), &cmd[1]);
  jntPosInterface_.registerHandle(posHandle4);
  hardware_interface::JointHandle posHandle5(
      jntStateInterface_.getHandle("xarm/arm_joint5"), &cmd[1]);
  jntPosInterface_.registerHandle(posHandle5);
  registerInterface(&jntPosInterface_);

  // connect the control board via USB-HID
  if (myHid->Open() != 0) {
    return;
  }
  std::clog << "[INFO] [" << ros::Time::now()
            << "]: xArm control board connected!" << std::endl;

  // initiate all joints
  SpinServos({1, 2, 3, 4, 5, 6}, {500, 500, 500, 500, 500, 500}, 1800);
  ros::Duration(2).sleep();
  std::clog << "[INFO] [" << ros::Time::now() << "]: Arm joints initialized!"
            << std::endl;
}

XArmDriver::~XArmDriver() { myHid->Close(); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_driver");

  ros::NodeHandle nh;
  XArmDriver xArmDriver(0x0483, 0x5750, nh);

  ros::spin();

  return 0;
}
