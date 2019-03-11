#include <array>
#include <string>

#include "lobot_hardware_interface/xarm_hardware_interface.h"

using namespace lobot_hardware_interface;

XArmHardwareInterface::XArmHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh), controllerManager_(this) {
  // Names of arm joints
  const std::array<std::string, JOINT_NUM> jointName{
      "arm_joint1", "arm_joint2", "arm_joint3",
      "arm_joint4", "arm_joint5", "gripper_joint1"};

  for (int i = 0; i != JOINT_NUM; ++i) {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle jointStateHandle(
        jointName.at(i), &jointPosition_.at(i), &jointVelocity_.at(i),
        &jointEffort_.at(i));
    jointStateInterface_.registerHandle(jointStateHandle);

    // Connect and register the joint position interface
    hardware_interface::JointHandle jointPosHandle(jointStateHandle,
                                                   &jointPositionCmd_.at(i));
    positionJointInterface_.registerHandle(jointPosHandle);
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&positionJointInterface_);

  timer = nh.createTimer(
      ros::Duration(0.1),
      &lobot_hardware_interface::XArmHardwareInterface::Update, this);
}

XArmHardwareInterface::~XArmHardwareInterface() {}

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  XArmHardwareInterface xArm(nh);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
