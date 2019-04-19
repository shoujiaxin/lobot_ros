#include <array>
#include <string>

#include "xarm_hardware_interface/xarm_hardware_interface.h"

namespace lobot_hardware_interface {

XArmHardwareInterface::XArmHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh),
      controllerManager_(this),
      gripperCmdServer_(
          nh, "xarm_gripper_command",
          boost::bind(&XArmHardwareInterface::GripperCmdCallback, this, _1),
          false) {
  // Names of arm joints
  const std::array<std::string, SERVO_NUM> jointName{
      "arm_joint1", "arm_joint2", "arm_joint3",
      "arm_joint4", "arm_joint5", "gripper_joint1"};

  for (int i = 0; i != SERVO_NUM; ++i) {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle jointStateHandle(
        jointName[i], &jointPosition_[i], &jointVelocity_[i], &jointEffort_[i]);
    jointStateInterface_.registerHandle(jointStateHandle);

    // Connect and register the joint position interface
    hardware_interface::JointHandle jointPosHandle(jointStateHandle,
                                                   &jointPositionCmd_[i]);
    positionJointInterface_.registerHandle(jointPosHandle);
  }

  registerInterface(&jointStateInterface_);
  registerInterface(&positionJointInterface_);

  timer = nh.createTimer(
      ros::Duration(0.1),
      &lobot_hardware_interface::XArmHardwareInterface::update, this);

  gripperCmdServer_.start();
}

XArmHardwareInterface::~XArmHardwareInterface() {}

}  // namespace lobot_hardware_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);

  lobot_hardware_interface::XArmHardwareInterface xArmHWInterface(nh);

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
