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

  // Connect and register the joint state & position interface
  for (int i = 0; i != JOINT_NUM; ++i) {
    hardware_interface::JointStateHandle jointStateHandle(
        jointName[i], &jointPosition_[i], &jointVelocity_[i], &jointEffort_[i]);
    jointStateInterface_.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointPosHandle(jointStateHandle,
                                                   &jointPositionCmd_[i]);
    positionJointInterface_.registerHandle(jointPosHandle);
  }
  // Connect and register the gripper state interface
  hardware_interface::JointStateHandle jointStateHandle(
      jointName[GRIPPER_ID], &jointPosition_[GRIPPER_ID],
      &jointVelocity_[GRIPPER_ID], &jointEffort_[GRIPPER_ID]);
  jointStateInterface_.registerHandle(jointStateHandle);

  registerInterface(&jointStateInterface_);
  registerInterface(&positionJointInterface_);

  timer = nh.createTimer(
      ros::Duration(0.1),
      &lobot_hardware_interface::XArmHardwareInterface::update, this);

  gripperCmdServer_.start();
}

XArmHardwareInterface::~XArmHardwareInterface() {}

void XArmHardwareInterface::GripperCmdCallback(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  // Command is the distance of the gripper form its initial position
  jointPositionCmd_[GRIPPER_ID] = goal->command.position;
  // gripperCmdFeedback_.feedback.position = jointPosition_[5];

  // if (abs(goal->command.position - jointPosition_[5]) < 0.01) {
  //   gripperCmdResult_.result.reached_goal = true;
  // }
}

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
