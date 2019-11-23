#include <array>
#include <string>

#include "xarm_hardware_interface/xarm_hardware_interface.h"

namespace lobot_hardware_interface
{
XarmHardwareInterface::XarmHardwareInterface(ros::NodeHandle& nh)
  : nh_(nh)
  , controller_manager_(this)
  , gripper_cmd_action_server_(nh, "xarm_gripper_command",
                               boost::bind(&XarmHardwareInterface::gripperCmdCallback, this, _1), false)
{
  // Names of arm joints
  const std::array<std::string, SERVO_NUM> joint_names{ "arm_joint1", "arm_joint2", "arm_joint3",
                                                        "arm_joint4", "arm_joint5", "gripper_joint1" };

  // Connect and register the joint state & position interface
  for (auto i = 0; i != JOINT_NUM; ++i)
  {
    hardware_interface::JointStateHandle jointStateHandle(joint_names[i], &joint_positions_[i], &joint_velocities_[i],
                                                          &joint_efforts_[i]);
    joint_state_interface_.registerHandle(jointStateHandle);

    hardware_interface::JointHandle jointPosHandle(jointStateHandle, &joint_position_cmds_[i]);
    position_joint_interface_.registerHandle(jointPosHandle);
  }
  // Connect and register the gripper state interface
  hardware_interface::JointStateHandle jointStateHandle(joint_names[GRIPPER_ID], &joint_positions_[GRIPPER_ID],
                                                        &joint_velocities_[GRIPPER_ID], &joint_efforts_[GRIPPER_ID]);
  joint_state_interface_.registerHandle(jointStateHandle);

  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  timer = nh.createTimer(ros::Duration(0.1), &lobot_hardware_interface::XarmHardwareInterface::update, this);

  gripper_cmd_action_server_.start();
}

XarmHardwareInterface::~XarmHardwareInterface()
{
}

void XarmHardwareInterface::gripperCmdCallback(const control_msgs::GripperCommandGoalConstPtr& goal)
{
  int time_to_execute = 3000;  // Wait for 3 seconds
  constexpr unsigned short freq = 50;
  ros::Rate r(freq);

  while (time_to_execute > 0)
  {
    // Check that if preempt is requested by client
    if (gripper_cmd_action_server_.isPreemptRequested() || !ros::ok())
    {
      joint_position_cmds_[GRIPPER_ID] = joint_positions_[5];  // Stop moving
      ROS_INFO_NAMED("xarm_hardware_interface", "%s: Preempted");
      // Set the action state to preempted
      gripper_cmd_action_server_.setPreempted();
      break;
    }

    // Command is the distance of the gripper form its initial position
    joint_position_cmds_[GRIPPER_ID] = goal->command.position;
    // Publish feedback
    gripper_cmd_feedback_.position = joint_positions_[5];
    gripper_cmd_action_server_.publishFeedback(gripper_cmd_feedback_);

    time_to_execute -= 1000 / freq;
    r.sleep();
  }

  if (abs(goal->command.position - joint_positions_[5]) < 0.01)
  {
    gripper_cmd_action_server_.setSucceeded(gripper_cmd_result_);
  }
  else
  {
    gripper_cmd_action_server_.setAborted(gripper_cmd_result_);
  }
}

}  // namespace lobot_hardware_interface

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_hardware_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);

  lobot_hardware_interface::XarmHardwareInterface xarm_hardware_interface(nh);

  spinner.start();
  ros::waitForShutdown();

  return 0;
}
