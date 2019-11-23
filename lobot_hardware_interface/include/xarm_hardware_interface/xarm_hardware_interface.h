#ifndef XARM_HARDWARE_INTERFACE_H
#define XARM_HARDWARE_INTERFACE_H

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <array>

#include "xarm_driver/xarm_driver.h"

namespace lobot_hardware_interface
{
class XarmHardwareInterface : public hardware_interface::RobotHW
{
public:
  XarmHardwareInterface(ros::NodeHandle& nh);

  ~XarmHardwareInterface();

  void read(const ros::Time& time, const ros::Duration& period) override;

  void update(const ros::TimerEvent& e);

  void write(const ros::Time& time, const ros::Duration& period) override;

private:
  ros::NodeHandle nh_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  controller_manager::ControllerManager controller_manager_;

  // Shared memory
  std::array<double, SERVO_NUM> joint_positions_{ 0 };
  std::array<double, SERVO_NUM> joint_velocities_{ 0 };
  std::array<double, SERVO_NUM> joint_efforts_{ 0 };
  std::array<double, SERVO_NUM> joint_position_cmds_{ 0 };

  // Driver
  XarmDriver xarm_driver_;

  ros::Timer timer;

  // Gripper control
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> gripper_cmd_action_server_;
  control_msgs::GripperCommandFeedback gripper_cmd_feedback_;
  control_msgs::GripperCommandResult gripper_cmd_result_;

  void gripperCmdCallback(const control_msgs::GripperCommandGoalConstPtr& goal);
};

// Get joints' current angles
inline void XarmHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  xarm_driver_.getJointStates(joint_positions_);
}

inline void XarmHardwareInterface::update(const ros::TimerEvent& e)
{
  auto current_time = ros::Time::now();
  auto period = ros::Duration(e.current_real - e.last_real);

  read(current_time, period);
  controller_manager_.update(current_time, period);
  write(current_time, period);
}

// Send commands to control board
inline void XarmHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  xarm_driver_.execute(joint_position_cmds_, period);
}

}  // namespace lobot_hardware_interface

#endif  // XARM_HARDWARE_INTERFACE_H
