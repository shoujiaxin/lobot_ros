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

namespace lobot_hardware_interface {

class XArmHardwareInterface : public hardware_interface::RobotHW {
 public:
  XArmHardwareInterface(ros::NodeHandle& nh);
  ~XArmHardwareInterface();

  void read(const ros::Time& time, const ros::Duration& period) override;
  void update(const ros::TimerEvent& e);
  void write(const ros::Time& time, const ros::Duration& period) override;

 private:
  ros::NodeHandle nh_;

  // Interfaces
  hardware_interface::JointStateInterface jointStateInterface_;
  hardware_interface::PositionJointInterface positionJointInterface_;

  controller_manager::ControllerManager controllerManager_;

  // Shared memory
  std::array<double, SERVO_NUM> jointPosition_{0};
  std::array<double, SERVO_NUM> jointVelocity_{0};
  std::array<double, SERVO_NUM> jointEffort_{0};
  std::array<double, SERVO_NUM> jointPositionCmd_{0};

  // Driver
  XArmDriver xArmDriver_;

  ros::Timer timer;

  // Gripper control
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction>
      gripperCmdServer_;
  control_msgs::GripperCommandActionFeedback gripperCmdFeedback_;
  control_msgs::GripperCommandActionResult gripperCmdResult_;

  void GripperCmdCallback(const control_msgs::GripperCommandGoalConstPtr& goal);
};

// Get joints' current angles
inline void XArmHardwareInterface::read(const ros::Time& time,
                                        const ros::Duration& period) {
  jointPosition_ = *xArmDriver_.GetJointState();
}

inline void XArmHardwareInterface::update(const ros::TimerEvent& e) {
  auto currTime = ros::Time::now();
  auto period = ros::Duration(e.current_real - e.last_real);

  read(currTime, period);
  controllerManager_.update(currTime, period);
  write(currTime, period);
}

// Send commands to control board
inline void XArmHardwareInterface::write(const ros::Time& time,
                                         const ros::Duration& period) {
  xArmDriver_.Execute(jointPositionCmd_, period);
}

inline void XArmHardwareInterface::GripperCmdCallback(
    const control_msgs::GripperCommandGoalConstPtr& goal) {
  jointPositionCmd_[5] = goal->command.position;
  gripperCmdFeedback_.feedback.position = jointPosition_[5];

  if (abs(goal->command.position - jointPosition_[5]) < 0.01) {
    gripperCmdResult_.result.reached_goal = true;
  }
}

}  // namespace lobot_hardware_interface

#endif  // XARM_HARDWARE_INTERFACE_H
