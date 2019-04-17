#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include "xarm_ik/xarm_ik.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_move_group_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  static const std::string PLANNING_GROUP = "xarm_arm";
  moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);
  lobot_ik::XArmIk xArmIk;
  moveit::planning_interface::MoveGroupInterface::Plan myPlan;

  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_move_group_interface", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_move_group_interface", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());
  // Print list of all groups
  ROS_INFO_NAMED("xarm_move_group_interface", "Available Planning Groups:");
  std::copy(moveGroup.getJointModelGroupNames().begin(),
            moveGroup.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, "\n"));

  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  lobot_ik::XArmPose targetPose1(
      {0.07, 0, 0.1},
      {lobot_ik::XArmQuaternion::AngleType::RPY, 0, 3 * PI / 4, 0});
  xArmIk.SetTargetValue(targetPose1, moveGroup);
  // Print result
  bool success = (moveGroup.plan(myPlan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_interface",
                 "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_interface",
                 "Visualizing plan 1 as trajectory line");

  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start move to target pose 1");
  // Move to the goal
  moveGroup.move();

  ros::waitForShutdown();

  return 0;
}
