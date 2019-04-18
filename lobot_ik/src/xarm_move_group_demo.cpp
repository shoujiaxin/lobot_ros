#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include "xarm_ik/xarm_ik.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_move_group_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");
  moveit::planning_interface::MoveGroupInterface moveGroup("xarm_arm");
  moveit::planning_interface::MoveGroupInterface::Plan myPlan;
  ROS_INFO_NAMED("xarm_move_group_demo", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_move_group_demo", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());

  lobot_ik::XArmIk xArmIk;

  // Plan
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  geometry_msgs::Pose targetPose1;
  targetPose1.position.x = 0.04;
  targetPose1.position.y = 0.05;
  targetPose1.position.z = 0.1;
  targetPose1.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(M_PI / 6, 3 * M_PI / 4, M_PI / 3);
  xArmIk.SetPoseTarget(targetPose1, moveGroup);
  bool success = (moveGroup.plan(myPlan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_demo", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_demo",
                 "Visualizing plan 1 as trajectory line");

  // Move
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start move to target pose 1");
  if (success) {
    ROS_INFO_NAMED("xarm_move_group_demo", "Moving...");
    moveGroup.move();
  }

  return 0;
}
