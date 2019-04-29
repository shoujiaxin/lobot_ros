#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/console.h>
#include <ros/ros.h>

#include "xarm_ik/xarm_ik.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_pick_place_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  ROS_WARN_NAMED(
      "xarm_move_group_demo",
      "Package lobot_ik is deprecated, pleause use lobot_kinematics instead");

  // Arm
  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");
  moveit::planning_interface::MoveGroupInterface moveGroup("xarm_arm");
  moveit::planning_interface::MoveGroupInterface::Plan myPlan;
  ROS_INFO_NAMED("xarm_move_group_demo", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_move_group_demo", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());

  lobot_ik::XArmIk xArmIk;

  // Gripper
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripperCmdAC("xarm_gripper_command", true);
  control_msgs::GripperCommandGoal goal;
  ROS_INFO_NAMED("xarm_pick_place_demo",
                 "Waiting for GripperCommandAction server to start");
  gripperCmdAC.waitForServer(ros::Duration(3));
  ROS_INFO_NAMED("xarm_pick_place_demo",
                 "GripperCommandAction server connected");

  // Plan & move the arm
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
  moveGroup.move();

  // Close the gripper
  goal.command.position = 0.025;
  gripperCmdAC.sendGoal(goal);

  // Plan & move the arm
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  geometry_msgs::Pose targetPose2;
  targetPose2.position.x = 0.04;
  targetPose2.position.y = 0.0;
  targetPose2.position.z = 0.1;
  targetPose2.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(0, 3 * M_PI / 4, M_PI / 3);
  xArmIk.SetPoseTarget(targetPose2, moveGroup);
  success = (moveGroup.plan(myPlan) ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_demo", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_demo",
                 "Visualizing plan 1 as trajectory line");
  moveGroup.move();

  // Open the gripper
  goal.command.position = 0.0;
  gripperCmdAC.sendGoal(goal);

  return 0;
}
