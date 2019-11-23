#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visual_tools("bask_link");
  moveit::planning_interface::MoveGroupInterface move_group("xarm_arm");

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_pick_place", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_pick_place", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Gripper
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_cmd_action_client("xarm_gripper_command",
                                                                                              true);
  control_msgs::GripperCommandGoal goal;
  ROS_INFO_NAMED("xarm_pick_place", "Waiting for GripperCommandAction server to start");
  gripper_cmd_action_client.waitForServer(ros::Duration(3));
  ROS_INFO_NAMED("xarm_pick_place", "GripperCommandAction server connected");

  // Plan & move the arm
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.22;
  target_pose.position.y = 0;
  target_pose.position.z = 0.04;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  move_group.setJointValueTarget(target_pose);
  bool success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 as trajectory line");

  // Close the gripper
  goal.command.position = 0.025;
  gripper_cmd_action_client.sendGoal(goal);

  bool result = gripper_cmd_action_client.waitForResult(ros::Duration(3));
  if (result && gripper_cmd_action_client.getState() == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
  {
    // Plan & move the arm
    target_pose.position.x = 0;
    target_pose.position.y = 0.18;
    target_pose.position.z = 0.05;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 4, 0);
    move_group.setJointValueTarget(target_pose);
    success = (move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 as trajectory line");

    // Open the gripper
    goal.command.position = 0.0;
    gripper_cmd_action_client.sendGoal(goal);
  }

  return 0;
}
