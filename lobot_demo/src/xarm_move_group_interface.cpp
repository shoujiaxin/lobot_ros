#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_move_group_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visual_tools("bask_link");
  moveit::planning_interface::MoveGroupInterface move_group("xarm_arm");

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_move_group_interface", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_move_group_interface", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Plan & move
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.1;
  target_pose.position.y = 0;
  target_pose.position.z = 0.15;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 6, M_PI / 4, 0);
  move_group.setJointValueTarget(target_pose);
  bool success = (move_group.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_interface", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_interface", "Visualizing plan 1 as trajectory line");

  // Plan & move
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to start plan to target pose 2");
  target_pose.position.x = 0.25;
  target_pose.position.y = 0;
  target_pose.position.z = 0.15;
  move_group.setJointValueTarget(target_pose, "gripper_link");
  success = (move_group.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_interface", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_interface", "Visualizing plan 1 as trajectory line");

  return 0;
}
