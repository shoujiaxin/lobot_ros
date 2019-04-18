#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_move_group_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");
  moveit::planning_interface::MoveGroupInterface moveGroup("xarm_arm");

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_move_group_interface", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_move_group_interface", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());

  // Plan & move
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  geometry_msgs::Pose targetPose1;
  targetPose1.position.x = 0.1;
  targetPose1.position.y = 0;
  targetPose1.position.z = 0.15;
  targetPose1.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(M_PI / 6, M_PI / 2, 0);
  moveGroup.setJointValueTarget(targetPose1);
  bool success = (moveGroup.asyncMove() ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_interface",
                 "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_interface",
                 "Visualizing plan 1 as trajectory line");

  // Plan & move
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 2");
  geometry_msgs::Pose targetPose2;
  targetPose2.position.x = 0.25;
  targetPose2.position.y = 0;
  targetPose2.position.z = 0.15;
  moveGroup.setJointValueTarget(targetPose2, "gripper_link");
  success = (moveGroup.asyncMove() ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_move_group_interface",
                 "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_move_group_interface",
                 "Visualizing plan 1 as trajectory line");

  return 0;
}
