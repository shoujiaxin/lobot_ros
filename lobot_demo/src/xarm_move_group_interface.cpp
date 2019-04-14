#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_move_group_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  static const std::string PLANNING_GROUP = "xarm_arm";
  moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);

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
  geometry_msgs::Pose targetPose1;
  targetPose1.orientation.w = 1;
  targetPose1.orientation.x = 0;
  targetPose1.orientation.y = 0;
  targetPose1.orientation.z = 0;
  targetPose1.position.x = 0.2;
  targetPose1.position.y = 0;
  targetPose1.position.z = 0.15;
  moveGroup.setApproximateJointValueTarget(targetPose1, "gripper_link");
  moveit::planning_interface::MoveGroupInterface::Plan myPlan;
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

  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 2");

  geometry_msgs::Pose targetPose2;
  targetPose2.orientation.w = 1;
  targetPose2.orientation.x = 0;
  targetPose2.orientation.y = 0;
  targetPose2.orientation.z = 0;
  targetPose2.position.x = 0.25;
  targetPose2.position.y = 0;
  targetPose2.position.z = 0.15;
  moveGroup.setApproximateJointValueTarget(targetPose2, "gripper_link");
  // Print result
  success = (moveGroup.asyncMove() ==
             moveit::planning_interface::MoveItErrorCode::SUCCESS);

  return 0;
}
