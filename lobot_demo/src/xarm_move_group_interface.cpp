#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_move_group_interface");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  static const std::string PLANNING_GROUP = "xarm_arm";
  moveit::planning_interface::MoveGroupInterface moveGroup(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      moveGroup.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");
  visualTools.deleteAllMarkers();
  visualTools.loadRemoteControl();
  Eigen::Isometry3d textPose = Eigen::Isometry3d::Identity();
  textPose.translation().z() = 1.75;
  visualTools.publishText(textPose, "MoveGroupInterface Demo", rvt::WHITE,
                          rvt::XLARGE);
  visualTools.trigger();

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("tutorial", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());
  // Print list of all groups
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(moveGroup.getJointModelGroupNames().begin(),
            moveGroup.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to start the demo");

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
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visualTools.publishAxisLabeled(targetPose1, "pose1");
  visualTools.publishText(textPose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visualTools.publishTrajectoryLine(myPlan.trajectory_, joint_model_group);
  visualTools.trigger();
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Move to the goal
  moveGroup.move();

  return 0;
}
