#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");
  moveit::planning_interface::MoveGroupInterface moveGroup("xarm_arm");

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_pick_place", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_pick_place", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());

  // Gripper
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction>
      gripperCmdAC("xarm_gripper_command", true);
  control_msgs::GripperCommandGoal goal;
  ROS_INFO_NAMED("xarm_pick_place",
                 "Waiting for GripperCommandAction server to start");
  gripperCmdAC.waitForServer(ros::Duration(3));
  ROS_INFO_NAMED("xarm_pick_place", "GripperCommandAction server connected");

  // Plan & move the arm
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start plan to target pose 1");
  geometry_msgs::Pose targetPose1;
  targetPose1.position.x = 0.22;
  targetPose1.position.y = 0;
  targetPose1.position.z = 0.04;
  targetPose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  moveGroup.setJointValueTarget(targetPose1);
  bool success = (moveGroup.move() ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");
  ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 as trajectory line");

  // Close the gripper
  goal.command.position = 0.025;
  gripperCmdAC.sendGoal(goal);

  bool result = gripperCmdAC.waitForResult(ros::Duration(3));
  if (result && gripperCmdAC.getState() ==
                    actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
    // Plan & move the arm
    geometry_msgs::Pose targetPose2;
    targetPose2.position.x = 0;
    targetPose2.position.y = 0.18;
    targetPose2.position.z = 0.05;
    targetPose2.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 4, 0);
    moveGroup.setJointValueTarget(targetPose2);
    success = (moveGroup.move() ==
               moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 (pose goal) %s",
                   success ? "" : "FAILED");
    ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 as trajectory line");

    // Open the gripper
    goal.command.position = 0.0;
    gripperCmdAC.sendGoal(goal);
  }

  return 0;
}
