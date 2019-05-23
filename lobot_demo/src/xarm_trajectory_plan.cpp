#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "xarm_trajectory_plan");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visualTools("bask_link");
  moveit::planning_interface::MoveGroupInterface moveGroup("xarm_arm");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  std::vector<geometry_msgs::Pose> wayPoints;

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_trajectory_plan", "Planning frame: %s",
                 moveGroup.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_trajectory_plan", "End effector link: %s",
                 moveGroup.getEndEffectorLink().c_str());

  // Move to start pose
  geometry_msgs::Pose targetPose;
  targetPose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 6, 0);

  // Compute path
  visualTools.prompt(
      "Press 'next' in the RvizVisualToolsGui to start trajectory plan");
  double centerX = 0.13;
  double centerZ = 0.16;
  double radius = 0.02;
  for (double th = 0; th < M_PI_2; th += 0.04) {
    targetPose.position.x = centerX + radius * cos(th);
    targetPose.position.z = centerZ + radius * sin(th);
    wayPoints.push_back(targetPose);
  }
  moveGroup.setJointValueTarget(wayPoints[0]);
  moveGroup.move();
  auto fraction =
      moveGroup.computeCartesianPath(wayPoints, 0.005, 1, plan.trajectory_);
  std::cout << fraction << std::endl;
  for (auto w : wayPoints) {
    std::cout << w.position.x << " " << w.position.y << " " << w.position.z
              << std::endl;
  }

  auto pointIt = plan.trajectory_.joint_trajectory.points.begin();
  while (pointIt != plan.trajectory_.joint_trajectory.points.end()) {
    if (isnan(pointIt->positions[0]) || isnan(pointIt->positions[1]) ||
        pointIt->positions[4] != 0) {
      pointIt = plan.trajectory_.joint_trajectory.points.erase(pointIt);
      continue;
    }
    ++pointIt;
  }
  for (auto point : plan.trajectory_.joint_trajectory.points) {
    std::cout << point.positions[0] << "  " << point.positions[1] - M_PI / 2
              << "  " << point.positions[2] << "  "
              << point.positions[3] - M_PI / 2 << "  " << point.positions[4]
              << std::endl;
  }

  // Plan
  bool success = (moveGroup.execute(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // moveGroup.execute(plan);
  ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 (pose goal) %s",
                 success ? "" : "FAILED");

  return 0;
}
