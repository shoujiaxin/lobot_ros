#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <tf/tf.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_trajectory_plan");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  spinner.start();

  moveit_visual_tools::MoveItVisualTools visual_tools("bask_link");
  moveit::planning_interface::MoveGroupInterface move_group("xarm_arm");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  std::vector<geometry_msgs::Pose> way_points;

  // Print the names of reference frame and end-effector link
  ROS_INFO_NAMED("xarm_trajectory_plan", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("xarm_trajectory_plan", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Move to start pose
  geometry_msgs::Pose target_pose;
  target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, M_PI / 6, 0);

  // Compute path
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to start trajectory plan");
  double center_x = 0.13;
  double center_z = 0.16;
  double radius = 0.02;
  for (double th = 0; th < M_PI_2; th += 0.04)
  {
    target_pose.position.x = center_x + radius * cos(th);
    target_pose.position.z = center_z + radius * sin(th);
    way_points.push_back(target_pose);
  }
  move_group.setJointValueTarget(way_points[0]);
  move_group.move();
  auto fraction = move_group.computeCartesianPath(way_points, 0.005, 1, plan.trajectory_);
  std::cout << fraction << std::endl;
  for (const auto& point : way_points)
  {
    std::cout << point.position.x << " " << point.position.y << " " << point.position.z << std::endl;
  }

  auto point_it = plan.trajectory_.joint_trajectory.points.begin();
  while (point_it != plan.trajectory_.joint_trajectory.points.end())
  {
    if (isnan(point_it->positions[0]) || isnan(point_it->positions[1]) || point_it->positions[4] != 0)
    {
      point_it = plan.trajectory_.joint_trajectory.points.erase(point_it);
      continue;
    }
    ++point_it;
  }
  for (const auto& point : plan.trajectory_.joint_trajectory.points)
  {
    std::cout << point.positions[0] << "  " << point.positions[1] - M_PI / 2 << "  " << point.positions[2] << "  "
              << point.positions[3] - M_PI / 2 << "  " << point.positions[4] << std::endl;
  }

  // Plan
  bool success = (move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // move_group.execute(plan);
  ROS_INFO_NAMED("xarm_pick_place", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  return 0;
}
