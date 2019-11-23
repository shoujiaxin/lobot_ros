#ifndef XARM_KINEMATICS_PLUGIN_H
#define XARM_KINEMATICS_PLUGIN_H

#include <moveit/kinematics_base/kinematics_base.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace xarm_kinematics_plugin
{
#define JOINT_NUM 5

class XarmKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  XarmKinematicsPlugin();

  ~XarmKinematicsPlugin();

  const std::vector<std::string>& getJointNames() const override
  {
    return joint_names_;
  }

  const std::vector<std::string>& getLinkNames() const override
  {
    return link_names_;
  }

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

  bool getPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                  const std::string& base_frame, const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits, std::vector<double>& solution,
      const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;

  bool isPoseReachable(const geometry_msgs::Pose& pose) const;

  void quaternionToRpy(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) const;

  bool revisePose(geometry_msgs::Pose& pose) const;

  bool solveIk(const geometry_msgs::Point& p, tf::Matrix3x3& r, std::vector<double>& solution) const;
};

}  // namespace xarm_kinematics_plugin

#endif  // XARM_KINEMATICS_PLUGIN_H
