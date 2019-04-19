#ifndef XARM_KINEMATICS_PLUGIN_H
#define XARM_KINEMATICS_PLUGIN_H

#include <moveit/kinematics_base/kinematics_base.h>
#include <ros/ros.h>
#include <tf/tf.h>

#define JOINT_NUM 5

namespace xarm_kinematics_plugin {

class XArmKinematicsPlugin : public kinematics::KinematicsBase {
 public:
  XArmKinematicsPlugin() : initialized_(false) {}
  ~XArmKinematicsPlugin();

  bool getPositionIK(const geometry_msgs::Pose& ik_pose,
                     const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution,
                     moveit_msgs::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options =
                         kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose,
      const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options =
          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose,
      const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits,
      std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options =
          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose,
      const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options =
          kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose,
      const std::vector<double>& ik_seed_state, double timeout,
      const std::vector<double>& consistency_limits,
      std::vector<double>& solution, const IKCallbackFn& solution_callback,
      moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options =
          kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names,
                     const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::Pose>& poses) const override;

  bool initialize(const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames,
                  double search_discretization) override;

  const std::vector<std::string>& getJointNames() const override;

  const std::vector<std::string>& getLinkNames() const override;

 protected:
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose,
                        const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback,
                        moveit_msgs::MoveItErrorCodes& error_code,
                        const std::vector<double>& consistency_limits,
                        const kinematics::KinematicsQueryOptions& options =
                            kinematics::KinematicsQueryOptions()) const;

 private:
  bool initialized_;  // Internal variable that indicates whether solver is
                      // configured and ready
  unsigned int dimension_;  // Dimension of the group

  bool isPoseReachable(const geometry_msgs::Pose& pose) const;
  bool revisePose(geometry_msgs::Pose& pose) const;
  bool solveIK(const geometry_msgs::Point& p, const tf::Matrix3x3& r,
               std::vector<double>& solution) const;
};

}  // namespace xarm_kinematics_plugin

#endif  // XARM_KINEMATICS_PLUGIN_H
