#ifndef XARM_DRIVER_H
#define XARM_DRIVER_H

#include <actionlib/server/action_server.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <vector>

#include "hid/myhid.hpp"

#define JOINT_NUM 5  // number of joints
#define CMD_MULT_SERVO_SPIN 3
#define CMD_MULT_SERVO_POS_READ 21

class XArmDriver : public hardware_interface::RobotHW {
 public:
  XArmDriver(const unsigned short vendorId, const unsigned short productId,
             ros::NodeHandle& nh);
  ~XArmDriver();

 private:
  ros::NodeHandle nh_;

  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> sgh_;
  control_msgs::FollowJointTrajectoryResult result_;

  hardware_interface::JointStateInterface jntStateInterface_;
  hardware_interface::PositionJointInterface jntPosInterface_;

  ros::Publisher jntStatePub_;  // joint state publisher

  double cmd[JOINT_NUM];
  double pos[JOINT_NUM];
  double vel[JOINT_NUM];
  double eff[JOINT_NUM];

  std::shared_ptr<MyHid> myHid;

  void SpinServos(const std::initializer_list<unsigned>& idList,
                  const std::initializer_list<int>& posList,
                  const unsigned period = 20);

  void read();
  void write(
      actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction>
          sgh);
};

inline void XArmDriver::SpinServos(
    const std::initializer_list<unsigned>& idList,
    const std::initializer_list<int>& posList, const unsigned period) {
  auto idListSize = idList.size();
  auto posListSize = posList.size();
  if (idListSize != posListSize) {
    return;
  }

  unsigned per = (period > 5000) ? 5000 : period;
  std::vector<unsigned> argv;
  argv.push_back(static_cast<unsigned>(idListSize));
  argv.push_back(per & 0xFF);
  argv.push_back((per >> 8) & 0xFF);
  auto id = idList.begin();
  auto position = posList.begin();
  while (id != idList.end()) {
    argv.push_back(*id);
    argv.push_back(*position & 0xFF);
    argv.push_back((*position >> 8) & 0xFF);
    ++id;
    ++position;
  }

  myHid->MakeAndSendCmd(CMD_MULT_SERVO_SPIN, argv);
}

// read joint state
inline void XArmDriver::read() {
  std::clog << "XArmDriver::read() called!" << std::endl;
}

// send position command
inline void XArmDriver::write(
    actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction>
        sgh) {
  std::clog << "XArmDriver::write() called!" << std::endl;
}

#endif  // XARM_DRIVER_H
