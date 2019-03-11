#ifndef XARM_DRIVER_H
#define XARM_DRIVER_H

#include <ros/console.h>
#include <ros/ros.h>
#include <array>
#include <initializer_list>
#include <memory>
#include <vector>

#include "hid/myhid.hpp"

#define PI 3.1415926

#define JOINT_NUM 6

#define CMD_MULT_SERVO_SPIN 3
#define CMD_MULT_SERVO_POS_READ 21

namespace lobot_hardware_interface {

class XArmDriver {
 public:
  XArmDriver();
  ~XArmDriver();
  std::shared_ptr<std::array<double, JOINT_NUM>> GetJointState();
  void Init();
  void SpinServos(const std::initializer_list<unsigned>& idList,
                  const std::initializer_list<int>& posList,
                  const unsigned period = 2000);

 private:
  MyHid myHid_;

  std::array<int, JOINT_NUM> currPosArray;

  void GetCurrentPosition();
};

}  // namespace lobot_hardware_interface

lobot_hardware_interface::XArmDriver::XArmDriver() {
  myHid_ = MyHid(0x0483, 0x5750);
  try {
    myHid_.Open();
  } catch (std::runtime_error err) {
    ROS_ERROR(err.what());
    ros::shutdown();
  }

  ROS_INFO("xArm control board connected!");

  Init();
}

lobot_hardware_interface::XArmDriver::~XArmDriver() { myHid_.Close(); }

inline std::shared_ptr<std::array<double, JOINT_NUM>>
lobot_hardware_interface::XArmDriver::GetJointState() {
  std::array<double, JOINT_NUM> currAngleArray;

  GetCurrentPosition();

  // State of arm joints
  for (std::size_t i = 1; i != JOINT_NUM; ++i) {
    currAngleArray.at(JOINT_NUM - 1 - i) =
        (i == 2 || i == 3) ? ((500 - currPosArray.at(i)) * PI / 750)
                           : ((currPosArray.at(i) - 500) * PI / 750);
  }

  // State of the gripper joint, need to be mapped from angle to distance
  currAngleArray.at(5) = 0;

  return std::make_shared<std::array<double, JOINT_NUM>>(currAngleArray);
}

inline void lobot_hardware_interface::XArmDriver::GetCurrentPosition() {
  myHid_.MakeAndSendCmd(CMD_MULT_SERVO_POS_READ, {JOINT_NUM, 1, 2, 3, 4, 5, 6});
  auto recvDataVecPtr = myHid_.Read(21);

  if (recvDataVecPtr->size() != 0 &&
      recvDataVecPtr->at(0) == CMD_MULT_SERVO_POS_READ &&
      recvDataVecPtr->at(1) == JOINT_NUM) {
    auto currPos = currPosArray.begin();
    decltype(recvDataVecPtr->size()) i = 0;
    while (currPos != currPosArray.end()) {
      *currPos = static_cast<int>(recvDataVecPtr->at(3 * i + 3)) +
                 static_cast<int>(recvDataVecPtr->at(3 * i + 4) << 8);
      ++currPos;
      ++i;
    }
  }
}

void lobot_hardware_interface::XArmDriver::Init() {
  SpinServos({1, 2, 3, 4, 5, 6}, {500, 500, 500, 500, 500, 500});
  ros::Duration(2).sleep();
  ROS_INFO("Arm joints initialized!");
}

inline void lobot_hardware_interface::XArmDriver::SpinServos(
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

  myHid_.MakeAndSendCmd(CMD_MULT_SERVO_SPIN, argv);
}

#endif  // XARM_DRIVER_H
