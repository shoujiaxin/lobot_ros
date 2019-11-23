#ifndef XARM_DRIVER_H
#define XARM_DRIVER_H

#include <ros/console.h>
#include <ros/ros.h>
#include <array>
#include <initializer_list>
#include <vector>

#include "hid/myhid.hpp"

namespace lobot_hardware_interface
{
#define SERVO_NUM 6   // Number of servos
#define JOINT_NUM 5   // Number of the arm joints
#define GRIPPER_ID 5  // Index of the gripper joint

#define CMD_MULT_SERVO_SPIN 3
#define CMD_MULT_SERVO_POS_READ 21

class XarmDriver
{
public:
  XarmDriver();

  ~XarmDriver();

  void execute(const std::array<double, SERVO_NUM>& cmd, const ros::Duration& period);

  void getJointState(std::array<double, SERVO_NUM>& joint_state);

protected:
  MyHid my_hid_;
  std::array<int, SERVO_NUM> servo_position_;

private:
  void getCurrentServoPosition();

  void init();

  void spinServos(const std::initializer_list<unsigned>& id_list, const std::initializer_list<int>& position_list,
                  const unsigned period = 2000);
};

inline void XarmDriver::execute(const std::array<double, SERVO_NUM>& cmd, const ros::Duration& period)
{
  std::array<int, SERVO_NUM> position_cmd;

  // Commands for arm joints, convert radians to positions
  for (auto i = 0; i != JOINT_NUM; ++i)
  {
    position_cmd[i] = (i == 1) ? (500 - 750 * cmd[i] / M_PI) : (500 + 750 * cmd[i] / M_PI);
  }

  // Command for gripper joint
  auto gripper_cmd = (0.03 - cmd[GRIPPER_ID]) * 2000;
  position_cmd[GRIPPER_ID] = -0.003073 * gripper_cmd * gripper_cmd * gripper_cmd +
                             0.212188 * gripper_cmd * gripper_cmd - 10.335171 * gripper_cmd + 700.907820;

  // Send commands
  spinServos({ 2, 3, 4, 5, 6 }, { position_cmd[4], position_cmd[3], position_cmd[2], position_cmd[1], position_cmd[0] },
             period.toSec() * 1000);
  spinServos({ 1 }, { position_cmd[GRIPPER_ID] }, 600);
}

inline void XarmDriver::getJointState(std::array<double, SERVO_NUM>& joint_state)
{
  getCurrentServoPosition();

  // State of arm joints, convert positions to radians
  for (auto i = 1; i != SERVO_NUM; ++i)
  {
    joint_state[JOINT_NUM - i] =
        (i == 4) ? ((500 - servo_position_[i]) * M_PI / 750) : ((servo_position_[i] - 500) * M_PI / 750);
  }

  // State of the gripper joint, mapped from angle to distance
  joint_state[GRIPPER_ID] =
      0.03 -
      (-1.213930e-4 * servo_position_[0] * servo_position_[0] - 0.015326 * servo_position_[0] + 67.610949) / 2000;
}

inline void XarmDriver::getCurrentServoPosition()
{
  my_hid_.makeAndSendCmd(CMD_MULT_SERVO_POS_READ, { SERVO_NUM, 1, 2, 3, 4, 5, 6 });
  std::vector<unsigned> received_data;
  my_hid_.read(received_data, 21);

  if (received_data.size() != 0 && received_data[0] == CMD_MULT_SERVO_POS_READ && received_data[1] == SERVO_NUM)
  {
    auto position_it = servo_position_.begin();
    decltype(received_data.size()) i = 0;
    while (position_it != servo_position_.end())
    {
      *position_it = static_cast<int>(received_data[3 * i + 3]) + static_cast<int>(received_data[3 * i + 4] << 8);
      ++position_it;
      ++i;
    }
  }
}

inline void XarmDriver::spinServos(const std::initializer_list<unsigned>& id_list,
                                   const std::initializer_list<int>& position_list, const unsigned period)
{
  auto id_list_size = id_list.size();
  auto position_list_size = position_list.size();
  if (id_list_size != position_list_size)
  {
    return;
  }

  unsigned per = (period > 5000) ? 5000 : period;
  std::vector<unsigned> argv;
  argv.push_back(static_cast<unsigned>(id_list_size));
  argv.push_back(per & 0xFF);
  argv.push_back((per >> 8) & 0xFF);
  auto id_it = id_list.begin();
  auto position_it = position_list.begin();
  while (id_it != id_list.end())
  {
    argv.push_back(*id_it);
    argv.push_back(*position_it & 0xFF);
    argv.push_back((*position_it >> 8) & 0xFF);
    ++id_it;
    ++position_it;
  }

  my_hid_.makeAndSendCmd(CMD_MULT_SERVO_SPIN, argv);
}

}  // namespace lobot_hardware_interface

#endif  // XARM_DRIVER_H
