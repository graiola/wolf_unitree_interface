/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef UNITREE_ROBOT_HW_H
#define UNITREE_ROBOT_HW_H

#include <wolf_hardware_interface/wolf_robot_hw.h>
#include "wolf_unitree_interface/unitree_hal.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include <nav_msgs/Odometry.h>

namespace unitree2ros
{

class UnitreeRobotHw : public hardware_interface::RobotHW, public hardware_interface::WolfRobotHwInterface
{
public:
  UnitreeRobotHw();
  virtual ~UnitreeRobotHw();

  void init();
  void read();
  void write();

private:

  /** @brief Map unitree internal joint indices to WoLF joints order */
  std::array<unsigned int, 12> unitree_motor_idxs_
          {{
          unitreehal::FL_0, unitreehal::FL_1, unitreehal::FL_2, // LF
          unitreehal::RL_0, unitreehal::RL_1, unitreehal::RL_2, // LH
          unitreehal::FR_0, unitreehal::FR_1, unitreehal::FR_2, // RF
          unitreehal::RR_0, unitreehal::RR_1, unitreehal::RR_2, // RH
          }};

  /** @brief Unitree-HAL */
  std::shared_ptr<unitreehal::HighLevelInterface> unitree_highinterface_;
  std::shared_ptr<unitreehal::LowLevelInterface>  unitree_lowinterface_;
  unitreehal::HighState unitree_highstate_ = {0};
  unitreehal::LowState unitree_lowstate_ = {0};
  unitreehal::LowCmd unitree_lowcmd_ = {0};

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();


  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;

};

}

#endif
