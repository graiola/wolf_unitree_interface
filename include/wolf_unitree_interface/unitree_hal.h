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
#ifndef UNITREE_HAL_H
#define UNITREE_HAL_H

#include <array>

#include <unitree_legged_sdk/unitree_legged_sdk.h>

// using namespace UNITREE_LEGGED_SDK;
namespace unitree = UNITREE_LEGGED_SDK;

namespace unitreehal 
{


inline namespace unitree { using namespace ::unitree; }


class LowLevelInterface
{
public:
  LowLevelInterface();

  LowState ReceiveObservation();

  void SendLowCmd(LowCmd& motorcmd);
  void SendCommand(std::array<float, 60> motorcmd);
  void InitCmdData(LowCmd& motorcmd);

  UDP udp_;
  Safety safe_;

  LowCmd lowcmd_ = {0};
  LowState lowstate_ = {0};

};


class HighLevelInterface
{
public:
  HighLevelInterface();

  HighState ReceiveObservation();

  void SendHighCmd(HighCmd& descmd);
  // void SendCommand(std::array<float, 60> motorcmd);

  UDP udp_;
  Safety safe_;

  HighCmd highcmd_ = {0};
  HighState highstate_ = {0};

};


}


#endif
