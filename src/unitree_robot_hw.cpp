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

#include "wolf_unitree_interface/unitree_robot_hw.hpp"

namespace unitree2ros
{

using namespace hardware_interface;

int64_t utime_now() {

    struct timeval timeofday;
    gettimeofday(&timeofday,NULL);
    if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
        throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
    uint32_t sec	= timeofday.tv_sec;
    uint32_t nsec = timeofday.tv_usec * 1000;

    return (int64_t) (((uint64_t)sec)*1000000 + ((uint64_t)nsec) / 1000);
}

UnitreeRobotHw::UnitreeRobotHw()
{

    // FIXME add robot model
    //robot_name_ = "unitree";
}

UnitreeRobotHw::~UnitreeRobotHw()
{

}

void UnitreeRobotHw::init()
{

    // Hardware interfaces: Joints
    auto joint_names = loadJointNamesFromSRDF();
    if(joint_names.size()>0)
    {
      WolfRobotHwInterface::initializeJointsInterface(joint_names);
      registerInterface(&joint_state_interface_);
      registerInterface(&joint_effort_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register joint interface.");
      return;
    }

    // Hardware interfaces: IMU
    auto imu_name = loadImuLinkNameFromSRDF();
    if(!imu_name.empty())
    {
      WolfRobotHwInterface::initializeImuInterface(imu_name);
      registerInterface(&imu_sensor_interface_);
    }
    else
    {
      ROS_ERROR_NAMED(CLASS_NAME,"Failed to register imu interface.");
      return;
    }

    // Hardware interfaces: Ground Truth (NOTE: this is the odom from the robot)
    auto base_name = loadBaseLinkNameFromSRDF();
    if(!base_name.empty())
    {
      WolfRobotHwInterface::initializeGroundTruthInterface(loadBaseLinkNameFromSRDF());
      registerInterface(&ground_truth_interface_);
    }
    else
    {
      ROS_WARN_NAMED(CLASS_NAME,"Failed to register ground truth interface.");
    }

    // Hardware interfaces: Contact sensors
    auto contact_names = loadContactNamesFromSRDF();
    if(!contact_names.empty())
    {
      WolfRobotHwInterface::initializeContactSensorsInterface(contact_names);
      registerInterface(&contact_sensor_interface_);
    }
    else
    {
      ROS_WARN_NAMED(CLASS_NAME,"Failed to register contacts interface.");
    }

    unitree_interface_.InitCmdData(unitree_lowcmd_);
    startup_routine();

    ros::NodeHandle nh;
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh,	"odometry/robot", 1));


}

void UnitreeRobotHw::read()
{
    // Get robot data
    unitree_lowstate_ = unitree_interface_.ReceiveObservation();

    // ------
    // Joints
    // ------
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
    {
        joint_position_[jj] = static_cast<double>(unitree_lowstate_.motorState[unitree_motor_idxs_[jj]].q)     ;
        joint_velocity_[jj] = static_cast<double>(unitree_lowstate_.motorState[unitree_motor_idxs_[jj]].dq)    ;
        joint_effort_[jj]   = static_cast<double>(unitree_lowstate_.motorState[unitree_motor_idxs_[jj]].tauEst);
    }

    // ---
    // IMU
    // ---
    imu_orientation_[0] = static_cast<double>(unitree_lowstate_.imu.quaternion[0]);  // w
    imu_orientation_[1] = static_cast<double>(unitree_lowstate_.imu.quaternion[1]);  // x
    imu_orientation_[2] = static_cast<double>(unitree_lowstate_.imu.quaternion[2]);  // y
    imu_orientation_[3] = static_cast<double>(unitree_lowstate_.imu.quaternion[3]);  // z

    imu_ang_vel_[0] = static_cast<double>(unitree_lowstate_.imu.gyroscope[0]);
    imu_ang_vel_[1] = static_cast<double>(unitree_lowstate_.imu.gyroscope[1]);
    imu_ang_vel_[2] = static_cast<double>(unitree_lowstate_.imu.gyroscope[2]);

    imu_lin_acc_[0] = static_cast<double>(unitree_lowstate_.imu.accelerometer[0]);
    imu_lin_acc_[1] = static_cast<double>(unitree_lowstate_.imu.accelerometer[1]);
    imu_lin_acc_[2] = static_cast<double>(unitree_lowstate_.imu.accelerometer[2]);

    // ---
    // GT
    // ---
    // TODO

    // ---
    // Contacts 
    // ---
    // TODO


    // Publish the IMU data NOTE: missing covariances
    if(odom_pub_.get() && odom_pub_->trylock())
    {
      odom_pub_->msg_.pose.pose.orientation.w  = imu_orientation_[0];
      odom_pub_->msg_.pose.pose.orientation.x  = imu_orientation_[1];
      odom_pub_->msg_.pose.pose.orientation.y  = imu_orientation_[2];
      odom_pub_->msg_.pose.pose.orientation.z  = imu_orientation_[3];
      odom_pub_->msg_.twist.twist.angular.x    = imu_ang_vel_[0];
      odom_pub_->msg_.twist.twist.angular.y    = imu_ang_vel_[1];
      odom_pub_->msg_.twist.twist.angular.z    = imu_ang_vel_[2];

      odom_pub_->msg_.header.stamp = ros::Time::now();
      odom_pub_->unlockAndPublish();
    }
}

void UnitreeRobotHw::write()
{
    for (unsigned int jj = 0; jj < n_dof_; ++jj)
      unitree_lowcmd_.motorCmd[unitree_motor_idxs_[jj]].tau = static_cast<float>(joint_effort_command_[jj]  );

    unitree_interface_.SendLowCmd(unitree_lowcmd_);
}

void UnitreeRobotHw::send_zero_command()
{
    std::array<float, 60> zero_command = {0};
    // unitree_interface_->SendCommand(zero_command);
    unitree_interface_.SendCommand(zero_command);
}

void UnitreeRobotHw::startup_routine()
{
    send_zero_command();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace
