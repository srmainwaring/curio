//
//  Software License Agreement (BSD-3-Clause)
//   
//  Copyright (c) 2020 Rhys Mainwaring
//  All rights reserved
//   
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1.  Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//  2.  Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//  3.  Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//

#ifndef CURIO_BASE_ROVER_BASE_HARDWARE_H_
#define CURIO_BASE_ROVER_BASE_HARDWARE_H_

#include "curio_base/rover_base_hal.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <memory>
#include <string>
#include <vector>

namespace curio_base
{

    class RoverBaseHardware : public hardware_interface::RobotHW
    {
    public:
        virtual ~RoverBaseHardware();

        RoverBaseHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

        virtual void read(const ros::Time &time, const ros::Duration &period);

        virtual void write(const ros::Time &time, const ros::Duration &period);

    private:
        // ROS node handle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // ros_control interfaces: joints, velocity for wheels, position for steers.
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        // ROS parameters
        double wheel_radius_ = 0.06;
        
        // Passive joints - not controlled, joint interface only
        const size_t k_num_passive_joints_ = 5;
        std::vector<double> passive_joint_positions_;
        std::vector<double> passive_joint_velocities_;
        std::vector<double> passive_joint_efforts_;
        std::vector<std::string> passive_joint_names_;

        // Wheel joints - velocity controlled
        std::vector<double> wheel_joint_positions_;
        std::vector<double> wheel_joint_velocities_;
        std::vector<double> wheel_joint_efforts_;
        std::vector<double> wheel_joint_velocity_commands_;
        std::vector<std::string> wheel_joint_names_;

        // Steer joints - position controlled
        std::vector<double> steer_joint_positions_;
        std::vector<double> steer_joint_velocities_;
        std::vector<double> steer_joint_efforts_;
        std::vector<double> steer_joint_position_commands_;
        std::vector<std::string> steer_joint_names_;

        // Hardware abstraction
        std::unique_ptr<RoverBaseHAL> rover_hal_;

        void registerControlInterfaces();
    };

} // namespace curio_base

#endif // CURIO_BASE_ROVER_BASE_HARDWARE_H_
