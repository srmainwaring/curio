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

#include "curio_base/rover_base_hardware.h"
#include "curio_base/rover_base_hal_lx16a.h"
#include "curio_base/rover_base_hal_mock.h"

namespace curio_base
{

    RoverBaseHardware::~RoverBaseHardware()
    {        
    }

    RoverBaseHardware::RoverBaseHardware(ros::NodeHandle nh, ros::NodeHandle private_nh)
    {
        ROS_INFO_STREAM("Initialising rover base hardware...");

        // @TODO - control switch with parameter / plugin.
        // LX16A HAL
        std::unique_ptr<RoverBaseHALLX16A> rover_hal_lx16a(new RoverBaseHALLX16A(nh));
        rover_hal_ = std::move(rover_hal_lx16a);

        // Mock HAL
        // std::unique_ptr<RoverBaseHALMock> rover_hal_mock(new RoverBaseHALMock()); 
        // rover_hal_ = std::move(rover_hal_mock);

        // Resize passive joints
        passive_joints_.resize(k_num_passive_joints_);
        ROS_INFO_STREAM("Number of passive joints: " << k_num_passive_joints_);

        // Resize controlled joints
        const int k_num_wheel_joints = rover_hal_->getNumWheels();
        const int k_num_steer_joints = rover_hal_->getNumSteers();
        wheel_joints_.resize(k_num_wheel_joints);
        steer_joints_.resize(k_num_steer_joints);
        ROS_INFO_STREAM("Number of wheel joints: " << k_num_wheel_joints);
        ROS_INFO_STREAM("Number of steer joints: " << k_num_steer_joints);

        // Initialise parameters 

        // Startup
        registerControlInterfaces();
    }

    void RoverBaseHardware::read(const ros::Time &time, const ros::Duration &period)
    {
        const int k_num_wheel_joints = rover_hal_->getNumWheels();
        const int k_num_steer_joints = rover_hal_->getNumSteers();

        // Set position of wheel joints
        for (int i=0; i<k_num_wheel_joints; ++i)
        {
            try {
                wheel_joints_[i].position = rover_hal_->getWheelPosition(time, i);
            } catch (const RoverBaseHALException &e) {
                // Report error and use last known position. 
                ROS_ERROR_STREAM(e.what());
            }
        }

        // Set velocities of wheel joints
        for (int i=0; i<k_num_wheel_joints; ++i)
        {
            wheel_joints_[i].velocity = rover_hal_->getWheelVelocity(time, i);
        }

        // Set position of steer joints
        for (int i=0; i<k_num_steer_joints; ++i)
        {
            try {
               steer_joints_[i].position = rover_hal_->getSteerAngle(time, i);
            } catch (const RoverBaseHALException &e) {
                // Report error and use last known position. 
                ROS_ERROR_STREAM(e.what());
            }
        }
    }

    void RoverBaseHardware::write(const ros::Time &time, const ros::Duration &period)
    {
        const int k_num_wheel_joints = rover_hal_->getNumWheels();
        const int k_num_steer_joints = rover_hal_->getNumSteers();

        // Set commanded velocities
        for (int i=0; i<k_num_wheel_joints; ++i)
        {
            rover_hal_->setWheelVelocity(time, i, wheel_joints_[i].velocity_command);
        }

        // Set commanded positions
        for (int i=0; i<k_num_steer_joints; ++i)
        {
            rover_hal_->setSteerAngle(time, i, steer_joints_[i].position_command);
        }
    }

    void RoverBaseHardware::registerControlInterfaces()
    {
        const int k_num_wheel_joints = rover_hal_->getNumWheels();
        const int k_num_steer_joints = rover_hal_->getNumSteers();

        ROS_INFO_STREAM("Registering passive joint interfaces");

        // Register the names for passive joints.
        std::vector<std::string> passive_joint_names_ = {
            "diff_brace_joint",
            "left_bogie_joint",
            "left_rocker_joint",
            "right_bogie_joint",
            "right_rocker_joint"
        };

        // Register the wheel joint state and joint handles.
        for (int i=0; i<k_num_passive_joints_; ++i)
        {
            hardware_interface::JointStateHandle joint_state_handle(
                passive_joint_names_[i],
                &passive_joints_[i].position,
                &passive_joints_[i].velocity,
                &passive_joints_[i].effort);
            joint_state_interface_.registerHandle(joint_state_handle);
        }

        ROS_INFO_STREAM("Registering wheel joint interfaces");

        // Register the names for wheel joint controllers.
        std::vector<std::string> wheel_joint_names_ = {
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "mid_left_wheel_joint",
            "mid_right_wheel_joint",
            "back_left_wheel_joint",
            "back_right_wheel_joint"
        };

        // Register the wheel joint state and joint handles.
        for (int i=0; i<k_num_wheel_joints; ++i)
        {
            hardware_interface::JointStateHandle joint_state_handle(
                wheel_joint_names_[i],
                &wheel_joints_[i].position,
                &wheel_joints_[i].velocity,
                &wheel_joints_[i].effort);
            joint_state_interface_.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_handle(
                joint_state_handle, &wheel_joints_[i].velocity_command);
            velocity_joint_interface_.registerHandle(joint_handle);
        }

        ROS_INFO_STREAM("Registering steer joint interfaces");

        // Register the names for steer joint controllers.
        std::vector<std::string> steer_joint_names_ = {
            "front_left_corner_joint",
            "front_right_corner_joint",
            "back_left_corner_joint",
            "back_right_corner_joint"
        };

        // Register the steer joint state and joint handles.
        for (int i=0; i<k_num_steer_joints; ++i)
        {
            hardware_interface::JointStateHandle joint_state_handle(
                steer_joint_names_[i],
                &steer_joints_[i].position,
                &steer_joints_[i].velocity,
                &steer_joints_[i].effort);
            joint_state_interface_.registerHandle(joint_state_handle);

            hardware_interface::JointHandle joint_handle(
                joint_state_handle, &steer_joints_[i].position_command);
            position_joint_interface_.registerHandle(joint_handle);
        }

        // Register interfaces
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&position_joint_interface_);
    }


} // namespace curio_base
