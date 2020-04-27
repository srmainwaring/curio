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

#include "curio_base/rover_base_hal_mock.h"

namespace curio_base
{
    RoverBaseHALMock::~RoverBaseHALMock()
    {        
    }

    RoverBaseHALMock::RoverBaseHALMock() :
        RoverBaseHAL(),
        wheel_last_times_(k_num_wheels_),
        wheel_last_positions_(k_num_wheels_),
        wheel_velocities_(k_num_wheels_),
        steer_positions_(k_num_steers_)
    {
        // Initialise velocity controlled wheel variables
        for (size_t i=0; i<k_num_wheels_; ++i)
        {
            wheel_last_times_[i] = time_source::now();
            wheel_last_positions_[i] = 0.0;
            wheel_velocities_[i] = 0.0;
        }

        // Initialise position controlled steer variables
        for (size_t i=0; i<k_num_steers_; ++i)
        {
            steer_positions_[i] = 0.0;
        }
    }

    size_t RoverBaseHALMock::getNumWheels() const
    {
        return k_num_wheels_;
    }

    size_t RoverBaseHALMock::getNumSteers() const
    {
        return k_num_steers_;
    }

    double RoverBaseHALMock::getWheelPosition(int i) const
    {
        // Calculate the current wheel position.
        time_source::time_point this_time = time_source::now();        
        std::chrono::duration<double> elapsed_duration = this_time - wheel_last_times_[i];
        return  wheel_last_positions_[i] + elapsed_duration.count() * wheel_velocities_[i];
    }

    double RoverBaseHALMock::getWheelVelocity(int i) const
    {
        return wheel_velocities_[i];
    }

    void RoverBaseHALMock::setWheelVelocity(int i, double velocity)
    {
        // Determine the elapsed time since the last update for this wheel.
        time_source::time_point this_time = time_source::now();
        std::chrono::duration<double> elapsed_duration = this_time - wheel_last_times_[i];
        wheel_last_times_[i] = this_time;

        // Update the wheel position since the last change in velocity.
        wheel_last_positions_[i] += elapsed_duration.count() * wheel_velocities_[i]; 

        // Set the new wheel velocity.
        wheel_velocities_[i] = velocity;
    }

    double RoverBaseHALMock::getSteerAngle(int i) const
    {
        return steer_positions_[i];
    }

    void RoverBaseHALMock::setSteerAngle(int i, double angle)
    {
        steer_positions_[i] = angle;
    }

} // namespace curio_base
