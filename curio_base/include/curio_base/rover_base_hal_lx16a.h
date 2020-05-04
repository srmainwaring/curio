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

#ifndef CURIO_BASE_ROVER_BASE_HAL_LX16A_H_
#define CURIO_BASE_ROVER_BASE_HAL_LX16A_H_

#include "curio_base/rover_base_hal.h"
#include "curio_base/lx16a_driver.h"
#include "curio_base/lx16a_encoder_filter.h"

#include <ros/ros.h>

#include <string>
#include <memory>

namespace curio_base
{
    /// \brief Hardware Abstraction Layer for the rover base.
    class RoverBaseHALLX16A : public RoverBaseHAL
    {
    public:
        /// \brief Destructor.
        virtual ~RoverBaseHALLX16A();

        /// \brief Constructor.
        RoverBaseHALLX16A(ros::NodeHandle &nh);

        /// \brief Get the number of wheels.
        virtual size_t getNumWheels() const;

        /// \brief Get the number of steers.
        virtual size_t getNumSteers() const;

        /// \brief Get the angular position of the i-th wheel [rad].
        virtual double getWheelPosition(const ros::Time &time, int i) const;

        /// \brief Get the angular velocity of the i-th wheel [rad/s].
        virtual double getWheelVelocity(const ros::Time &time, int i) const;

        /// \brief Set the angular velocity of the i-th wheel [rad/s].
        virtual void setWheelVelocity(const ros::Time &time, int i, double velocity);

        /// \brief Get the steering angle of the i-th steer [rad].
        virtual double getSteerAngle(const ros::Time &time, int i) const;

        /// \brief Set the angle of the i-th steer [rad].
        virtual void setSteerAngle(const ros::Time &time, int i, double angle);

        inline const LX16ADriver& servoDriver() const { return *servo_driver_; }

        LX16ADriver& servoDriver() { return *servo_driver_; }

    private:
        // Constants
        const size_t k_num_wheels_ = 6;
        const size_t k_num_steers_ = 4;

        // Node handle
        ros::NodeHandle nh_;
        
        // Servo ids
        std::vector<uint8_t> wheel_servo_ids_;
        std::vector<uint8_t> steer_servo_ids_;

        // Wheel orientation
        std::vector<int8_t> wheel_servo_orientations_;
        std::vector<int8_t> steer_servo_orientations_;

        // Current duty set points
        std::vector<uint16_t> wheel_servo_duties_;

        // Drivers and filters
        std::unique_ptr<LX16ADriver> servo_driver_;
        std::unique_ptr<LX16AEncoderFilter> encoder_filter_;
    };

} // namespace curio_base

#endif // CURIO_BASE_ROVER_BASE_HAL_LX16A_H_
