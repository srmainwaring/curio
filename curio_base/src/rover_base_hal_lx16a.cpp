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

#include "curio_base/rover_base_hal_lx16a.h"

namespace curio_base
{
    
    double servoPos2Angle(int16_t position)
    {
        // Map steering angle degrees [-120, 120] to servo position [0, 1000]
        // servo_pos = int(map(angle_deg * servo.orientation,
        //     -BaseController.SERVO_ANGLE_MAX, BaseController.SERVO_ANGLE_MAX,
        //     BaseController.SERVO_POS_MIN, BaseController.SERVO_POS_MAX))
        return 0.0;
    }

    int16_t angle2ServoPos(double angle)
    {
        // Map steering angle degrees [-120, 120] to servo position [0, 1000]
        // servo_pos = int(map(angle_deg * servo.orientation,
        //     -BaseController.SERVO_ANGLE_MAX, BaseController.SERVO_ANGLE_MAX,
        //     BaseController.SERVO_POS_MIN, BaseController.SERVO_POS_MAX))
        return 500;
    }

    int16_t velocity2ServoDuty(double velocity)
    {
        // Convert from velocity to servo duty.
        //
        double  wheel_diameter       =  0.6;
        int16_t servo_counts_per_rev =  1500;
        int16_t servo_counts_per_sec =  1500;
        int16_t servo_min_duty       = -1000;
        int16_t servo_max_duty       =  1000;
        return 500;
    }

    RoverBaseHALLX16A::~RoverBaseHALLX16A()
    {        
    }

    RoverBaseHALLX16A::RoverBaseHALLX16A() :
        wheel_servo_ids_(k_num_wheels_),
        steer_servo_ids_(k_num_steers_),
        wheel_servo_duties_(k_num_wheels_, 0)
    {
        wheel_servo_ids_ = { 11, 21, 12, 22, 13, 23 };
        steer_servo_ids_ = { 111, 211, 131, 231 };
    }

    size_t RoverBaseHALLX16A::getNumWheels() const
    {
        return k_num_wheels_;
    }

    size_t RoverBaseHALLX16A::getNumSteers() const
    {
        return k_num_steers_;
    }

    double RoverBaseHALLX16A::getWheelPosition(int i) const
    {
        // @TODO get time as argument
        ros::Time ros_time = ros::Time::now();

        // Get position and duty.
        uint8_t servo_id = wheel_servo_ids_[i];
        int16_t pos = servo_driver_->getPosition(servo_id);
        int16_t duty = wheel_servo_duties_[i];
        
        // Update filter
        encoder_filter_->update(ros_time, duty, pos);

        // Get angle
        double angle = encoder_filter_->getAngularPosition();
        return angle;
    }

    double RoverBaseHALLX16A::getWheelVelocity(int i) const
    {
        uint8_t servo_id = wheel_servo_ids_[i];

        return 0;
    }

    void RoverBaseHALLX16A::setWheelVelocity(int i, double velocity)
    {
        uint8_t servo_id = wheel_servo_ids_[i];

        // Convert to duty.
    }

    double RoverBaseHALLX16A::getSteerAngle(int i) const
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int16_t pos = servo_driver_->getPosition(servo_id);
        double angle = servoPos2Angle(pos); 
        return angle;
    }

    void RoverBaseHALLX16A::setSteerAngle(int i, double angle)
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int16_t pos = angle2ServoPos(angle);
        bool status = servo_driver_->move(servo_id, pos, 0);
    }

} // namespace curio_base
