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
#include "curio_base/lx16a_encoder_filter_client.h"
#include <serial/serial.h>
#include <cmath>

namespace curio_base
{
    // Velocity limits for the rover
    static const double LINEAR_VEL_MAX =  0.37;
    static const double ANGULAR_VEL_MAX = 1.45;

    // Servo limits - LX-16A has max angular velocity of approx 1 revolution per second
    static const double SERVO_ANG_VEL_MAX = 2 * M_PI;
    static const double SERVO_DUTY_MAX = 1000.0;

    // Steering angle limits
    static const double REVERSE_SERVO_ANGLE = 90.0; // The angle at which we reverse the servo by 180 deg.
    static const double SERVO_ANGLE_MAX = 120.0;    // Maximum (abs) angle at which the servo can be set.
    static const double SERVO_POS_MIN = 0.0;        // Minimum servo position (servo units).
    static const double SERVO_POS_MAX = 1000.0;     // Maximum servo position (servo units).

    static const int16_t SERVO_MOVE_TIME = 50;      // Time taken to move steering servos to new position [ms]. 

    // 6 wheels, 4 steering.
    //NUM_WHEELS = 6
    //NUM_STEERS = 4

    /// \brief Map a value in one range to its equivalent in another.
    ///
    /// \param x        The value to be mapped
    /// \param in_min   The minimum value the input variable can take. 
    /// \param in_max   The maximum value the input variable can take. 
    /// \param out_min  The minimum value the output variable can take. 
    /// \param out_max  The maximum value the output variable can take. 
    /// \return The mapped value.
    ///
    double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /// \brief Convert radians to degrees
    double rad2Deg(double radian)
    {
        return radian * 180.0 / M_PI;
    }

    /// \brief Convert degrees to radians
    double deg2Rad(double degree)
    {
        return degree * M_PI / 180.0;
    }

    /// \brief Map servo duty [-1000, 1000] to angular velocity [rad] 
    double duty2AngularVelocity(int16_t duty)
    {
        double angular_velocity = map(duty,
            -SERVO_DUTY_MAX, SERVO_DUTY_MAX, -SERVO_ANG_VEL_MAX, SERVO_ANG_VEL_MAX);
        return angular_velocity;
    }

    /// \brief Map angular velocity [rad] to servo duty [-1000, 1000]
    int16_t angularVelocity2Duty(double angular_velocity)
    {
        int16_t duty = static_cast<int16_t>(map(angular_velocity,
            -SERVO_ANG_VEL_MAX, SERVO_ANG_VEL_MAX, -SERVO_DUTY_MAX, SERVO_DUTY_MAX));
        return duty;
    }

    /// \brief Map servo position [0, 1000] to steering angle degrees [-120, 120]
    double servoPos2Angle(int16_t servo_pos)
    {
        double angle_deg = map(servo_pos,
            SERVO_POS_MIN, SERVO_POS_MAX, -SERVO_ANGLE_MAX, SERVO_ANGLE_MAX);
        double angle_rad = deg2Rad(angle_deg);
        return angle_rad;
    }

    /// \brief Map steering angle degrees [-120, 120] to servo position [0, 1000]
    int16_t angle2ServoPos(double angle_rad)
    {
        double angle_deg = rad2Deg(angle_rad);
        int16_t servo_pos = static_cast<int16_t>(map(angle_deg,
            -SERVO_ANGLE_MAX, SERVO_ANGLE_MAX, SERVO_POS_MIN, SERVO_POS_MAX));
        return servo_pos;
    }

    // Convert from velocity to servo duty.
    // int16_t velocity2ServoDuty(double velocity)
    // {
    //     double  wheel_diameter       =  0.6;
    //     int16_t servo_counts_per_rev =  1500;
    //     int16_t servo_counts_per_sec =  1500;
    //     int16_t servo_min_duty       = -1000;
    //     int16_t servo_max_duty       =  1000;
    //     return 500;
    // }

    RoverBaseHALLX16A::~RoverBaseHALLX16A()
    {        
    }

    RoverBaseHALLX16A::RoverBaseHALLX16A(ros::NodeHandle &nh) :
        wheel_servo_ids_(k_num_wheels_),
        steer_servo_ids_(k_num_steers_),
        wheel_servo_orientations_(k_num_wheels_),
        steer_servo_orientations_(k_num_steers_),
        wheel_servo_duties_(k_num_wheels_, 0),
        wheel_servo_last_positions_(k_num_wheels_, 0),
        steer_servo_last_positions_(k_num_steers_, 500)
    {
        // @TODO hardcoded parameters
        // Parameters
        wheel_servo_ids_ = { 11, 21, 12, 22, 13, 23 };
        steer_servo_ids_ = { 111, 211, 131, 231 };

        // Right wheel servos aligned: -y axis => negative orientation
        wheel_servo_orientations_ = { 1, -1, 1, -1, 1, -1 };

        // Steer servos aligned: -z axis => negative orientation.
        steer_servo_orientations_ = { -1, -1, -1, -1 };

        const std::string port("/dev/cu.usbmodemFD4121");
        const uint32_t baudrate = 115200;
        const uint32_t timeout = 1000;       // [ms]

        std::string classifier_filename("./src/curio/curio_base/data/lx16a_tree_classifier.joblib");
        std::string regressor_filename("./src/curio/curio_base/data/lx16a_tree_regressor.joblib");
        int16_t window = 10;

        // Initialise servo driver.
        serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
        ros::Duration response_timeout(0.015);
        servo_driver_ = std::unique_ptr<LX16ADriver>(new LX16ADriver());
        servo_driver_->setPort(port);
        servo_driver_->setBaudrate(baudrate);
        servo_driver_->setTimeout(serial_timeout);
        servo_driver_->setResponseTimeout(response_timeout);
        servo_driver_->open();
        ROS_INFO_STREAM("port: " << servo_driver_->getPort());
        ROS_INFO_STREAM("baudrate: " << servo_driver_->getBaudrate());
        ROS_INFO_STREAM("is_open: " << servo_driver_->isOpen());

        // Wait for Arduino bootloader to complete before sending any
        // data on the serial connection.
        ROS_INFO_STREAM("Waiting for bootloader to complete...");
        ros::Duration(1.0).sleep();

        // Initialise encoder filter.
        std::unique_ptr<LX16AEncoderFilterClient> filter(
            new LX16AEncoderFilterClient(
                nh, classifier_filename, regressor_filename, window));
        encoder_filter_ = std::move(filter);
        encoder_filter_->init();
        ROS_INFO_STREAM("classifier_filename: " << classifier_filename);
        ROS_INFO_STREAM("regressor_filename: " << regressor_filename);
        ROS_INFO_STREAM("window: " << window);
        
        // Add servos
        for (size_t i=0; i<k_num_wheels_; ++i)
        {
            encoder_filter_->add(wheel_servo_ids_[i]);
        }
    }

    size_t RoverBaseHALLX16A::getNumWheels() const
    {
        return k_num_wheels_;
    }

    size_t RoverBaseHALLX16A::getNumSteers() const
    {
        return k_num_steers_;
    }

    double RoverBaseHALLX16A::getWheelPosition(const ros::Time &time, int i) const
    {
        uint8_t servo_id = wheel_servo_ids_[i];
        int8_t orientation = wheel_servo_orientations_[i];
        int16_t duty = wheel_servo_duties_[i];

        // Get position and update filter.
        try {
            int16_t pos = servo_driver_->getPosition(servo_id);
            encoder_filter_->update(servo_id, time, duty, pos);
            double angle = encoder_filter_->getAngularPosition(servo_id) * orientation;        

            ROS_DEBUG_STREAM("get wheel: id: " << static_cast<int>(servo_id)
                << ", angle: " << angle
                << ", pos: " << static_cast<int>(pos)
                << ", orient: " << static_cast<int>(orientation)
                << ", duty: " << static_cast<int>(duty));

            return angle;
        }
        catch(const LX16AException &e) {
            ROS_ERROR_STREAM(e.what()
                << " wheel: id: " << static_cast<int>(servo_id));
            return 0.0;
        }
    }

    double RoverBaseHALLX16A::getWheelVelocity(const ros::Time &time, int i) const
    {
        uint8_t servo_id = wheel_servo_ids_[i];
        int8_t orientation = wheel_servo_orientations_[i];
        int16_t duty = wheel_servo_duties_[i];
        double velocity = duty2AngularVelocity(duty) * orientation;

        ROS_DEBUG_STREAM("get wheel: id: " << static_cast<int>(servo_id)
            << ", vel: " << velocity
            << ", orient: " << static_cast<int>(orientation)
            << ", duty: " << static_cast<int>(duty));

        return velocity;
    }

    void RoverBaseHALLX16A::setWheelVelocity(const ros::Time &time, int i, double velocity)
    {
        uint8_t servo_id = wheel_servo_ids_[i];
        int8_t orientation = wheel_servo_orientations_[i];
        int16_t duty = angularVelocity2Duty(velocity * orientation);

        // Save the duty set point.
        wheel_servo_duties_[i] = duty;

        // Update the servo.
        try {
            servo_driver_->setMotorMode(servo_id, duty);   

            ROS_DEBUG_STREAM("set wheel: id: " << static_cast<int>(servo_id)
                << ", vel: " << velocity
                << ", orient: " << static_cast<int>(orientation)
                << ", duty: " << static_cast<int>(duty));
        }
        catch(const LX16AException &e) {
            ROS_ERROR_STREAM(e.what()
                << " set wheel: id: " << static_cast<int>(servo_id));
        }
    }

    double RoverBaseHALLX16A::getSteerAngle(const ros::Time &time, int i) const
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int8_t orientation = steer_servo_orientations_[i];

        try {
            int16_t pos = servo_driver_->getPosition(servo_id);
            double angle = servoPos2Angle(pos) * orientation;        

            ROS_DEBUG_STREAM("get steer: id: " << static_cast<int>(servo_id)
                << ", angle: " << angle
                << ", pos: " << static_cast<int>(pos));

            return angle;
        }
        catch(const LX16AException &e) {
            ROS_ERROR_STREAM(e.what()
                << " get steer: id: " << static_cast<int>(servo_id));
            return 0.0;
        }
    }

    void RoverBaseHALLX16A::setSteerAngle(const ros::Time &time, int i, double angle)
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int8_t orientation = steer_servo_orientations_[i];
        int16_t pos = angle2ServoPos(angle * orientation);

        try {
            servo_driver_->move(servo_id, pos, SERVO_MOVE_TIME);

            ROS_DEBUG_STREAM("set steer: id: " << static_cast<int>(servo_id)
                << ", angle: " << angle
                << ", pos: " << static_cast<int>(pos));
        }
        catch(const LX16AException &e) {
            ROS_ERROR_STREAM(e.what()
                << " set steer: id: " << static_cast<int>(servo_id));
        }
    }

} // namespace curio_base
