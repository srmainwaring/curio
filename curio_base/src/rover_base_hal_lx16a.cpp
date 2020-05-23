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
#include "lx16a/lx16a_encoder_filter_client.h"
#include "lx16a/lx16a_encoder_filter_python.h"
#include "lx16a/lx16a_exception.h"
#include <serial/serial.h>
#include <algorithm>
#include <cmath>
#include <string>

namespace curio_base
{
    // Velocity limits for the rover
    static const double LINEAR_VEL_MAX =  0.37;
    static const double ANGULAR_VEL_MAX = 1.45;

    // Servo limits - LX-16A has max angular velocity of approx 1 revolution per second
    static const double SERVO_ANG_VEL_MAX = 2 * M_PI;
    static const int16_t SERVO_DUTY_MAX = 1000;

    // Steering angle limits
    static const double REVERSE_SERVO_ANGLE = 90.0; // The angle at which we reverse the servo by 180 deg.
    static const double SERVO_ANGLE_MAX = 120.0;    // Maximum (abs) angle at which the servo can be set.
    static const int16_t SERVO_POS_MIN = 0;         // Minimum servo position (servo units).
    static const int16_t SERVO_POS_MAX = 1000;      // Maximum servo position (servo units).

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

    int16_t clamp(int16_t x, int16_t lower, int16_t upper)
    {
        return std::min(std::max(x, lower), upper);
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

    template <typename T>
    bool validateServoParam(const T& param, const std::string& name, size_t expected_length)
    {
        if (param.size() != expected_length)
        {
            std::stringstream ss;
            ss << "Parameter '" << name
                << "' must be an array length " << expected_length
                << ", got: " << param.size();
            ROS_FATAL_STREAM(ss.str());
            throw RoverBaseHALException(ss.str().c_str());
            return false;
        }
        return true;
    }

    /// \brief simple case insensitive comparison
    ///
    /// See:
    /// https://stackoverflow.com/questions/11635/case-insensitive-string-comparison-in-c
    /// https://en.cppreference.com/w/cpp/string/byte/tolower
    bool iequal(const std::string& a, const std::string& b)
    {
        if (a.size() != b.size()) return false;
        bool is_equal = true;
        for (auto sa = a.begin(), sb = b.begin();
            sa != a.end() && sb != b.end();
            ++sa, ++sb)
        {
            is_equal &= (std::tolower(static_cast<unsigned char>(*sa)) 
                == std::tolower(static_cast<unsigned char>(*sb)));
        }
        return is_equal;

        // C++14
        // return std::for_each(a.begin(), a.end(), 
        //     [](char a, char b) {
        //         return tolower(static_cast<unsigned char>(a)) == tolower(static_cast<unsigned char>(b));
        //     });
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

    RoverBaseHALLX16A::RoverBaseHALLX16A(ros::NodeHandle &nh, ros::NodeHandle &private_nh) :
        nh_(nh),
        private_nh_(private_nh),
        wheel_servo_ids_(k_num_wheels_),
        steer_servo_ids_(k_num_steers_),
        wheel_servo_orientations_(k_num_wheels_),
        steer_servo_orientations_(k_num_steers_),
        wheel_servo_duties_(k_num_wheels_, 0)
    {
        // Initialise parameters

        // Wheel servo parameters
        // wheel_servo_ids:           [     11,     12,      13,      21,      22,      23]
        // wheel_servo_lon_labels:    ['front',  'mid',  'back', 'front',   'mid',  'back']
        // wheel_servo_lat_labels:    [ 'left', 'left',  'left', 'right', 'right', 'right']
        std::vector<int> wheel_servo_ids;
        std::vector<std::string> wheel_servo_lon_labels;
        std::vector<std::string> wheel_servo_lat_labels;
        ros::param::get("~wheel_servo_ids", wheel_servo_ids);
        ros::param::get("~wheel_servo_lon_labels", wheel_servo_lon_labels);
        ros::param::get("~wheel_servo_lat_labels", wheel_servo_lat_labels);
        validateServoParam(wheel_servo_ids, "wheel_servo_ids", k_num_wheels_);
        validateServoParam(wheel_servo_lon_labels, "wheel_servo_lon_labels", k_num_wheels_);
        validateServoParam(wheel_servo_lat_labels, "wheel_servo_lat_labels", k_num_wheels_);

        // Steering servo parameters
        // steer_servo_ids:           [    111,    131,     211,     231]
        // steer_servo_lon_labels:    ['front', 'back', 'front',  'back']
        // steer_servo_lat_labels:    [ 'left', 'left', 'right', 'right']
        // steer_servo_angle_offsets: [    -10,     -5,      40,      20]
        std::vector<int> steer_servo_ids;
        std::vector<std::string> steer_servo_lon_labels;
        std::vector<std::string> steer_servo_lat_labels;
        std::vector<int> steer_servo_angle_offsets;
        ros::param::get("~steer_servo_ids", steer_servo_ids);
        ros::param::get("~steer_servo_lon_labels", steer_servo_lon_labels);
        ros::param::get("~steer_servo_lat_labels", steer_servo_lat_labels);
        ros::param::get("~steer_servo_angle_offsets", steer_servo_angle_offsets);
        validateServoParam(steer_servo_ids, "steer_servo_ids", k_num_steers_);
        validateServoParam(steer_servo_lon_labels, "steer_servo_lon_labels", k_num_steers_);
        validateServoParam(steer_servo_lat_labels, "steer_servo_lat_labels", k_num_steers_);
        validateServoParam(steer_servo_angle_offsets, "steer_servo_angle_offsets", k_num_steers_);

        // wheel_servo_ids_ = { 11, 21, 12, 22, 13, 23 };
        std::copy(wheel_servo_ids.begin(), wheel_servo_ids.end(), wheel_servo_ids_.begin());

        // steer_servo_ids_ = { 111, 211, 131, 231 };
        std::copy(steer_servo_ids.begin(), steer_servo_ids.end(), steer_servo_ids_.begin());

        // Right wheel servos aligned: -y axis => negative orientation
        // wheel_servo_orientations_ = { 1, -1, 1, -1, 1, -1 };
        for (size_t i=0; i<k_num_wheels_; ++i)
        {
            if (iequal(wheel_servo_lat_labels[i], "LEFT"))
            {
                wheel_servo_orientations_[i] = 1;
            }
            else if (iequal(wheel_servo_lat_labels[i], "RIGHT"))
            {
                wheel_servo_orientations_[i] = -1;
            }
            else
            {
                std::stringstream ss;
                ss << "Invalid lateral label for wheel servo: "
                    << wheel_servo_ids_[i]
                    << ", got: " << wheel_servo_lat_labels[i]
                    << " expect 'left' or 'right'";
                ROS_FATAL_STREAM(ss.str());
                throw RoverBaseHALException(ss.str().c_str());
            }
        }

        // Steer servos aligned: -z axis => negative orientation.
        // steer_servo_orientations_ = { -1, -1, -1, -1 };
        for (size_t i=0; i<k_num_steers_; ++i)
        {
            steer_servo_orientations_[i] = -1;
        }

        ROS_INFO_STREAM("Loading wheel servo parameters...");
        for (size_t i=0; i<k_num_wheels_; ++i)
        {
            ROS_INFO_STREAM("id: " << wheel_servo_ids[i]
                << ", lon: " << wheel_servo_lon_labels[i]
                << ", lat: " << wheel_servo_lat_labels[i]
                << ", orient: " << static_cast<int>(wheel_servo_orientations_[i]));
        }

        ROS_INFO_STREAM("Loading steer servo parameters...");
        for (size_t i=0; i<k_num_steers_; ++i)
        {
            ROS_INFO_STREAM("id: " << steer_servo_ids[i]
                << ", lon: " << steer_servo_lon_labels[i]
                << ", lat: " << steer_servo_lat_labels[i]
                << ", orient: " << static_cast<int>(steer_servo_orientations_[i])
                << ", offset: " << steer_servo_angle_offsets[i]);
        }

        // LX-16A servo driver - all parameters are required
        // Servo driver port settings
        // port: /dev/ttyACM0
        // baudrate: 115200
        // timeout: 1.0
        std::string port;
        int baudrate = 115200;
        double timeout = 1.0;               // [s]
        double response_timeout = 0.015;    // [s]
        ros::param::get("~port", port);
        ros::param::get("~baudrate", baudrate);
        ros::param::get("~timeout", timeout);
        ros::param::get("~response_timeout", response_timeout);

        // Encoder filters
        std::string classifier_filename;
        std::string regressor_filename;
        int classifier_window = 10;        
        double encoder_service_timeout = 0.005;        
        ros::param::get("~classifier_window", classifier_window);
        if (!ros::param::has("~classifier_filename"))
        {
            ROS_FATAL("Missing parameter: classifier_filename");
        }
        ros::param::get("~classifier_filename", classifier_filename);

        if (!ros::param::has("~regressor_filename"))
        {
            ROS_FATAL("Missing parameter: regressor_filename");
        }
        ros::param::get("~regressor_filename", regressor_filename);
        ros::param::get("~encoder_service_timeout", encoder_service_timeout);

        // Initialise servo driver.
        ROS_INFO("Initialising LX16A servo driver...");
        uint32_t timeout_millis = static_cast<uint32_t>(timeout * 1000);
        serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout_millis);
        std::chrono::milliseconds response_timeout_millis(static_cast<int>(1000 * response_timeout));
        servo_driver_ = std::unique_ptr<lx16a::LX16ADriver>(new lx16a::LX16ADriver());
        servo_driver_->setPort(port);
        servo_driver_->setBaudrate(baudrate);
        servo_driver_->setTimeout(serial_timeout);
        servo_driver_->setResponseTimeout(response_timeout_millis);
        try
        {
            servo_driver_->open();
        }
        catch(const serial::IOException & e)
        {
            ROS_FATAL_STREAM("LX16A driver: failed to open port: " << port);
        }        
        ROS_INFO_STREAM("LX16A driver: port: " << servo_driver_->getPort());
        ROS_INFO_STREAM("LX16A driver: baudrate: " << servo_driver_->getBaudrate());
        ROS_INFO_STREAM("LX16A driver: is_open: " << servo_driver_->isOpen());

        // Wait for Arduino bootloader to complete before sending any
        // data on the serial connection.
        ROS_INFO_STREAM("Waiting for bootloader to complete...");
        ros::Duration(1.0).sleep();

        // Initialise encoder filter.
        ROS_INFO("Initialising LX16A encoder filters...");
        ROS_INFO_STREAM("LX16A encoder: classifier_filename: " << classifier_filename);
        ROS_INFO_STREAM("LX16A encoder: regressor_filename: " << regressor_filename);
        ROS_INFO_STREAM("LX16A encoder: classifier_window: " << classifier_window);

// Select the encoder filter type (service or in-proc)
#if 0
        std::unique_ptr<lx16a::LX16AEncoderFilterClient> filter(
            new lx16a::LX16AEncoderFilterClient(
                nh, classifier_filename, regressor_filename, classifier_window));
        ros::Duration encoder_timeout(encoder_service_timeout);
        filter->setTimeout(encoder_timeout);
        encoder_filter_ = std::move(filter);
#else
        std::unique_ptr<lx16a::LX16AEncoderFilterPython> filter(
            new lx16a::LX16AEncoderFilterPython(
                classifier_filename, regressor_filename, classifier_window));
        encoder_filter_ = std::move(filter);
#endif

        try
        {
            encoder_filter_->init();
            encoder_filter_->add_v(wheel_servo_ids_);
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_FATAL_STREAM(e.what());
        }
        
        // Initialise steering trim (offsets)
        ROS_INFO_STREAM("Setting steer trim...");
        for (size_t i=0; i<k_num_steers_; ++i)
        {
            setSteerTrim(i, static_cast<int16_t>(steer_servo_angle_offsets[i]));
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
        try
        {
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
        catch(const lx16a::LX16AException &e)
        {
            std::stringstream ss;
            ss << "get wheel position: id: " << static_cast<int>(servo_id)
            << " failed\n" << e.what();
            throw RoverBaseHALException(ss.str().c_str());
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

        // Apply a simple clamp to the duty
        duty = clamp(duty, -SERVO_DUTY_MAX, SERVO_DUTY_MAX);

        // Save the duty set point.
        wheel_servo_duties_[i] = duty;

        // Update the servo.
        try
        {
            servo_driver_->setMotorMode(servo_id, duty);   

            ROS_DEBUG_STREAM("set wheel: id: " << static_cast<int>(servo_id)
                << ", vel: " << velocity
                << ", orient: " << static_cast<int>(orientation)
                << ", duty: " << static_cast<int>(duty));
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_ERROR_STREAM(e.what()
                << " set wheel: id: " << static_cast<int>(servo_id));
        }
    }

    double RoverBaseHALLX16A::getSteerAngle(const ros::Time &time, int i) const
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int8_t orientation = steer_servo_orientations_[i];

        try
        {
            int16_t pos = servo_driver_->getPosition(servo_id);
            double angle = servoPos2Angle(pos) * orientation;        

            ROS_DEBUG_STREAM("get steer: id: " << static_cast<int>(servo_id)
                << ", angle: " << angle
                << ", pos: " << static_cast<int>(pos));

            return angle;
        }
        catch(const lx16a::LX16AException &e)
        {
            std::stringstream ss;
            ss << "get steer angle: id: " << static_cast<int>(servo_id)
            << " failed\n" << e.what();
            throw RoverBaseHALException(ss.str().c_str());
        }
    }

    void RoverBaseHALLX16A::setSteerAngle(const ros::Time &time, int i, double angle)
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int8_t orientation = steer_servo_orientations_[i];
        int16_t pos = angle2ServoPos(angle * orientation);

        try
        {
            servo_driver_->move(servo_id, pos, SERVO_MOVE_TIME);

            ROS_DEBUG_STREAM("set steer: id: " << static_cast<int>(servo_id)
                << ", angle: " << angle
                << ", pos: " << static_cast<int>(pos));
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_ERROR_STREAM(e.what()
                << " set steer: id: " << static_cast<int>(servo_id));
        }
    }

    void RoverBaseHALLX16A::getWheelPositions(const ros::Time &time, std::vector<double>& positions) const
    {
        try
        {
            // Get servo positions.
            std::vector<int16_t> servo_positions(k_num_wheels_, 0.0);
            for (size_t i=0; i<k_num_wheels_; ++i)
            {
                uint8_t servo_id = wheel_servo_ids_[i];
                int16_t pos = servo_driver_->getPosition(servo_id);
                servo_positions[i] = pos;
            }

            // Update filter.
            encoder_filter_->update_v(
                wheel_servo_ids_,
                time,
                wheel_servo_duties_,
                servo_positions,
                positions);

            // Adjust for orientation
            for (size_t i=0; i<k_num_wheels_; ++i)
            {
                positions[i] *= wheel_servo_orientations_[i];
            }
        }
        catch(const lx16a::LX16AException &e)
        {
            std::stringstream ss;
            ss << "get wheel positions failed\n" << e.what();
            throw RoverBaseHALException(ss.str().c_str());
        }
    }

    void RoverBaseHALLX16A::getWheelVelocities(const ros::Time &time, std::vector<double>& velocities) const
    {
        for (size_t i=0; i<k_num_wheels_; ++i)
        {
            velocities[i] = getWheelVelocity(time, i);
        }
    }

    void RoverBaseHALLX16A::setWheelVelocities(const ros::Time &time, const std::vector<double>& velocities)
    {
        for (size_t i=0; i<k_num_wheels_; ++i)
        {
            setWheelVelocity(time, i, velocities[i]);
        }
    }

    void RoverBaseHALLX16A::getSteerAngles(const ros::Time &time, std::vector<double>& positions) const
    {
        for (size_t i=0; i<k_num_steers_; ++i)
        {
            positions[i] = getSteerAngle(time, i);
        }
    }

    void RoverBaseHALLX16A::setSteerAngles(const ros::Time &time, const std::vector<double>& positions)
    {
        for (size_t i=0; i<k_num_steers_; ++i)
        {
            setSteerAngle(time, i, positions[i]);
        }
    }

    void RoverBaseHALLX16A::setSteerTrim(int i, int16_t offset)
    {
        uint8_t servo_id = steer_servo_ids_[i];
        int8_t orientation = steer_servo_orientations_[i];

        try
        {
            servo_driver_->setPositionOffset(servo_id, offset);
            servo_driver_->savePositionOffset(servo_id);

            ROS_DEBUG_STREAM("set steer: id: " << static_cast<int>(servo_id)
                << ", offset: " << static_cast<int>(offset));
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_ERROR_STREAM(e.what()
                << " set steer: id: " << static_cast<int>(servo_id));
        }
    }

} // namespace curio_base
