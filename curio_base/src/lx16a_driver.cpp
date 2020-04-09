//
//  Software License Agreement (BSD-3-Clause)
//   
//  Copyright (c) 2019 Rhys Mainwaring
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

#include "curio_base/lx16a_driver.h"

#include <ros/ros.h>

#include <cstdarg>
#include <iostream>
#include <string>
#include <sstream>

#define LX16A_DEBUG

namespace curio_base
{
    const uint8_t SERVO_BUS_HEADER            = 0x55;
    const uint8_t SERVO_BUS_MIN_ID            = 0x00;
    const uint8_t SERVO_BUS_MAX_ID            = 0xFE;
    const uint8_t SERVO_BUS_BROADCAST_ID      = 0xFE;
    
    const uint8_t SERVO_MOVE_TIME_WRITE       = 1;  // 0x01
    const uint8_t SERVO_MOVE_TIME_READ        = 2;  // 0x02
    const uint8_t SERVO_MOVE_TIME_WAIT_WRITE  = 7;  // 0x07
    const uint8_t SERVO_MOVE_TIME_WAIT_READ   = 8;  // 0x08
    const uint8_t SERVO_MOVE_START            = 11; // 0x0B
    const uint8_t SERVO_MOVE_STOP             = 12; // 0x0C
    const uint8_t SERVO_ID_WRITE              = 13; // 0x0D
    const uint8_t SERVO_ID_READ               = 14; // 0x0E
    const uint8_t SERVO_ANGLE_OFFSET_ADJUST   = 17; // 0x11
    const uint8_t SERVO_ANGLE_OFFSET_WRITE    = 18; // 0x12
    const uint8_t SERVO_ANGLE_OFFSET_READ     = 19; // 0x13
    const uint8_t SERVO_ANGLE_LIMIT_WRITE     = 20; // 0x14
    const uint8_t SERVO_ANGLE_LIMIT_READ      = 21; // 0x15
    const uint8_t SERVO_VIN_LIMIT_WRITE       = 22; // 0x16
    const uint8_t SERVO_VIN_LIMIT_READ        = 23; // 0x17
    const uint8_t SERVO_TEMP_MAX_LIMIT_WRITE  = 24; // 0x18
    const uint8_t SERVO_TEMP_MAX_LIMIT_READ   = 25; // 0x19
    const uint8_t SERVO_TEMP_READ             = 26; // 0x1A
    const uint8_t SERVO_VIN_READ              = 27; // 0x1B
    const uint8_t SERVO_POS_READ              = 28; // 0x1C
    const uint8_t SERVO_OR_MOTOR_MODE_WRITE   = 29; // 0x1D
    const uint8_t SERVO_OR_MOTOR_MODE_READ    = 30; // 0x1E
    const uint8_t SERVO_LOAD_OR_UNLOAD_WRITE  = 31; // 0x1F
    const uint8_t SERVO_LOAD_OR_UNLOAD_READ   = 32; // 0x20
    const uint8_t SERVO_LED_CTRL_WRITE        = 33; // 0x21
    const uint8_t SERVO_LED_CTRL_READ         = 34; // 0x22
    const uint8_t SERVO_LED_ERROR_WRITE       = 35; // 0x23
    const uint8_t SERVO_LED_ERROR_READ        = 36; // 0x24

    LX16ADriver::~LX16ADriver()
    {
    }

    LX16ADriver::LX16ADriver()
    {
    }

    void LX16ADriver::open()
    {
        serial_.open();
    }

    void LX16ADriver::close()
    {
        if(serial_.isOpen())
        {
            serial_.close();
        }
    }

    bool LX16ADriver::isOpen() const
    {
        return serial_.isOpen();
    }

    std::string LX16ADriver::getPort() const
    {
        return serial_.getPort();
    }

    void LX16ADriver::setPort(const std::string &port)
    {
        serial_.setPort(port);
    }

    uint32_t LX16ADriver::getBaudrate() const
    {
        return serial_.getBaudrate();
    }

    void LX16ADriver::setBaudrate(uint32_t baudrate)
    {
        serial_.setBaudrate(baudrate);
    }

    serial::Timeout LX16ADriver::getTimeout() const
    {
        return serial_.getTimeout();
    }

    void LX16ADriver::setTimeout(serial::Timeout &timeout)
    {
        serial_.setTimeout(timeout);
    }

    ros::Duration LX16ADriver::getResponseTimeout() const
    {
        return response_timeout_;
    }

    void LX16ADriver::setResponseTimeout(ros::Duration &response_timeout)
    {
        response_timeout_ = response_timeout;
    }

    bool LX16ADriver::moveTimeWrite(uint8_t servo_id, uint16_t servo_pos, uint16_t move_time)
    {    
        uint8_t pos_lsb = servo_pos & 0xff;
        uint8_t pos_hsb = (servo_pos >> 8) & 0xff;
        uint8_t move_time_lsb = move_time & 0xff;
        uint8_t move_time_hsb = (move_time >> 8) & 0xff;
        std::vector<uint8_t> data = { pos_lsb, pos_hsb, move_time_lsb, move_time_hsb };
        if (sendCommand(servo_id, 7, SERVO_MOVE_TIME_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveTimeWrite");
            return false;
        }
        return true;
    }

    bool LX16ADriver::moveTimeRead(uint8_t servo_id, int16_t &pos, int16_t &move_time)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_MOVE_TIME_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveTimeRead");
            return false;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 7, SERVO_MOVE_TIME_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: moveTimeRead");
            return false;
        }
        uint16_t pos_raw  = data[0] | (data[1] << 8);
        uint16_t move_time_raw = data[2] | (data[3] << 8);
        pos = static_cast<int16_t>(pos_raw);
        move_time = static_cast<int16_t>(move_time_raw);
        return true;
    }

    bool LX16ADriver::moveTimeWaitWrite(uint8_t servo_id, uint16_t servo_pos, uint16_t move_time)
    {
        uint8_t pos_lsb = servo_pos & 0xff;
        uint8_t pos_hsb = (servo_pos >> 8) & 0xff;
        uint8_t move_time_lsb = move_time & 0xff;
        uint8_t move_time_hsb = (move_time >> 8) & 0xff;
        std::vector<uint8_t> data = { pos_lsb, pos_hsb, move_time_lsb, move_time_hsb };
        if (sendCommand(servo_id, 7, SERVO_MOVE_TIME_WAIT_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: move_time_wait_write");
            return false;
        }
        return true;
    }

    uint16_t LX16ADriver::moveTimeWaitRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_ANGLE_OFFSET_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveTimeWaitRead");
            return -1;
        }
        std::vector<uint8_t> data;
        if(!readResponse(servo_id, 4, SERVO_ANGLE_OFFSET_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: moveTimeWaitRead");
            return -1;
        }
        uint16_t move_time_wait = data[0] | (data[1] << 8);
        return move_time_wait;
    }

    void LX16ADriver::moveStart(uint8_t servo_id)
    {
        if(sendCommand(servo_id, 3, SERVO_MOVE_START) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveStart");
        }
    }

    void LX16ADriver::moveStop(uint8_t servo_id)
    {
        if (sendCommand(servo_id, 3, SERVO_MOVE_STOP) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveStop");
        }
    }

    bool LX16ADriver::idWrite(uint8_t servo_id)
    {
        // TODO_IMPLEMENT
        return false;
    }

    uint8_t LX16ADriver::idRead(uint8_t servo_id)
    {
        // TODO_IMPLEMENT
        return SERVO_BUS_BROADCAST_ID;
    }

    bool LX16ADriver::angleOffsetAdjust(uint8_t servo_id, int16_t deviation)
    {
        uint8_t deviation_lsb = deviation & 0xff;
        std::vector<uint8_t> data = { deviation_lsb };
        if (sendCommand(servo_id, 4, SERVO_ANGLE_OFFSET_ADJUST, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: angleOffsetAdjust");
            return false;
        }
        return true;
    }

    bool LX16ADriver::angleOffsetWrite(uint8_t servo_id)
    {
        if (sendCommand(servo_id, 3, SERVO_ANGLE_OFFSET_WRITE) == 0)
        {
            ROS_WARN_STREAM("Servo command error: angleOffsetWrite");
            return false;
        }
        return true;
    }

    int16_t LX16ADriver::angleOffsetRead(int8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_ANGLE_OFFSET_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: angleOffsetRead");
            return -1;
        }
        std::vector<uint8_t> data;
        if(!readResponse(servo_id, 4, SERVO_ANGLE_OFFSET_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: angleOffsetRead");
            return -1;
        }
        uint8_t angle_offset_raw = data[0];
        return static_cast<int16_t>(angle_offset_raw);
    }

    bool LX16ADriver::angleLimitWrite(uint8_t servo_id, uint16_t min_angle, uint16_t max_angle)
    {
        uint8_t min_angle_lsb = min_angle & 0xff;
        uint8_t min_angle_hsb = (min_angle >> 8) & 0xff;
        uint8_t max_angle_lsb = max_angle & 0xff;
        uint8_t max_angle_hsb = (max_angle >> 8) & 0xff;
        std::vector<uint8_t> data = { min_angle_lsb, min_angle_hsb, max_angle_lsb, max_angle_hsb };
        if (sendCommand(servo_id, 7, SERVO_ANGLE_LIMIT_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: angleLimitWrite");
            return false;
        }
        return true;
    }

    bool LX16ADriver::angleLimitRead(uint8_t servo_id, uint16_t &min_angle, uint16_t &max_angle)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_ANGLE_LIMIT_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: angleLimitRead");
            return false;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 7, SERVO_ANGLE_LIMIT_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: angleLimitRead");
            return false;
        }
        uint16_t min_angle_raw = data[0] + (data[1] << 8);
        uint16_t max_angle_raw = data[2] + (data[3] << 8);
        min_angle = min_angle_raw;
        max_angle = max_angle_raw;
        return true;
    }

    bool LX16ADriver::vinLimitWrite(uint8_t servo_id, double min_vin, double max_vin)
    {
        uint16_t min_vin_int = uint16_t(min_vin * 1000);
        uint16_t max_vin_int = uint16_t(max_vin * 1000);
        uint8_t min_vin_lsb = min_vin_int & 0xff;
        uint8_t min_vin_hsb = (min_vin_int >> 8) & 0xff;
        uint8_t max_vin_lsb = max_vin_int & 0xff;
        uint8_t max_vin_hsb = (max_vin_int >> 8) & 0xff;
        std::vector<uint8_t> data = { min_vin_lsb, min_vin_hsb, max_vin_lsb, max_vin_hsb };
        if (sendCommand(servo_id, 7, SERVO_VIN_LIMIT_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: vinLimitWrite");
            return false;
        }
        return true;
    }

    bool LX16ADriver::vinLimitRead(uint8_t servo_id, double &min_vin, double &max_vin)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_VIN_LIMIT_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: vinLimitRead");
            return false;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 7, SERVO_VIN_LIMIT_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: vinLimitRead");
            return false;
        }
        uint16_t min_vin_raw = data[0] | (data[1] << 8);
        uint16_t max_vin_raw = data[2] | (data[3] << 8);
        min_vin = static_cast<double>(min_vin_raw) / 1000.0;
        max_vin = static_cast<double>(max_vin_raw) / 1000.0;
        return true;
    }

    void LX16ADriver::tempMaxLimitWrite(uint8_t servo_id, uint8_t max_temp)
    {
        uint8_t max_temp_lsb = max_temp & 0xff;
        std::vector<uint8_t> data(max_temp_lsb);
        if (sendCommand(servo_id, 4, SERVO_TEMP_MAX_LIMIT_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: tempMaxLimitWrite");
        }
    }

    int16_t LX16ADriver::tempMaxLimitRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_TEMP_MAX_LIMIT_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: tempMaxLimitRead");
            return -1;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 4, SERVO_TEMP_MAX_LIMIT_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: tempMaxLimitRead");
            return -1;
        }
        uint8_t temp_max_limit = data[0];
        return static_cast<int16_t>(temp_max_limit);
    }

    int16_t LX16ADriver::tempRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_TEMP_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: tempRead");
            return -1;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 4, SERVO_TEMP_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: tempRead");
            return -1;
        }
        uint8_t temp = data[0];
        return static_cast<int16_t>(temp);
    }

    double LX16ADriver::vinRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_VIN_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: vinRead");
            return -1.0;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 5, SERVO_VIN_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: vinRead");
            return -1.0;
        }
        uint16_t vin_raw = data[1] << 8 | data[0];
        return static_cast<double>(vin_raw) / 1000.0;
    }

    int16_t LX16ADriver::posRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_POS_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: posRead");
            return -1;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 5, SERVO_POS_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: posRead");
            return -1;
        }
        uint16_t pos = data[1] << 8 | data[0];
        return static_cast<int16_t>(pos);
    }

    bool LX16ADriver::motorModeWrite(uint8_t servo_id, int16_t duty)
    {
        uint8_t duty_lsb = duty & 0xff;
        uint8_t duty_hsb = (duty >> 8) & 0xff; 
        std::vector<uint8_t> data = { 1, 0, duty_lsb, duty_hsb };
        if (sendCommand(servo_id, 7, SERVO_OR_MOTOR_MODE_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: motorModeWrite");
            return false;
        }
        return true;
    }

    void LX16ADriver::servoModeWrite(uint8_t servo_id)
    {
        std::vector<uint8_t> data = { 0, 0, 0, 0 };
        if (sendCommand(servo_id, 7, SERVO_OR_MOTOR_MODE_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: servoModeWrite");
        }
    }

    bool LX16ADriver::modeRead(uint8_t servo_id, uint8_t &mode, int16_t &duty)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_OR_MOTOR_MODE_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: modeRead");
            return false;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 7, SERVO_OR_MOTOR_MODE_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: modeRead");
            return false;
        }
        mode = data[0];
        uint16_t duty_raw = data[3] << 8 | data[2];
        duty = static_cast<int16_t>(duty_raw);
        return true;
    }

    void LX16ADriver::loadOrUnloadWrite(uint8_t servo_id, uint8_t is_loaded)
    {
        uint8_t is_loaded_lsb = is_loaded & 0xff;
        std::vector<uint8_t> data = { is_loaded_lsb };
        if (sendCommand(servo_id, 4, SERVO_LOAD_OR_UNLOAD_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: loadOrUnloadWrite");
        }
    }

    uint8_t LX16ADriver::loadOrUnloadRead(uint8_t servo_id)
    {
        if (sendCommand(servo_id, 3, SERVO_LOAD_OR_UNLOAD_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: loadOrUnloadRead");
            return false;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 4, SERVO_LOAD_OR_UNLOAD_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: loadOrUnloadRead");
            return false;
        }
        uint8_t load_or_unload = data[0];
        return load_or_unload;
    }

    void LX16ADriver::ledCtrlWrite(uint8_t servo_id, uint8_t is_light_off)
    {
        uint8_t is_light_off_lsb = is_light_off & 0xff;
        std::vector<uint8_t> data = { is_light_off_lsb };
        if (sendCommand(servo_id, 4, SERVO_LED_CTRL_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: ledCtrlWrite");
        }
    }

    uint8_t LX16ADriver::ledCtrlRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_LED_CTRL_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: ledCtrlRead");
            return -1;
        }
        std::vector<uint8_t> data;
        if (!readResponse(servo_id, 4, SERVO_LED_CTRL_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: ledCtrlRead");
            return -1;
        }
        uint8_t is_light_off = data[0];
        return is_light_off;
    }

    void LX16ADriver::ledErrorWrite(uint8_t servo_id, uint8_t fault_code)
    {
        uint8_t fault_code_lsb = fault_code & 0xff;
        std::vector<uint8_t> data = { fault_code_lsb }; 
        if (sendCommand(servo_id, 4, SERVO_LED_ERROR_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: ledErrorWrite");
        }
    }

    uint8_t LX16ADriver::ledErrorRead(uint8_t servo_id)
    {
        serial_.flushInput();
        if (sendCommand(servo_id, 3, SERVO_LED_ERROR_READ) == 0)
        {
            ROS_WARN_STREAM("Servo command error: ledErrorRead");
            return -1;
        }
        std::vector<uint8_t> data; 
        if (!readResponse(servo_id, 4, SERVO_LED_ERROR_READ, data))
        {
            ROS_WARN_STREAM("Servo read error: ledErrorRead");
            return -1;
        }
        uint8_t fault_code = data[0];
        return fault_code;
    }

    uint8_t LX16ADriver::checksum(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data)
    {
        uint8_t checksum = servo_id + length + command;
        for (uint8_t d : data)
        {
            checksum += d;
        }
        checksum = (~checksum) & 0xff;
        return checksum;
    }

    bool LX16ADriver::readResponse(uint8_t servo_id, uint8_t length, uint8_t command, std::vector<uint8_t> &data)
    {
        // State machine states
        enum ReadState
        {
            READ_WAIT_FOR_RESPONSE = 0,
            READ_HEADER_1 = 1,
            READ_HEADER_2 = 2,
            READ_SERVO_ID = 3,
            READ_LENGTH = 4,
            READ_COMMAND = 5,
            READ_DATA = 6,
            READ_CHECKSUM = 7,
            READ_COMPLETE = 8,
            READ_ERROR = 9,
            READ_TIMED_OUT = 10,
        };

        // Check port is open
        if (!serial_.isOpen())
        {
            ROS_WARN_STREAM("Serial port not open");
            return false;
        }

        // Initialise state machine
        ReadState state = READ_WAIT_FOR_RESPONSE;
        std::stringstream error_msg;
        error_msg << "No response";
        const ros::Time start = ros::Time::now();         
        while (true) 
        {
            // Check for timeout
            ros::Time time_now = ros::Time::now();
            if (time_now - start > response_timeout_) 
            {
                state = READ_TIMED_OUT;
            }
            
            // State machine transitions
            uint8_t byte, cs;
            switch (state)
            {
            case READ_WAIT_FOR_RESPONSE:
            {
                if (serial_.available() > 0)
                {
                    state = READ_HEADER_1; 
                }
                // ROS_INFO_STREAM("READ_WAIT_FOR_RESPONSE: " << (time_now - start).toSec());
                break;
            }
            case READ_HEADER_1:
            {
                size_t bytes_read = serial_.read(&byte, 1);
                // ROS_INFO_STREAM("READ_HEADER_1: 0x" << static_cast<int>(byte));
                if (bytes_read == 1 && byte == SERVO_BUS_HEADER)
                {
                    state = READ_HEADER_2;
                }
                else
                {
                    state = READ_HEADER_1;
                    error_msg.str("");
                    error_msg << "Invalid 1st header byte: expecting: 0x" << std::hex << static_cast<int>(SERVO_BUS_HEADER)
                        << " got: 0x" << std::hex << static_cast<int>(byte);
                }
                break;
            }
            case READ_HEADER_2:
            {
                size_t bytes_read = serial_.read(&byte, 1);
                // ROS_INFO_STREAM("READ_HEADER_2: 0x" << static_cast<int>(byte));
                if (bytes_read == 1 && byte == SERVO_BUS_HEADER)
                {
                    state = READ_SERVO_ID;
                }
                else
                {
                    state = READ_HEADER_1;
                    error_msg.str("");
                    error_msg << "Invalid 2nd header byte: expecting: 0x" << std::hex << static_cast<int>(SERVO_BUS_HEADER)
                        << " got: 0x" << std::hex << static_cast<int>(byte);
                }
                break;
            }
            case READ_SERVO_ID:
            {
                size_t bytes_read = serial_.read(&byte, 1);
                // ROS_INFO_STREAM("READ_SERVO_ID: 0x" << static_cast<int>(byte));
                if (bytes_read == 1 && byte == servo_id)
                {
                    state = READ_LENGTH;
                }
                else
                {
                    state = READ_HEADER_1;
                    error_msg.str("");
                    error_msg << "Invalid servo_id: expecting: 0x" << std::hex << static_cast<int>(servo_id)
                        << " got: 0x" << std::hex << static_cast<int>(byte);
                }
                break;
            }
            case READ_LENGTH:
            {
                size_t bytes_read = serial_.read(&byte, 1);
                // ROS_INFO_STREAM("READ_LENGTH: 0x" << static_cast<int>(byte));
                if (bytes_read == 1 && byte == length)
                {
                    state = READ_COMMAND;
                }
                else
                {
                    state = READ_HEADER_1;
                    error_msg.str("");
                    error_msg << "Invalid length: expecting: 0x" << std::hex << static_cast<int>(length)
                        << " got: 0x" << std::hex << static_cast<int>(byte);
                }
                break;
            }
            case READ_COMMAND:
            {
                size_t bytes_read = serial_.read(&byte, 1);
                // ROS_INFO_STREAM("READ_COMMAND: 0x" << static_cast<int>(byte));
                if (bytes_read == 1 && byte == command)
                {
                    state = READ_DATA;
                }
                else
                {
                    state = READ_HEADER_1;
                    error_msg.str("");
                    error_msg << "Invalid command: expecting: 0x" << std::hex << static_cast<int>(command)
                        << " got: 0x" << std::hex << static_cast<int>(byte);
                }
                break;
            }
            case READ_DATA:
            {
                size_t bytes_read = serial_.read(data, length - 3);
                if (bytes_read == length - 3)
                {
                    state = READ_CHECKSUM;
                }
                else
                {
                    state = READ_ERROR;
                    error_msg.str("");
                    error_msg << "Invalid data size: expecting: " << length - 3
                        << " got: " << bytes_read;
                }
                break;
            }
            case READ_CHECKSUM:
            {
                cs = checksum(servo_id, length, command, data);
                size_t bytes_read = serial_.read(&byte, 1);
                // ROS_INFO_STREAM("READ_CHECKSUM: 0x" << static_cast<int>(byte));
                if (bytes_read == 1 && byte == cs)
                {
                    state = READ_COMPLETE;
                }
                else
                {
                    state = READ_ERROR;
                    error_msg.str("");
                    error_msg << "Invalid checksum: expecting: 0x" << std::hex << static_cast<int>(cs)
                        << " got: 0x" << std::hex << static_cast<int>(byte);
                }
                break;
            }
            case READ_COMPLETE:
            {
#if defined(LX16A_DEBUG)
                std::stringstream ss;
                ss << "RX:";
                ss << " 0x" << std::hex << static_cast<int>(SERVO_BUS_HEADER);
                ss << " 0x" << std::hex << static_cast<int>(SERVO_BUS_HEADER);
                ss << " 0x" << std::hex << static_cast<int>(servo_id);
                ss << " 0x" << std::hex << static_cast<int>(length);
                ss << " 0x" << std::hex << static_cast<int>(command);
                for (uint8_t d : data)
                {
                    ss  << " 0x" << std::hex << static_cast<int>(d);
                }
                ss << " 0x" << std::hex << static_cast<int>(cs);
                ss << " (" << std::dec << static_cast<int>(1000*(time_now - start).toSec()) << "ms)";
                ROS_INFO_STREAM(ss.str());
#endif
                return true;
            }
            case READ_ERROR:
            {
                ROS_WARN_STREAM("ERROR:"
                    << " id: " << static_cast<int>(servo_id)
                    << ", cmd: 0x" << std::hex << command
                    << ", msg: " << std::hex << error_msg.str());
                return false;
            }
            case READ_TIMED_OUT:
            {
                ROS_WARN_STREAM("TIMED_OUT:"
                    << " id: " << static_cast<int>(servo_id)
                    << ", cmd: 0x" << std::hex << static_cast<int>(command)
                    << ", msg: " << std::hex << error_msg.str());
                return false;
            }
            default:
            {
                ROS_ERROR_STREAM("INVALID STATE IN LX16A DRIVER");
                return false;
            }
            }
        }
    }

    size_t LX16ADriver::sendCommand(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data)
    {
        // Check the port is open
        if (!serial_.isOpen())
        {
            return 0;
        }

        // Packet header [0x55, 0x55]
        std::vector<uint8_t> packet = { SERVO_BUS_HEADER, SERVO_BUS_HEADER }; 

        // Check the servo id is in range
        if (servo_id < SERVO_BUS_MIN_ID || servo_id > SERVO_BUS_MAX_ID)
        {
            return 0;
        }
        packet.push_back(servo_id);

        // Check the data is consistent with the specfied packet length
        if (length != 3 + data.size())
        { 
            return 0;
        }
        packet.push_back(length);

        // Append command to packet
        packet.push_back(command);

        // Append parameter data to packet
        for (uint8_t d : data)
        {
            packet.push_back(d);
        }

        // Calculate checksum and append to packet
        uint8_t cs = checksum(servo_id, length, command, data);
        packet.push_back(cs);

        // Send command
        size_t bytes_sent = serial_.write(packet);

        // DEBUG_INFO
#if defined(LX16A_DEBUG)
        std::stringstream ss;
        ss << "TX:";
        for (uint8_t v: packet)
        {
            ss << " 0x"<<  std::hex << static_cast<int>(v);
        }
        ROS_INFO_STREAM(ss.str());
        // ROS_INFO_STREAM("bytes_sent: " << bytes_sent);
#endif
        return bytes_sent;
    }

} // namespace curio_base

