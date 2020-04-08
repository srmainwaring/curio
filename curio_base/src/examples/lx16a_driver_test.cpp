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

// #include "curio_base/lx16a_driver.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <serial/serial.h>

#include <signal.h>

#include <cstddef>
#include <cstdint>
#include <thread>

// #define LX16A_DEBUG

////////////////////////////////////////////////////////////////////////
// LX16ADriver (version 2)
//
// This version is a port of the Python driver lx16a_driver.py

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

namespace curio_base
{
   class LX16ADriver
    {
    public:
        /// \brief Destructor
        ~LX16ADriver()
        {
        }

        /// \brief Constructor
        LX16ADriver()
        {
        }

    /// \brief Open the serial port for communication.
    void open()
    {
        serial_.open();
    }

    /// \brief Close the serial port immediatelty.
    void close()
    {
        if(serial_.isOpen())
        {
            serial_.close();
        }
    }

    /// \brief Return True if the serial port is open.
    /// 
    /// Returns
    /// -------
    /// bool
    ///     True if the port is open
    ///
    bool isOpen() const
    {
        return serial_.isOpen();
    }

    /// \brief Get the serial device.
    /// 
    /// Returns
    /// -------
    /// string
    ///     The name of the serial device.
    ///
    std::string getPort() const
    {
        return serial_.getPort();
    }

    /// \brief Set the serial device.
    /// 
    /// Parameters
    /// ----------
    /// port : str
    ///     The name of the serial device.
    ///
    void setPort(const std::string &port)
    {
        serial_.setPort(port);
    }

    /// \brief Get the baudrate.
    /// 
    /// Returns
    /// -------
    /// int
    ///     The baudrate for the serial connection.
    ///
    uint32_t getBaudrate() const
    {
        return serial_.getBaudrate();
    }

    /// \brief Set the baudrate.
    /// 
    /// Parameters
    /// ----------
    /// baudrate : int
    ///     The baudrate for the serial connection.
    ///
    void setBaudrate(uint32_t baudrate)
    {
        serial_.setBaudrate(baudrate);
    }

    /// \brief Get the serial timeout [s]
    /// 
    /// Returns
    /// -------
    /// float
    ///     The serial port timeout.
    ///
    serial::Timeout getTimeout() const
    {
        return serial_.getTimeout();
    }

    /// \brief Set the serial timeout [s]
    /// 
    /// Parameters
    /// -------
    /// timeout : float
    ///     The serial port timeout in seconds.
    ///
    void setTimeout(serial::Timeout &timeout)
    {
        serial_.setTimeout(timeout);
    }

    //  Commands

    /// \brief  Move a servo to the commanded position in a given time.
    /// 
    /// Parameters
    /// ----------
    /// servo_id : int
    ///     The servo serial identifier, in [0, 253]
    /// servo_pos : int
    ///     The commanded servo position, in [0, 1000]
    /// move_time : int
    ///      The desired time for the servo to move from its
    ///      position to the commanded position, in [0, 30000] ms
    ///
    bool moveTimeWrite(uint8_t servo_id, uint16_t servo_pos, uint16_t move_time=0)
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

    /// \brief Read the current commanded position and move time.
    /// 
    /// Parameters
    /// ----------
    /// servo_id : int
    ///     The servo serial identifier, in [0, 253]
    /// 
    /// Returns:
    /// list
    ///     A two element list containing:
    ///     servo_pos : int
    ///         The commanded servo position, in [0, 1000]
    ///     move_time : int
    ///         The desired time for the servo to move from its
    ///         position to the commanded position, in [0, 30000] ms
    ///
    bool moveTimeRead(uint8_t servo_id, int16_t &pos, int16_t &move_time)
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

    bool moveTimeWaitWrite(uint8_t servo_id, uint16_t servo_pos, uint16_t move_time)
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

    // # @TOOD: there appears to be a problem with this command
    // #        - the response from the servo is badly formed (and it also
    // #        affects the execution of the write version of this command)
    // def move_time_wait_read(self, servo_id):
    //     # @TODO investigate issue further
    //     serial_.flushInput()
    //     if sendCommand(servo_id, 3, SERVO_MOVE_TIME_WAIT_READ) == -1:
    //         ROS_WARN_STREAM('Servo command error: move_time_wait_read')
    //         return -1, -1
    //     data = readResponse(servo_id, 7, SERVO_MOVE_TIME_WAIT_READ)
    //     if data == -1:
    //         ROS_WARN_STREAM('Servo read error: move_time_wait_read')
    //         return -1, -1
    //     pos  = data[0] + (data[1] << 8)
    //     move_time = data[2] + (data[3] << 8)
    //     return pos, move_time

    void moveStart(uint8_t servo_id)
    {
        if(sendCommand(servo_id, 3, SERVO_MOVE_START) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveStart");
        }
    }

    // @NOTE: The servo must be in servo mode for the move_stop to be
    //        effective. When in motor_mode you must set the speed to
    //        zero instead.
    void moveStop(uint8_t servo_id)
    {
        if (sendCommand(servo_id, 3, SERVO_MOVE_STOP) == 0)
        {
            ROS_WARN_STREAM("Servo command error: moveStop");
        }
    }

    // def id_write(self, servo_id):
    //     # @TODO implement
    //     pass

    // def id_read(self, servo_id):
    //     # @TODO implement
    //     pass

    bool angleOffsetAdjust(uint8_t servo_id, int16_t deviation)
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

    bool angleOffsetWrite(uint8_t servo_id)
    {
        if (sendCommand(servo_id, 3, SERVO_ANGLE_OFFSET_WRITE) == 0)
        {
            ROS_WARN_STREAM("Servo command error: angleOffsetWrite");
            return false;
        }
        return true;
    }

    int16_t angleOffsetRead(int8_t servo_id)
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

    bool angleLimitWrite(uint8_t servo_id, uint16_t min_angle, uint16_t max_angle)
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

    bool angleLimitRead(uint8_t servo_id, uint16_t &min_angle, uint16_t &max_angle)
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

    bool vinLimitWrite(uint8_t servo_id, double min_vin, double max_vin)
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

    bool vinLimitRead(uint8_t servo_id, double &min_vin, double &max_vin)
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

    void tempMaxLimitWrite(uint8_t servo_id, uint8_t max_temp)
    {
        uint8_t max_temp_lsb = max_temp & 0xff;
        std::vector<uint8_t> data(max_temp_lsb);
        if (sendCommand(servo_id, 4, SERVO_TEMP_MAX_LIMIT_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: tempMaxLimitWrite");
        }
    }

    int16_t tempMaxLimitRead(uint8_t servo_id)
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

    int16_t tempRead(uint8_t servo_id)
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

    double vinRead(uint8_t servo_id)
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

    int16_t posRead(uint8_t servo_id)
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

    bool motorModeWrite(uint8_t servo_id, int16_t duty)
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

    void servoModeWrite(uint8_t servo_id)
    {
        std::vector<uint8_t> data = { 0, 0, 0, 0 };
        if (sendCommand(servo_id, 7, SERVO_OR_MOTOR_MODE_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: servoModeWrite");
        }
    }

    bool modeRead(uint8_t servo_id, uint8_t &mode, int16_t &duty)
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

    void loadOrUnloadWrite(uint8_t servo_id, uint8_t is_loaded)
    {
        uint8_t is_loaded_lsb = is_loaded & 0xff;
        std::vector<uint8_t> data = { is_loaded_lsb };
        if (sendCommand(servo_id, 4, SERVO_LOAD_OR_UNLOAD_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: loadOrUnloadWrite");
        }
    }

    // # @TODO: there appears to be a problem with this command
    // #        - there is no response to the read request command. 
    // def load_or_unload_read(self, servo_id):
    //     # @TODO investigate issue further
    //     serial_.flushInput()
    //     if sendCommand(servo_id, 3, SERVO_LOAD_OR_UNLOAD_READ) == -1:
    //         ROS_WARN_STREAM('Servo command error: load_or_unload_read')
    //         return -1
    //     data = readResponse(servo_id, 4, SERVO_LOAD_OR_UNLOAD_READ)
    //     if data == -1:
    //         ROS_WARN_STREAM('Servo read error: load_or_unload_read')
    //         return -1
    //     load_or_unload = data[0]
    //     return load_or_unload  

    void ledCtrlWrite(uint8_t servo_id, uint8_t is_light_off)
    {
        uint8_t is_light_off_lsb = is_light_off & 0xff;
        std::vector<uint8_t> data = { is_light_off_lsb };
        if (sendCommand(servo_id, 4, SERVO_LED_CTRL_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: ledCtrlWrite");
        }
    }

    uint8_t ledCtrlRead(uint8_t servo_id)
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

    void ledErrorWrite(uint8_t servo_id, uint8_t fault_code)
    {
        uint8_t fault_code_lsb = fault_code & 0xff;
        std::vector<uint8_t> data = { fault_code_lsb }; 
        if (sendCommand(servo_id, 4, SERVO_LED_ERROR_WRITE, data) == 0)
        {
            ROS_WARN_STREAM("Servo command error: ledErrorWrite");
        }
    }

    uint8_t ledErrorRead(uint8_t servo_id)
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

    // Serial communication protocol

    static uint8_t checksum(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data)
    {
        uint8_t checksum = servo_id + length + command;
        for (uint8_t d : data)
        {
            checksum += d;
        }
        checksum = (~checksum) & 0xff;
        return checksum;
    }

    uint8_t readByte()
    {
        uint8_t byte;
        size_t bytes_read = serial_.read(&byte, 1);
        if (bytes_read != 1)
        {
            ROS_ERROR_STREAM("Serial read error, expecting 1 byte: got: " << bytes_read);
        }
        return byte;
    }

    size_t readByteArray(std::vector<uint8_t> &buffer, size_t size=1)
    {
        return serial_.read(buffer, size);
    }

    bool readResponse(uint8_t servo_id, uint8_t length, uint8_t command, std::vector<uint8_t> &data)
    {
        // Check port is open
        if (!serial_.isOpen())
        {
            ROS_WARN_STREAM("Serial port not open");
            return false;
        }

        // Read header (2 bytes)
        uint8_t byte = readByte();
        if (byte != SERVO_BUS_HEADER)
        {
            ROS_WARN_STREAM("Invalid 1st header byte: expecting: " << SERVO_BUS_HEADER
                << ", got: " << byte);
            return false;
        }

        byte = readByte();
        if (byte != SERVO_BUS_HEADER)
        {
            ROS_WARN_STREAM("Invalid 2nd header byte: expecting: " << SERVO_BUS_HEADER
                << ", got: " << byte);
            return false;
        }

        // Read id
        byte = readByte();
        if (byte != servo_id)
        {
            ROS_WARN_STREAM("Invalid servo_id: expecting: " << servo_id 
                << ", got: " << byte);
            return false;
        }

        // Read length
        byte = readByte();
        if (byte != length)
        {
            ROS_WARN_STREAM("Invalid length: expecting: " << length
                << ", got: " << byte);
            return false;
        }

        // Read command
        byte = readByte();
        if (byte != command)
        {
            ROS_WARN_STREAM("Invalid command: expecting: " << command
                << ", got: " << byte);
            return false;
        }

        // Read data. There should be length - 3 parameters in the data block
        size_t bytes_read = readByteArray(data, length - 3);
        if (bytes_read != length - 3)
        {
            ROS_WARN_STREAM("Invalid len(data): expecting: " << length - 3
                << ", got: " << bytes_read);
            return false;
        }

        // Calculate checksum
        uint8_t cs = checksum(servo_id, length, command, data);

        // Read checksum
        byte = readByte();
        if (byte != cs)
        {
            ROS_WARN_STREAM("Invalid checksum: expecting: " << cs
                << ", got: " << byte);
            return false;
        }

        // DEBUG_INFO
#if defined(LX16A_DEBUG)
        std::stringstream ss;
        ss << "RX:";
        ss << " 0x" << std::hex << static_cast<int>(SERVO_BUS_HEADER);
        ss << " 0x" << std::hex << static_cast<int>(SERVO_BUS_HEADER);
        ss << " 0x" << std::hex << static_cast<int>(servo_id);
        ss << " 0x" << std::hex << static_cast<int>(length);
        ss << " 0x" << std::hex << static_cast<int>(command);
        if (bytes_read)
        {
            for (uint8_t d : data)
            {
                ss  << " 0x" << std::hex << static_cast<int>(d);
            }
        }
        ss << " 0x" << std::hex << static_cast<int>(cs);
        ROS_INFO_STREAM(ss.str());
#endif
        return true;
    }

    size_t sendCommand(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data = std::vector<uint8_t>())
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
        ROS_INFO_STREAM("bytes_sent: " << bytes_sent);
#endif
        return bytes_sent;
    }

    private:
        serial::Serial serial_;

    };
}

////////////////////////////////////////////////////////////////////////

// Servo ids
const uint8_t SERVO_ID = 1;

// Servo driver
curio_base::LX16ADriver servo_driver;

void testReadVin()
{
    double vin = servo_driver.vinRead(SERVO_ID);
    int16_t pos = servo_driver.posRead(SERVO_ID);
    ROS_INFO_STREAM("vin: " << vin << ", pos: " << pos);

    // Read mode and duty
    uint8_t mode;
    int16_t duty;
    servo_driver.modeRead(SERVO_ID, mode, duty);
    ROS_INFO_STREAM("mode: " << static_cast<int>(mode) <<  ", duty: " << duty);
}

void testServoProperties(curio_base::LX16ADriver& servo_driver)
{
    ROS_INFO_STREAM("Test Servo Properties");

    //     // Display servo properties
    //     int vin = servo_driver.readVin(SERVO_ID);
    //     ROS_INFO_STREAM("vin: " << vin);

    //     // Run servo in motor (continuous) mode
    //     ROS_INFO_STREAM("Set motor speed");
    //     int duty = 800;
    //     int pos = servo_driver.readPosition(SERVO_ID);
    //     ROS_INFO_STREAM("position: " << pos);
    //     servo_driver.move(SERVO_ID, duty, 10);

    //     ros::Time start = ros::Time::now();
    //     ros::Duration run_duration = ros::Duration(2);      // 2s
    //     ros::Rate rate(500);                                // 500 Hz
    //     int count = 0;
    //     while (ros::Time::now() - start < run_duration)
    //     {
    //         pos = servo_driver.readPosition(SERVO_ID);
    //         ROS_INFO_STREAM("position: " << pos);
    //         count++;
    //         rate.sleep();
    //     }

    //     servo_driver.move(SERVO_ID, 0, 10);
    //     pos = servo_driver.readPosition(SERVO_ID);
    //     ROS_INFO_STREAM("position: " << pos);
    //     ROS_INFO_STREAM("read : " << count << " in: " << run_duration.toSec() << " s");

}

// Custom interrupt signal handler
void sigintHandler(int sig)
{
    // Do some custom action.
    ROS_INFO_STREAM("Stopping node lx16a_driver_test...");
  
    ROS_INFO_STREAM("Stop motor");
    servo_driver.motorModeWrite(SERVO_ID, 0);

    // Wait for serial out to clear.
    ros::Duration(1.0).sleep();


    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

// Entry point
int main(int argc, char *argv[])
{
    // Initialise node
    ros::init(argc, argv, "lx16a_driver_test", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh, private_nh("~");
    ROS_INFO("Starting node lx16a_driver_test...");

    // Register custom interrupt signal handler.
    signal(SIGINT, sigintHandler);

    // Parameters
    const std::string port("/dev/cu.usbmodem7197691");
    // const std::string port("/dev/cu.usbmodemFD5121");
    // const std::string port("/dev/cu.usbmodem136");
    // const std::string port("/dev/cu.SLAB_USBtoUART");
    const uint32_t baudrate = 115200;
    const uint32_t timeout = 1000;       // [ms]
    const uint32_t read_rate = 300;      // [Hz]
    double control_frequency = 50.0;     // [Hz]

    // Serial
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
    servo_driver.setPort(port);
    servo_driver.setBaudrate(baudrate);
    servo_driver.setTimeout(serial_timeout);
    servo_driver.open();
    ROS_INFO_STREAM("port: " << servo_driver.getPort());
    ROS_INFO_STREAM("baudrate: " << servo_driver.getBaudrate());
    ROS_INFO_STREAM("is_open: " << servo_driver.isOpen());

    // Wait for Arduino bootloader to complete before sending any
    // data on the serial connection.
    ROS_INFO_STREAM("Waiting for bootloader to complete...");
    ros::Duration(1.0).sleep();

    // Start motor
    servo_driver.motorModeWrite(SERVO_ID, 500);

    // Loop
    ros::Rate rate(read_rate);
    ros::Time start = ros::Time::now();
    uint32_t count = 0;
    while (ros::ok())
    {
        testReadVin();
        double sec = (ros::Time::now() - start).toSec();
        double cps = count/sec;
        count++;
        if (count > 100)
        {
            ROS_INFO_STREAM("reads/sec: " << cps);
            count = 0;
            start = ros::Time().now();
        }

        rate.sleep();
    }

    // Control loop timer
    // ros::Timer control_timer = nh.createTimer(
    //     ros::Duration(1.0 / control_frequency),
    //     controlLoop);

    // testServoProperties( servo_driver);

    // Start spinners
    // ros::spin();
    // spinner.start();
    ros::waitForShutdown();
    return 0;
}


