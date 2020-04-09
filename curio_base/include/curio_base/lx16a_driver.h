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

#ifndef CURIO_BASE_LX16A_DRIVER_H_
#define CURIO_BASE_LX16A_DRIVER_H_

#include <serial/serial.h>
#include <ros/ros.h>

#include <cstddef>
#include <cstdint>

namespace curio_base
{
   class LX16ADriver
    {
    public:
        /// \brief Destructor
        ~LX16ADriver();

        /// \brief Constructor
        LX16ADriver();

        /// \brief Open the serial port for communication.
        void open();

        /// \brief Close the serial port immediately.
        void close();

        /// \brief Get the open status of the serial port.
        /// 
        /// \return  Returns true if the port is open, false otherwise.
        ///
        bool isOpen() const;

        /// \brief Get the serial port identifier.
        /// 
        /// \return  Returns a std::string containing the serial port identifier.
        ///
        std::string getPort() const;

        /// \brief Set the serial port identifier.
        /// 
        /// \param port A const std::string containing the name of the serial port
        ///
        void setPort(const std::string &port);

        /// \brief Get the baudrate for the serial port.
        /// 
        /// \return An integer that sets the baud rate for the serial port
        ///
        uint32_t getBaudrate() const;

        /// \brief Set the baudrate for the serial port.
        /// 
        /// \param baudrate An integer that sets the baud rate for the serial port.
        ///
        void setBaudrate(uint32_t baudrate);

        /// \brief Get the serial timeout.
        /// 
        /// \return A serial::Timeout object specifiying the timeout for reads and writes.
        ///
        serial::Timeout getTimeout() const;

        /// \brief Set the serial timeout.
        /// 
        /// \param timeout A serial::Timeout object specifiying the timeout for reads and writes.
        ///
        void setTimeout(serial::Timeout &timeout);

        ros::Duration getResponseTimeout() const;

        void setResponseTimeout(ros::Duration &response_timeout);

        //  Commands

        /// \brief  Move a servo to the commanded position in a given time.
        /// 
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param servo_pos An integer to set the servo position, in [0, 1000]
        /// \param move_time An integer setting the desired time for the servo
        /// to move from its current position to the commanded position, in [0, 30000] ms.
        /// \return Returns true if succesful, false otherwise.
        ///
        bool moveTimeWrite(uint8_t servo_id, uint16_t servo_pos, uint16_t move_time=0);

        /// \brief Read the current commanded position and move time.
        /// 
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param pos An integer reference for the servo position, in [0, 1000]
        /// \param move_time An integer reference for the desired time for the servo
        /// to move from its current position to the commanded position, in [0, 30000] ms.
        /// \return Returns true if succesful, false otherwise.
        ///
        bool moveTimeRead(uint8_t servo_id, int16_t &pos, int16_t &move_time);

        bool moveTimeWaitWrite(uint8_t servo_id, uint16_t servo_pos, uint16_t move_time);

        uint16_t moveTimeWaitRead(uint8_t servo_id);

        void moveStart(uint8_t servo_id);

        /// The servo must be in servo mode for the move_stop to be effective.
        /// When in motor_mode you must set the speed to zero instead.
        void moveStop(uint8_t servo_id);

        bool idWrite(uint8_t servo_id);

        uint8_t idRead(uint8_t servo_id);

        bool angleOffsetAdjust(uint8_t servo_id, int16_t deviation);

        bool angleOffsetWrite(uint8_t servo_id);

        int16_t angleOffsetRead(int8_t servo_id);

        bool angleLimitWrite(uint8_t servo_id, uint16_t min_angle, uint16_t max_angle);

        bool angleLimitRead(uint8_t servo_id, uint16_t &min_angle, uint16_t &max_angle);

        bool vinLimitWrite(uint8_t servo_id, double min_vin, double max_vin);

        bool vinLimitRead(uint8_t servo_id, double &min_vin, double &max_vin);

        void tempMaxLimitWrite(uint8_t servo_id, uint8_t max_temp);

        int16_t tempMaxLimitRead(uint8_t servo_id);

        int16_t tempRead(uint8_t servo_id);

        double vinRead(uint8_t servo_id);

        int16_t posRead(uint8_t servo_id);

        bool motorModeWrite(uint8_t servo_id, int16_t duty);

        void servoModeWrite(uint8_t servo_id);

        bool modeRead(uint8_t servo_id, uint8_t &mode, int16_t &duty);

        void loadOrUnloadWrite(uint8_t servo_id, uint8_t is_loaded);

        uint8_t loadOrUnloadRead(uint8_t servo_id);

        void ledCtrlWrite(uint8_t servo_id, uint8_t is_light_off);

        uint8_t ledCtrlRead(uint8_t servo_id);

        void ledErrorWrite(uint8_t servo_id, uint8_t fault_code);

        uint8_t ledErrorRead(uint8_t servo_id);

    private:
        serial::Serial serial_;
        ros::Duration response_timeout_ = ros::Duration(0.01);  // [s]

        // Serial communication protocol

        static uint8_t checksum(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data);

        bool readResponse(uint8_t servo_id, uint8_t length, uint8_t command, std::vector<uint8_t> &data);

        size_t sendCommand(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data = std::vector<uint8_t>());
    };

} // namespace curio_base

#endif // CURIO_BASE_LX16A_DRIVER_H_
