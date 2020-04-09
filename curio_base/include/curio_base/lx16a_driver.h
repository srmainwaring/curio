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

#include <cstdint>

namespace curio_base
{
    /// Driver for the Lewansoul LX16A bus servos
    ///
    /// Acknowledgments
    ///     [Maxim Kulkin's LewanSoul LX-16A servos driver & GUI](https://github.com/maximkulkin/lewansoul-lx16a)
    ///     (Adopt the same function names for ease of porting from Python to C++)
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
        /// \param position An integer to set the servo position, in [0, 1000]
        /// \param move_time An integer setting the desired time for the servo
        /// to move from its current position to the commanded position, in [0, 30000] ms.
        /// \return Returns true if succesful, false otherwise.
        ///
        bool move(uint8_t servo_id, uint16_t position, uint16_t move_time=0);

        /// \brief Read the current commanded position and move time.
        /// 
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param position An integer reference for the servo position, in [0, 1000]
        /// \param move_time An integer reference for the desired time for the servo
        /// to move from its current position to the commanded position, in [0, 30000] ms.
        /// \return Returns true if succesful, false otherwise.
        ///
        bool getMove(uint8_t servo_id, int16_t &position, int16_t &move_time);

        /// \brief Prepare a servo for a move to a new position.
        bool setPreparedMove(uint8_t servo_id, uint16_t position, uint16_t move_time);

        /// \brief Get the move time of a prepared servo move.
        uint16_t getPreparedMove(uint8_t servo_id);

        /// \brief Start a prepared servo move.
        bool moveStart(uint8_t servo_id);

        /// \brief Immediately stop the servo from moving.
        ///
        /// The servo must be in servo mode for the move_stop to be effective.
        /// When in motor mode you must set the duty to zero instead.
        bool moveStop(uint8_t servo_id);

        /// \brief Set the servo serial identifier.
        bool setServoId(uint8_t servo_id);

        /// \brief Get the servo serial identifier.
        uint8_t getServoId(uint8_t servo_id);

        /// \brief Set the servo position offset.
        bool setPositionOffset(uint8_t servo_id, int16_t position_offset);

        /// \brief Save the current servo position offset.
        bool savePositionOffset(uint8_t servo_id);

        /// \brief Get the current servo position offset.
        int16_t getPositionOffset(int8_t servo_id);

        /// \brief Set the servo position minimum and maximum limits.
        bool setPositionLimits(uint8_t servo_id, uint16_t min_position, uint16_t max_position);

        /// \brief Get the servo position minimum and maximum limits.
        bool getPositionLimits(uint8_t servo_id, uint16_t &min_position, uint16_t &max_position);

        /// \brief Set the servo voltage minimum and maximum limits.
        bool setVoltageLimits(uint8_t servo_id, double min_voltage, double max_voltage);

        /// \brief Get the servo voltage minimum and maximum limits.
        bool getVoltageLimits(uint8_t servo_id, double &min_voltage, double &max_voltage);

        /// \brief Set the servo maximum temperature limit.
        void setMaxTemperatureLimit(uint8_t servo_id, double max_temperature);

        /// \brief Get the servo maximum temperature limit.
        double getMaxTemperatureLimit(uint8_t servo_id);

        /// \brief Get the servo temperature.
        double getTemperature(uint8_t servo_id);

        /// \brief Get the servo voltage.
        double getVoltage(uint8_t servo_id);

        /// \brief Get the servo position.
        int16_t getPosition(uint8_t servo_id);

        /// \brief Set the servo to 'motor' mode.
        bool setMotorMode(uint8_t servo_id, int16_t duty);

        /// \brief Set the servo to 'servo' mode.
        void setServoMode(uint8_t servo_id);

        /// \brief Get the servo mode.
        bool getMode(uint8_t servo_id, uint8_t &mode, int16_t &duty);

        /// \brief Power the servo motor on.
        bool setMotorOn(uint8_t servo_id);

        /// \brief Power the servo motor off.
        bool setMotorOff(uint8_t servo_id);

        /// \brief Get the servo motor power state.
        bool isMotorOn(uint8_t servo_id);

        /// \brief Turn the servo LED on.
        void setLedOn(uint8_t servo_id);

        /// \brief Turn the servo LED off.
        void setLedOff(uint8_t servo_id);

        /// \brief Get the state of the servo LED.
        bool isLedOn(uint8_t servo_id);

        /// \brief Set the list of faults that cause the LED to flash.
        void setLedErrors(uint8_t servo_id, uint8_t fault_code);

        /// \brief Get the list of faults that cause the LED to flash.
        uint8_t getLedErrors(uint8_t servo_id);

    private:
        serial::Serial serial_;
        ros::Duration response_timeout_ = ros::Duration(0.01);  // [s]

        // Serial communication protocol

        /// \brief Calculate the checksum for a servo serial message.
        static uint8_t checksum(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data);

        /// \brief Read a servo response (to be called immediately after calling sendCommand).
        bool readResponse(uint8_t servo_id, uint8_t length, uint8_t command, std::vector<uint8_t> &data);

        /// \brief Send a command to a servo. Read commands should be immediately followed by readResponse.
        size_t sendCommand(uint8_t servo_id, uint8_t length, uint8_t command, const std::vector<uint8_t> &data = std::vector<uint8_t>());
    };

} // namespace curio_base

#endif // CURIO_BASE_LX16A_DRIVER_H_
