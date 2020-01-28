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

// 
// Servo driver implementation adapted from Lewansoul Arduino examples. 
// 

#ifndef CURIO_BASE_LX16A_DRIVER_H_
#define CURIO_BASE_LX16A_DRIVER_H_

#include <serial/serial.h>

#include <cstddef>
#include <cstdint>

namespace curio_base
{
    class LX16ADriver
    {
    public:
        /// Destructor
        ~LX16ADriver();

        /// Constructor
        LX16ADriver();

        void move(uint8_t id, int16_t position, uint16_t time);
        void stopMove(serial::Serial &SerialX, uint8_t id);
        void angleAdjust(uint8_t id, uint8_t deviation);
        void setMode(uint8_t id, uint8_t mode, int16_t duty);
        int readPosition(uint8_t id);
        int readVin(uint8_t id);

        // Serial interface
        void open();
        bool isOpen() const;
        void close();
        void setPort(const std::string &port);
        std::string getPort() const;
        void setBaudrate(uint32_t baudrate);
        uint32_t getBaudrate() const;
        void setTimeout(uint32_t timeout); // milliseconds

    private:
        serial::Serial serial_;

    };
} // namespace curio_base

#endif // CURIO_BASE_LX16A_DRIVER_H_
