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

#include <cstdarg>
#include <chrono>
#include <string>
#include <iostream>

namespace curio_base
{
    LX16ADriver::LX16ADriver()
    {
    }

    LX16ADriver::~LX16ADriver()
    {
        serial_.close();
    }

    // Serial
    void LX16ADriver::open()
    {
        serial_.open();
    }

    bool LX16ADriver::isOpen() const
    {
        return serial_.isOpen();
    }

    void LX16ADriver::close()
    {
        serial_.close();
    }

    void LX16ADriver::setPort(const std::string &port)
    {
        serial_.setPort(port);
    }

    std::string LX16ADriver::getPort() const
    {
        return serial_.getPort();
    }

    void LX16ADriver::setBaudrate(uint32_t baudrate)
    {
        serial_.setBaudrate(baudrate);
    }

    uint32_t LX16ADriver::getBaudrate() const
    {
        return serial_.getBaudrate();
    }

    void LX16ADriver::setTimeout(uint32_t timeout)
    {
        serial::Timeout t = serial::Timeout::simpleTimeout(timeout);
        serial_.setTimeout(t);
    }

} // namespace curio_base

