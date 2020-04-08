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

//  Test serial relay.
//
//  Usage:
//      Read Serial2:
//      $ python -m serial.tools.miniterm /dev/cu.SLAB_USBtoUART 115200
//       
//      Write to Serial 
//      $ rosrun curio_base serial_test
// 
//  Notes:
//
//     - [Arduino blocked when receiving serial](https://forum.arduino.cc/index.php?topic=424406.0)
//

#include <serial/serial.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <sstream>

serial::Serial ser;
long count = 0;

void write()
{
    std::stringstream ss;
    ss << "Count: " << count++ << std::endl;
    size_t bytes_out = ser.write(ss.str());
    std::cout << "TX[" << bytes_out << "]: " << "Count: " << count << std::endl;
}

int main(int argc, char *argv[])
{
    std::cout << "Starting serial_test..." << std::endl;

    // Parameters
    // const std::string port("/dev/cu.usbmodemFD5121");
    const std::string port("/dev/cu.SLAB_USBtoUART");
    const uint32_t baudrate = 115200; 
    const uint32_t timeout = 1000;       // [ms]
    const uint32_t write_rate = 500;     // [Hz]

    // Serial
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    ser.setTimeout(serial_timeout);
    ser.open();
    std::cout << "port: " << ser.getPort() << std::endl;
    std::cout << "baudrate: " << ser.getBaudrate() << std::endl;
    std::cout << "bytesize: " << ser.getBytesize() << std::endl;
    std::cout << "parity: " << ser.getParity() << std::endl;
    std::cout << "stopbits: " << ser.getStopbits() << std::endl;
    std::cout << "flowcontrol: " << ser.getFlowcontrol() << std::endl;
    std::cout << "inter_byte_timeout: " << ser.getTimeout().inter_byte_timeout << std::endl;
    std::cout << "read_timeout_constant: " << ser.getTimeout().read_timeout_constant << std::endl;
    std::cout << "read_timeout_multiplier: " << ser.getTimeout().read_timeout_multiplier << std::endl;
    std::cout << "write_timeout_constant: " << ser.getTimeout().write_timeout_constant << std::endl;
    std::cout << "write_timeout_multiplier: " << ser.getTimeout().write_timeout_multiplier << std::endl;
    std::cout << "is_open: " << ser.isOpen() << std::endl;
    std::cout << "write_rate: " << write_rate << " Hz" << std::endl;

    // Wait for Arduino bootloader to complete after establishing
    // the serial connection before sending further data.
    std::cout << "Waiting for bootloader to complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // Loop
    std::chrono::milliseconds duration(1000/write_rate);
    auto last_time = std::chrono::steady_clock::now();
    while (true)
    {
        if (std::chrono::steady_clock::now() - last_time > duration)
        {            
            last_time += duration;            
            write();
        }
    }

    return 0;
}

