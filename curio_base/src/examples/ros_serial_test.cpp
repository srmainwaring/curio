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

//  Test serial relay using ROS.
//
//  Usage:
//      Read Serial2:
//      $ python -m serial.tools.miniterm /dev/cu.SLAB_USBtoUART 115200
//       
//      Write to Serial 
//      $ rosrun curio_base ros_serial_test
// 
//  Notes:
//
//     - [Arduino blocked when receiving serial](https://forum.arduino.cc/index.php?topic=424406.0)
//

#include <serial/serial.h>

#include <ros/ros.h>

#include <vector>

serial::Serial ser;
long count = 0;

void write()
{
    std::stringstream ss;
    ss << "Count: " << count++ << std::endl;
    size_t bytes_out = ser.write(ss.str());
    ROS_INFO_STREAM("TX[" << bytes_out << "]: " << "Count: " << count);
}

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_VIN_READ             27
const uint8_t SERVO_ID = 1;

uint8_t LobotCheckSum(uint8_t buf[])
{
  uint8_t i;
  uint16_t temp = 0;
  for (uint8_t i = 2; i < buf[3] + 2; ++i)
  {
    temp += buf[i];
  }
  temp = ~temp;
  i = (uint8_t)temp;
  return i;
}

void writeLX16ABuffer()
{
    uint8_t buf[6];
    buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
    buf[2] = SERVO_ID;
    buf[3] = 3;
    buf[4] = LOBOT_SERVO_VIN_READ;
    buf[5] = LobotCheckSum(buf);
    size_t bytes_out = ser.write(buf, 6);

    // std::vector<uint8_t> buf(6);
    // buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
    // buf[2] = SERVO_ID;
    // buf[3] = 3;
    // buf[4] = LOBOT_SERVO_VIN_READ;
    // buf[5] = LobotCheckSum(&buf[0]);
    // size_t bytes_out = ser.write(buf);

    // size_t bytes_out = ser.write("0x55 0x55 0x01 0x03 0x1B 0xE0\n");
    // size_t bytes_out = 6;
    ROS_INFO_STREAM("TX[" << bytes_out << "]:" 
        << " 0x" << std::hex << static_cast<int>(buf[0])
        << " 0x" << std::hex << static_cast<int>(buf[1])
        << " 0x" << std::hex << static_cast<int>(buf[2])
        << " 0x" << std::hex << static_cast<int>(buf[3])
        << " 0x" << std::hex << static_cast<int>(buf[4])
        << " 0x" << std::hex << static_cast<int>(buf[5]));
}

int main(int argc, char *argv[])
{
    // Initialise node
    ros::init(argc, argv, "ros_serial_test");
    ros::NodeHandle nh, private_nh("~");
    ROS_INFO_STREAM("Starting ros_serial_test...");

    // Parameters
    const std::string port("/dev/cu.SLAB_USBtoUART");
    const uint32_t baudrate = 115200; 
    const uint32_t timeout = 1000;       // [ms]
    const uint32_t write_rate = 50;      // [Hz]

    // Serial
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
    ser.setPort(port);
    ser.setBaudrate(baudrate);
    ser.setTimeout(serial_timeout);
    ser.open();
    ROS_INFO_STREAM("port: " << ser.getPort());
    ROS_INFO_STREAM("baudrate: " << ser.getBaudrate());
    ROS_INFO_STREAM("bytesize: " << ser.getBytesize());
    ROS_INFO_STREAM("parity: " << ser.getParity());
    ROS_INFO_STREAM("stopbits: " << ser.getStopbits());
    ROS_INFO_STREAM("flowcontrol: " << ser.getFlowcontrol());
    ROS_INFO_STREAM("inter_byte_timeout: " << ser.getTimeout().inter_byte_timeout);
    ROS_INFO_STREAM("read_timeout_constant: " << ser.getTimeout().read_timeout_constant);
    ROS_INFO_STREAM("read_timeout_multiplier: " << ser.getTimeout().read_timeout_multiplier);
    ROS_INFO_STREAM("write_timeout_constant: " << ser.getTimeout().write_timeout_constant);
    ROS_INFO_STREAM("write_timeout_multiplier: " << ser.getTimeout().write_timeout_multiplier);
    ROS_INFO_STREAM("is_open: " << ser.isOpen());
    ROS_INFO_STREAM("write_rate: " << write_rate << " Hz");

    // Wait for Arduino bootloader to complete after establishing
    // the serial connection before sending further data.
    ROS_INFO_STREAM("Waiting for bootloader to complete...");
    ros::Duration(1.0).sleep();
    
    // Loop
    ros::Rate rate(write_rate);
    while (ros::ok())
    {
        write();
        // writeLX16ABuffer();
        rate.sleep();
    }

    return 0;
}

