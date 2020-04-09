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
#include <ros/callback_queue.h>

#include <signal.h>
#include <cstddef>
#include <cstdint>
#include <thread>

// Servo ids
const uint8_t SERVO_ID = 11;

// Servo driver
curio_base::LX16ADriver servo_driver;

void testGetPositionOffset()
{
    int16_t position_offset = 0;
    servo_driver.getPositionOffset(SERVO_ID);
    ROS_INFO_STREAM("position_offset: " << position_offset);
}

void testGetPositionLimits()
{
    uint16_t min_position = 0, max_position = 0;
    servo_driver.getPositionLimits(SERVO_ID, min_position, max_position);
    ROS_INFO_STREAM("min_position: " << min_position << ", max_position: " << max_position);
}

void testGetVoltageLimits()
{
    double min_voltage = 0, max_voltage = 0;
    servo_driver.getVoltageLimits(SERVO_ID, min_voltage, max_voltage);
    ROS_INFO_STREAM("min_voltage: " << min_voltage << ", max_voltage: " << max_voltage);
}

void testGetMaxTemperatureLimit()
{
    double max_temperature = servo_driver.getMaxTemperatureLimit(SERVO_ID);
    ROS_INFO_STREAM("max_temperature: " << max_temperature);
}

void testGetVoltage()
{
    double voltage = servo_driver.getVoltage(SERVO_ID);
    ROS_INFO_STREAM("voltage: " << voltage);
}

void testGetPosition()
{
    int16_t position = servo_driver.getPosition(SERVO_ID);
    ROS_INFO_STREAM("position: " << position);
}

void testGetTemperature()
{
    double temperature = servo_driver.getTemperature(SERVO_ID);
    ROS_INFO_STREAM("temperature: " << temperature);
}

void testGetMode()
{
    uint8_t mode;
    int16_t duty;
    servo_driver.getMode(SERVO_ID, mode, duty);
    ROS_INFO_STREAM("mode: " << static_cast<int>(mode) <<  ", duty: " << duty);
}

void testIsMotorOn()
{
    bool is_motor_on = servo_driver.isMotorOn(SERVO_ID);
    ROS_INFO_STREAM("is_motor_on: " << is_motor_on);
}

void testIsLedOn()
{
    bool is_led_on = servo_driver.isLedOn(SERVO_ID);
    ROS_INFO_STREAM("is_led_on: " << is_led_on);
}

// Custom interrupt signal handler
void sigintHandler(int sig)
{
    // Do some custom action.
    ROS_INFO_STREAM("Stopping node lx16a_driver_test...");
  
    ROS_INFO_STREAM("Stop motor");
    servo_driver.setMotorMode(SERVO_ID, 0);

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
    // const std::string port("/dev/cu.usbmodem7197691");
    const std::string port("/dev/cu.usbmodemFD5121");
    // const std::string port("/dev/cu.usbmodem136");
    // const std::string port("/dev/cu.SLAB_USBtoUART");
    const uint32_t baudrate = 115200;
    const uint32_t timeout = 1000;       // [ms]
    const uint32_t read_rate = 100;      // [Hz]

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
    servo_driver.setMotorMode(SERVO_ID, 500);

    // Loop
    ros::Rate rate(read_rate);
    ros::Time start = ros::Time::now();
    uint32_t count = 0;
    while (ros::ok())
    {
        testGetPositionOffset();
        testGetPositionLimits();
        testGetVoltageLimits();
        testGetMaxTemperatureLimit();
        testGetVoltage();
        testGetPosition();
        testGetTemperature();
        testGetMode();
        testIsMotorOn();
        testIsLedOn();

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

    ros::waitForShutdown();
    return 0;
}


