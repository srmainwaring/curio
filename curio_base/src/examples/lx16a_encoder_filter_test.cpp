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
#include "curio_base/lx16a_encoder_filter_client.h"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <signal.h>
#include <cstddef>
#include <cstdint>
#include <thread>

// Servo ids
const uint8_t SERVO_ID = 11;
const std::string CLASSIFIER_FILENAME = "./src/curio/curio_base/data/lx16a_tree_classifier.joblib";
const std::string REGRESSOR_FILENAME = "./src/curio/curio_base/data/lx16a_tree_regressor.joblib";
const int16_t WINDOW = 10;

// Servo driver
curio_base::LX16ADriver servo_driver;

// Custom interrupt signal handler
void sigintHandler(int sig)
{
    // Do some custom action.
    ROS_INFO_STREAM("Stopping node lx16a_encoder_filter_test...");
  
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
    ros::init(argc, argv, "lx16a_encoder_filter_test", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh, private_nh("~");
    ROS_INFO("Starting node lx16a_encoder_filter_test...");

    // Register custom interrupt signal handler.
    signal(SIGINT, sigintHandler);

    // Parameters
    const std::string port("/dev/cu.usbmodemFD5121");
    const uint32_t baudrate = 115200;
    const uint32_t timeout = 1000;       // [ms]
    const uint32_t read_rate = 20;       // [Hz]

    // Servo driver / serial
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
    servo_driver.setPort(port);
    servo_driver.setBaudrate(baudrate);
    servo_driver.setTimeout(serial_timeout);
    servo_driver.open();
    ROS_INFO_STREAM("port: " << servo_driver.getPort());
    ROS_INFO_STREAM("baudrate: " << servo_driver.getBaudrate());
    ROS_INFO_STREAM("is_open: " << servo_driver.isOpen());

    // Encoder filter
    curio_base::LX16AEncoderFilterClient encoder_filter(
        nh,
        SERVO_ID,
        CLASSIFIER_FILENAME,
        REGRESSOR_FILENAME,
        WINDOW);
    encoder_filter.init();

    // Wait for Arduino bootloader to complete before sending any
    // data on the serial connection.
    ROS_INFO_STREAM("Waiting for bootloader to complete...");
    ros::Duration(1.0).sleep();

    // Start motor
    int16_t duty = 500;
    servo_driver.setMotorMode(SERVO_ID, duty);

    // Loop
    ros::Rate rate(read_rate);
    ros::Time start = ros::Time::now();
    uint32_t read_count = 0;
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        int16_t position = servo_driver.getPosition(SERVO_ID);        
        encoder_filter.update(now, duty, position);
        int16_t count = encoder_filter.getCount();
        int16_t revolutions = encoder_filter.getRevolutions();

        ROS_INFO_STREAM("pos: " << position
            << ", count: " << count
            << ", rev: " << revolutions);

        double sec = (now - start).toSec();
        double cps = read_count/sec;
        read_count++;
        if (read_count > 100)
        {
            ROS_INFO_STREAM("reads/sec: " << cps);
            read_count = 0;
            start = ros::Time().now();
        }

        rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}


