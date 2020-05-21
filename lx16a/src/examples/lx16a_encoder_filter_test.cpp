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

#include "lx16a/lx16a_driver.h"
#include "lx16a/lx16a_encoder_filter_client.h"
#include "lx16a/lx16a_encoder_filter_python.h"
#include "lx16a/lx16a_exception.h"
#include "lx16a/lx16a_pybind11_embed.h"

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <signal.h>
#include <cstddef>
#include <cstdint>
#include <thread>

#define USE_ENCODER_SERVICE 1

// Servo ids
uint8_t SERVO_ID = 11;

// Servo driver
lx16a::LX16ADriver servo_driver;

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

    // Driver parameters
    int baudrate, timeout, control_frequency, servo_id;
    std::string port;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    private_nh.param<int>("baudrate", baudrate, 115200);
    private_nh.param<int>("timeout", timeout, 1000);
    private_nh.param<int>("control_frequency", control_frequency, 20);
    private_nh.param<int>("servo_id", servo_id, 11);
    SERVO_ID = static_cast<uint8_t>(servo_id);

    ROS_INFO_STREAM("port: " << port);
    ROS_INFO_STREAM("baudrate: " << baudrate);
    ROS_INFO_STREAM("timeout: " << timeout);
    ROS_INFO_STREAM("control_frequency: " << control_frequency);
    ROS_INFO_STREAM("servo_id: " << servo_id);

    // Encoder parameters
    int window;
    std::string classifier_filename, regressor_filename;
    private_nh.param<std::string>("classifier_filename", classifier_filename, "lx16a_tree_classifier.joblib");
    private_nh.param<std::string>("regressor_filename", regressor_filename, "lx16a_tree_regressor.joblib");
    private_nh.param<int>("window", window, 10);

    ROS_INFO_STREAM("classifier_filename: " << classifier_filename);
    ROS_INFO_STREAM("regressor_filename: " << regressor_filename);
    ROS_INFO_STREAM("window: " << window);

    // Driver
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
    servo_driver.setPort(port);
    servo_driver.setBaudrate(baudrate);
    servo_driver.setTimeout(serial_timeout);
    try
    {
        servo_driver.open();
    }
    catch(const serial::IOException & e)
    {
        ROS_FATAL_STREAM("LX16A driver: failed to open port: " << port);
    }        

    // Status
    ROS_INFO_STREAM("is_open: " << servo_driver.isOpen());

    // Wait for Arduino bootloader to complete before sending any
    // data on the serial connection.
    ROS_INFO_STREAM("Waiting for bootloader to complete...");
    ros::Duration(1.0).sleep();

    // Encoder filter
#if USE_ENCODER_SERVICE
    std::unique_ptr<lx16a::LX16AEncoderFilter>  encoder_filter(
        new lx16a::LX16AEncoderFilterClient(
            nh,
            classifier_filename,
            regressor_filename,
            window));
#else
    namespace py = pybind11;

    // Initialise the Python interpreter
    py::scoped_interpreter guard{};
    lx16a::addCmdArgsToSys(argc, argv);

    std::unique_ptr<lx16a::LX16AEncoderFilter>  encoder_filter(
        new lx16a::LX16AEncoderFilterPython(
            classifier_filename,
            regressor_filename,
            window));
#endif

    try
    {
        encoder_filter->init();
        encoder_filter->add(SERVO_ID);
    }
    catch(const lx16a::LX16AException& e)
    {
        ROS_FATAL("%s", e.what());
    }
    
    // Start motor
    int16_t duty = 500;
    servo_driver.setMotorMode(SERVO_ID, duty);

    // Loop
    ros::Rate rate(control_frequency);
    ros::Time start = ros::Time::now();
    uint32_t read_count = 0;
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        try
        {       
            int16_t position = servo_driver.getPosition(SERVO_ID); 
            encoder_filter->update(SERVO_ID, now, duty, position);
            int16_t count = encoder_filter->getCount(SERVO_ID);
            int16_t revolutions = encoder_filter->getRevolutions(SERVO_ID);

            ROS_INFO_STREAM("pos: " << position
                << ", count: " << count
                << ", rev: " << revolutions);
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_FATAL("%s", e.what());
        }

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


