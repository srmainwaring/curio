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

#define USE_ENCODER_SERVICE 0

namespace py = pybind11;

class TestNode
{
public:
    ~TestNode()
    {
        ROS_INFO_STREAM("Stopping motors");
        stopMotor();
    }

    TestNode(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        // Driver parameters
        int baudrate, timeout, servo_id;
        std::string port;
        private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
        private_nh.param<int>("baudrate", baudrate, 115200);
        private_nh.param<int>("timeout", timeout, 1000);
        private_nh.param<int>("control_frequency", control_frequency_, 20);
        private_nh.param<int>("servo_id", servo_id, 11);
        servo_id_ = static_cast<uint8_t>(servo_id);

        ROS_INFO_STREAM("port: " << port);
        ROS_INFO_STREAM("baudrate: " << baudrate);
        ROS_INFO_STREAM("timeout: " << timeout);
        ROS_INFO_STREAM("control_frequency: " << control_frequency_);
        ROS_INFO_STREAM("servo_id: " << servo_id_);

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
        std::unique_ptr<lx16a::LX16ADriver> servo_driver_impl(
            new lx16a::LX16ADriver());
        servo_driver_ = std::move(servo_driver_impl); 

        serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
        servo_driver_->setPort(port);
        servo_driver_->setBaudrate(baudrate);
        servo_driver_->setTimeout(serial_timeout);
        try
        {
            servo_driver_->open();
        }
        catch(const serial::IOException & e)
        {
            ROS_FATAL_STREAM("LX16A driver: failed to open port: " << port);
        }        

        // Status
        ROS_INFO_STREAM("is_open: " << servo_driver_->isOpen());

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

        std::unique_ptr<lx16a::LX16AEncoderFilterPython> encoder_filter_impl(
            new lx16a::LX16AEncoderFilterPython(
                classifier_filename,
                regressor_filename,
                window));
        encoder_filter_ = std::move(encoder_filter_impl);
    #endif

        try
        {
            encoder_filter_->init();
            encoder_filter_->add(servo_id_);
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_FATAL("%s", e.what());
        }

        // Initialise timer.
        ros::Time start_ = ros::Time::now();
    }

    int getControlFrequency()
    {
        return control_frequency_;
    }

    void startMotor()
    {
        servo_driver_->setMotorMode(servo_id_, duty_);
    }

    void stopMotor()
    {
        servo_driver_->setMotorMode(servo_id_, 0);
    }

    void update()
    {
        ros::Time now = ros::Time::now();
        try
        {       
            int16_t position = servo_driver_->getPosition(servo_id_); 
            encoder_filter_->update(servo_id_, now, duty_, position);
            int16_t count = encoder_filter_->getCount(servo_id_);
            int16_t revolutions = encoder_filter_->getRevolutions(servo_id_);

            ROS_INFO_STREAM("pos: " << position
                << ", count: " << count
                << ", rev: " << revolutions);
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_FATAL("%s", e.what());
        }

        double sec = (now - start_).toSec();
        double cps = read_count_/sec;
        read_count_++;
        if (read_count_ > 100)
        {
            ROS_INFO_STREAM("reads/sec: " << cps);
            read_count_ = 0;
            start_ = ros::Time().now();
        }
    }

private:
    ros::Time start_ = ros::Time::now();
    uint32_t read_count_ = 0;
    int control_frequency_ = 20;
    uint8_t servo_id_ = 11;
    int16_t duty_ = 500;
    std::unique_ptr<lx16a::LX16ADriver> servo_driver_;
    std::unique_ptr<lx16a::LX16AEncoderFilter>  encoder_filter_;
};

// Entry point
int main(int argc, char *argv[])
{
    // Initialise the Python interpreter
    py::scoped_interpreter guard{};
    lx16a::addCmdArgsToSys(argc, argv);

    // Initialise node
    ros::init(argc, argv, "lx16a_encoder_filter_test");
    ROS_INFO("Starting node lx16a_encoder_filter_test...");

    ros::NodeHandle nh, private_nh("~");
    TestNode test_node(nh, private_nh);
    test_node.startMotor();

    // Manage Python GIL 
    auto loop = [](TestNode& test_node)
    {
        py::gil_scoped_acquire gil{};
        ros::Rate rate(test_node.getControlFrequency());
        while (ros::ok())
        {
            test_node.update();
            rate.sleep();
        }
    };
    
    {
        // Single thread.        
        // loop(test_node);

        // Worker thread - will deadlock if the GIL is not released
        py::gil_scoped_release gil_release {};
        std::thread loop_thread(loop, std::ref(test_node));
        loop_thread.join();
    }

    ros::waitForShutdown();
    return 0;
}


