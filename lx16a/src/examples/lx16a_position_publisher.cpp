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
#include "lx16a/lx16a_exception.h"

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int64.h"

// Servo ids
std::vector<uint8_t> wheel_servo_ids = {
    11, 12, 13, 21, 22, 23
};

std::vector<uint8_t> steer_servo_ids = {
    111, 211
    // 111, 131, 211, 231
};

std::vector<uint16_t> wheel_positions(6);
std::vector<uint16_t> steer_positions(4);

// Servo driver
lx16a::LX16ADriver servo_driver;

// Subscriber
geometry_msgs::Twist cmd_vel_msg;
ros::Subscriber cmd_vel_sub;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_msg = *msg;
}

// Publisher
ros::Publisher position_pub;

// Control loop
bool e_stop = false;
void controlLoop(const ros::TimerEvent& event)
{
    if (e_stop)
    {
        return;
    }

    // Read and publish the position 
    for (int i=0; i<1 /*wheel_servo_ids.size()*/; ++i)
    {
        auto id = wheel_servo_ids[i];
        try
        {
            auto pos = servo_driver.getPosition(id);
            wheel_positions[i] = pos;
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_ERROR("%s", e.what());
        }        
    }
    for (int i=0; i<0 /*steer_servo_ids.size()*/; ++i)
    {
        auto id = steer_servo_ids[i];
        try
        {
            auto pos = servo_driver.getPosition(id);
            steer_positions[i] = pos;
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_ERROR("%s", e.what());
        }        
    }

    // Only publish the position of one wheel...
    std_msgs::Int64 position_msg;
    position_msg.data = wheel_positions[0];
    position_pub.publish(position_msg);

    // Send commands
    // scale duty [-1000, 1000]
    int16_t duty = static_cast<int16_t>(cmd_vel_msg.linear.x * 1000);
    duty = (duty < -1000) ? -1000 : (1000 < duty) ? 1000 : duty;  
    for (int i=0; i<wheel_servo_ids.size(); ++i)
    {
        try
        {
            auto id = wheel_servo_ids[i];
            servo_driver.setMotorMode(id, duty);
            ROS_INFO_STREAM("servo_id: " << static_cast<int>(id) << ", duty: " << duty);    
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_ERROR("%s", e.what());
        }
    }

    // scale steering to [0, 1000] 
    double angle_scaled = 0.5 * (cmd_vel_msg.angular.z / M_PI + 1.0);
    int16_t pos = static_cast<int16_t>(angle_scaled * 1000); 
    pos = (pos < 0) ? 0 : (1000 < pos) ? 1000 : pos;  
    for (int i=0; i<steer_servo_ids.size(); ++i)
    {
        try
        {
            auto id = steer_servo_ids[i];
            servo_driver.setServoMode(id);
            servo_driver.move(id, pos, 100);
            ROS_INFO_STREAM("servo_id: " << static_cast<int>(id) << ", pos: " << pos);    
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_ERROR("%s", e.what());
        }
    }
}

// Custom interrupt signal handler
void sigintHandler(int sig)
{
    // set e-stop to prevent further updates
    e_stop = true;

    // Do some custom action.
    ROS_INFO_STREAM("Stopping node lx16a_driver_test...");
  
    ROS_INFO_STREAM("Stop motors");

    auto stop_motors = []()
    {
        try
        {
            for (auto i=0; i<wheel_servo_ids.size(); ++i)
            {
                auto id = wheel_servo_ids[i];
                servo_driver.setMotorMode(id, 0);
            }
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_WARN_STREAM("" << e.what());
        }
        return false;
    };

    auto check_stopped = [](uint8_t servo_id)
    {
        try
        {
            servo_driver.setMotorMode(servo_id, 0);
            ros::Duration(0.5).sleep();

            ROS_INFO_STREAM("Checking motor "
                << static_cast<int>(servo_id)
                << " has stopped");

            uint8_t mode;
            int16_t duty;
            servo_driver.getMode(servo_id, mode, duty);
            ros::Duration(0.5).sleep();
            return (duty == 0);
        }
        catch(const lx16a::LX16AException &e)
        {
            ROS_WARN_STREAM("" << e.what());
        }
        return false;
    };

    stop_motors();
    for (auto i=0; i<wheel_servo_ids.size(); ++i)
    {
        int retries = 3;
        auto id = wheel_servo_ids[i];
        while (!check_stopped(id) && retries > 0)
        {
            --retries;
        }
    }

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

// Entry point
int main(int argc, char *argv[])
{
    // Initialise node
    ros::init(argc, argv, "lx16a_position_publisher");
    ros::NodeHandle nh, private_nh("~");
    ROS_INFO("Starting node lx16a_position_publisher...");

    // Register custom interrupt signal handler.
    signal(SIGINT, sigintHandler);

    // Driver parameters
    int baudrate, timeout, control_frequency, servo_id;
    std::string port;
    private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
    private_nh.param<int>("baudrate", baudrate, 115200);
    private_nh.param<int>("timeout", timeout, 1000);
    private_nh.param<int>("control_frequency", control_frequency, 20);

    ROS_INFO_STREAM("port: " << port);
    ROS_INFO_STREAM("baudrate: " << baudrate);
    ROS_INFO_STREAM("timeout: " << timeout);
    ROS_INFO_STREAM("control_frequency: " << control_frequency);

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

    // Publisher
    position_pub = nh.advertise<std_msgs::Int64>("servos/position", 100);

    // Subscriber
    cmd_vel_sub = nh.subscribe("cmd_vel", 100, cmdVelCallback);

    // Control loop timer
    ros::Timer control_timer = nh.createTimer(
        ros::Duration(1.0 / control_frequency),
        controlLoop);

    // Process ROS callbacks (controller_manager)
    ros::spin();
    ros::waitForShutdown();

    return 0;
}