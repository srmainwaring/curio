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

#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int64.h"

// Servo ids
std::vector<uint8_t> wheel_servo_ids = {
    11, 12, 13, 21, 22, 23
};

std::vector<uint8_t> steer_servo_ids = {
    111, 131, 211, 231
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
void controlLoop(const ros::TimerEvent& event)
{
    // Read and publish the position 
    for (int i=0; i<1; ++i)
    {
        uint8_t id = wheel_servo_ids[i];
        try
        {
            int pos = servo_driver.getPosition(id);
            wheel_positions[i] = pos;
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_ERROR_STREAM("" << e.what());
        }        
    }
    for (int i=0; i<0; ++i)
    {
        uint8_t id = steer_servo_ids[i];
        try
        {
            int pos = servo_driver.getPosition(id);
            steer_positions[i] = pos;
        }
        catch(const lx16a::LX16AException& e)
        {
            ROS_ERROR_STREAM("" << e.what());
        }        
    }

    std_msgs::Int64 position_msg;
    position_msg.data = wheel_positions[0];
    position_pub.publish(position_msg);

    // Send commands
    int16_t duty = static_cast<int16_t>(cmd_vel_msg.linear.x * 1000); 
    for (int i=0; i<6; ++i)
    {
        uint8_t id = wheel_servo_ids[i];
        // servo_driver.setMode(id, 1, duty);    
    }
}

// Entry point
int main(int argc, char *argv[])
{
    // Initialise node
    ros::init(argc, argv, "lx16a_position_publisher");
    ros::NodeHandle nh, private_nh("~");
    ROS_INFO("Starting node lx16a_position_publisher...");

    // Read parameters
    ROS_INFO("Loading parameters...");
    const std::string port("/dev/cu.usbmodem1421201");
    const uint32_t baudrate = 115200;
    const uint32_t timeout = 1000;      // [ms]
    double control_frequency = 1.0;    // [Hz]

    // Initialise driver
    ROS_INFO("Initialising LX-16A servo driver...");
    serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(timeout);
    servo_driver.setPort(port);
    servo_driver.setBaudrate(baudrate);
    servo_driver.setTimeout(serial_timeout);
    servo_driver.open();
    ROS_INFO_STREAM("port: " << servo_driver.getPort());
    ROS_INFO_STREAM("baudrate: " << servo_driver.getBaudrate());
    ROS_INFO_STREAM("is_open: " << servo_driver.isOpen());

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