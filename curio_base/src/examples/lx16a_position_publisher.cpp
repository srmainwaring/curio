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

// Servo driver
curio_base::LX16ADriver servo_driver;

// Control loop
void controlLoop(const ros::TimerEvent& event)
{
    ROS_INFO_STREAM("Control loop: time: " << event.current_real);
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
    std::string port("/dev/cu.wchusbserialfd5110");
    uint32_t baudrate = 115200;
    uint32_t timeout = 100;
    double control_frequency = 20.0;

    // Initialise driver
    ROS_INFO("Initialising LX-16A servo driver...");
    servo_driver.setPort(port);
    servo_driver.setBaudrate(baudrate);
    servo_driver.setTimeout(timeout);
    servo_driver.open();
    ROS_INFO_STREAM("port: " << servo_driver.getPort());
    ROS_INFO_STREAM("baudrate: " << servo_driver.getBaudrate());
    ROS_INFO_STREAM("is_open: " << servo_driver.isOpen());


    // Control loop timer
    ros::Timer control_timer = nh.createTimer(
        ros::Duration(1.0 / control_frequency),
        controlLoop);

    // Process ROS callbacks (controller_manager)
    ros::spin();
    ros::waitForShutdown();

    return 0;
}