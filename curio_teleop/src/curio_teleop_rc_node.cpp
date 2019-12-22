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

#include <curio_msgs/Channels.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <vector>

// Publisher
geometry_msgs::Twist twist_msg;
ros::Publisher twist_pub;

// Service Client
ros::ServiceClient motor_client; 
std::vector<int> channels;

bool have_new = false;
bool is_teleop_disabled = false;

// Defaults
int pwm_min = 1100;
int pwm_max = 1900;
int num_channels = 12;
int linear_x_channel = 1;
int angular_z_channel = 2;
double linear_x_max_velocity = 0.5;
double angular_z_max_velocity = 4.0;
int disable_teleop_channel = 5;

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void channelsCallback(const curio_msgs::Channels::ConstPtr &msg)
{    
    ROS_DEBUG_STREAM("ch1: " << msg->channels[0]
        << ", ch2: " << msg->channels[1]
        << ", ch3: " << msg->channels[2]
        << ", ch4: " << msg->channels[3]
        << ", ch5: " << msg->channels[4]
        << ", ch6: " << msg->channels[5]);

        if (msg->channels.size() < num_channels)
        {
            ROS_ERROR_STREAM("curio_msgs::Channels message must contain at least: "
                << num_channels << " channels");
        }
        for (int i=0; i<num_channels; ++i)
        {
            channels[i] = msg->channels[i];
        }
        have_new = true;
}

void publishMessages()
{
    if (have_new)
    {
        have_new = false;

        // Check the off channel - do not publish if off.
        if (channels[disable_teleop_channel - 1] > 1800)
        {
            if (!is_teleop_disabled)
            {
                ROS_INFO("RC teleop disabled");
                is_teleop_disabled = true;
            }
            return;
        }
        else if (is_teleop_disabled)
        {
            ROS_INFO("RC teleop enabled");
            is_teleop_disabled = false;
        }

        // Calculate the linear velocity.
        double lin_x = map(
            channels[linear_x_channel - 1],
            pwm_min, pwm_max, -linear_x_max_velocity, linear_x_max_velocity);

        // Calculate the angular velocity.
        double ang_z = map(
            channels[angular_z_channel - 1],
            pwm_min, pwm_max,
            -angular_z_max_velocity, angular_z_max_velocity);

        // Publish the twist message.
        twist_msg.linear.x = lin_x;
        twist_msg.angular.z = ang_z;
        twist_pub.publish(twist_msg);
    }
}

int main(int argc, char *argv[])
{
    ROS_INFO("Curio Teleop RC Node");

    // Initialise node.
    ros::init(argc, argv, "curio_teleop_rc_node");
    ros::NodeHandle nh, private_nh("~");

    // Get parameters.
    private_nh.param<int>("num_channels", num_channels, num_channels);
    private_nh.param<int>("linear/x/channel", linear_x_channel, linear_x_channel);
    private_nh.param<int>("angular/z/channel", angular_z_channel, angular_z_channel);
    private_nh.param<double>("linear/x/max_velocity", linear_x_max_velocity, linear_x_max_velocity);
    private_nh.param<double>("angular/z/max_velocity", angular_z_max_velocity, angular_z_max_velocity);
    private_nh.param<int>("disable_teleop/channel", disable_teleop_channel, disable_teleop_channel);    

    // Resize storage for channel PWM data. Set default midpoint to 1500.
    int max_channel = 0;
    max_channel = std::max(max_channel, linear_x_channel);
    max_channel = std::max(max_channel, angular_z_channel);
    max_channel = std::max(max_channel, disable_teleop_channel);
    if (num_channels < max_channel)
    {
        ROS_ERROR_STREAM("Parameter 'num_channels' must be at least: " << max_channel);
    }
    channels.resize(num_channels, 1500);    

    // Subscribers.
    ros::Subscriber channels_sub = nh.subscribe("channels", 10, channelsCallback);

    // Publishers.
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Loop.
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        publishMessages();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;   
}