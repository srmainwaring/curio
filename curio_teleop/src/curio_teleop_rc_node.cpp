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

#include "curio_teleop/curio_teleop_rc_node.h"
#include <curio_msgs/Channels.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace curio_teleop
{
    TeleopRC::TeleopRC(ros::NodeHandle &nh, ros::NodeHandle &private_nh) :
        nh_(nh),
        private_nh_(private_nh)
    {
        // Get parameters.
        private_nh_.param<int>("num_channels", num_channels_, num_channels_);
        private_nh_.param<int>("linear/x/channel", linear_x_channel_, linear_x_channel_);
        private_nh_.param<int>("angular/z/channel", angular_z_channel_, angular_z_channel_);
        private_nh_.param<double>("linear/x/max_velocity", linear_x_max_velocity_, linear_x_max_velocity_);
        private_nh_.param<double>("angular/z/max_velocity", angular_z_max_velocity_, angular_z_max_velocity_);
        private_nh_.param<int>("disable_teleop/channel", disable_teleop_channel_, disable_teleop_channel_);    

        // Resize storage for channel PWM data. Set default midpoint to 1500.
        int max_channel = 0;
        max_channel = std::max(max_channel, linear_x_channel_);
        max_channel = std::max(max_channel, angular_z_channel_);
        max_channel = std::max(max_channel, disable_teleop_channel_);
        if (num_channels_ < max_channel)
        {
            ROS_ERROR_STREAM("Parameter 'num_channels' must be at least: " << max_channel);
        }
        channels_.resize(num_channels_, 1500);    

        // Subscribers.
        ros::Subscriber channels_sub = nh.subscribe("channels", 10, &TeleopRC::channelsCallback, this);

        // Publishers.
        twist_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }


    double TeleopRC::map(double x, double in_min, double in_max, double out_min, double out_max) const
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    void TeleopRC::channelsCallback(const curio_msgs::Channels::ConstPtr &msg)
    {    
        ROS_DEBUG_STREAM("ch1: " << msg->channels[0]
            << ", ch2: " << msg->channels[1]
            << ", ch3: " << msg->channels[2]
            << ", ch4: " << msg->channels[3]
            << ", ch5: " << msg->channels[4]
            << ", ch6: " << msg->channels[5]);

            if (msg->channels.size() < num_channels_)
            {
                ROS_ERROR_STREAM("curio_msgs::Channels message must contain at least: "
                    << num_channels_ << " channels");
            }
            for (int i=0; i<num_channels_; ++i)
            {
                channels_[i] = msg->channels[i];
            }
            have_new_ = true;
    }

    void TeleopRC::publishMessages()
    {
        if (have_new_)
        {
            have_new_ = false;

            // Check the off channel - do not publish if off.
            if (channels_[disable_teleop_channel_ - 1] > 1800)
            {
                if (!is_teleop_disabled_)
                {
                    ROS_INFO("Teleop RC disabled");
                    is_teleop_disabled_ = true;
                }
                return;
            }
            else if (is_teleop_disabled_)
            {
                ROS_INFO("Teleop RC enabled");
                is_teleop_disabled_ = false;
            }

            // Calculate the linear velocity.
            double lin_x = map(
                channels_[linear_x_channel_ - 1],
                pwm_min_, pwm_max_, -linear_x_max_velocity_, linear_x_max_velocity_);

            // Calculate the angular velocity.
            double ang_z = map(
                channels_[angular_z_channel_ - 1],
                pwm_min_, pwm_max_,
                -angular_z_max_velocity_, angular_z_max_velocity_);

            // Publish the twist message.
            twist_msg_.linear.x = lin_x;
            twist_msg_.angular.z = ang_z;
            twist_pub_.publish(twist_msg_);
        }
    }
}

int main(int argc, char *argv[])
{
    ROS_INFO("Starting Curio Teleop RC Node");

    // Initialise node.
    ros::init(argc, argv, "curio_teleop_rc_node");
    ros::NodeHandle nh, private_nh("~");

    curio_teleop::TeleopRC teleop_rc(nh, private_nh);

    // Loop.
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        teleop_rc.publishMessages();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;   
}