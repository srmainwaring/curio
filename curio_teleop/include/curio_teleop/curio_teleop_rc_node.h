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

#ifndef CURIO_TELEOP_CURIO_TELEOP_RC_NODE_H_
#define CURIO_TELEOP_CURIO_TELEOP_RC_NODE_H_

#include <curio_msgs/Channels.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace curio_teleop
{
    /// \brief Radio Control Teleop node manager
    ///
    /// The node subscribes to RC data arriving as a curio_msgs::Channels message
    /// on topic 'channels' and publishes geometry_msgs::Twist commands to 'cmd_vel'
    /// when teleoperation is enabled.
    ///
    /// Parameters may be used to assign RC transmitter switches to
    /// enable / disable RC teleop control.
    ///
    class TeleopRC
    {
    public:
        /// \brief Constructor
        TeleopRC(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

        /// \brief Publish messages
        //
        /// Publish the Twist message to topic 'cmd_vel' if teleoperation is enabled. 
        void publishMessages();

    private:
        // Node handles
        ros::NodeHandle nh_, private_nh_;

        // Publisher Twist message to topic 'cmd_vel' at 10 Hz
        geometry_msgs::Twist twist_msg_;
        ros::Publisher twist_pub_;

        // RC channel data
        std::vector<int> channels_;

        bool have_new_ = false;
        bool is_teleop_disabled_ = false;

        // Parameters
        int pwm_min_ = 1100;
        int pwm_max_ = 1900;
        int num_channels_ = 12;
        int linear_x_channel_ = 1;
        int angular_z_channel_ = 2;
        double linear_x_max_velocity_ = 0.5;
        double angular_z_max_velocity_ = 4.0;
        int disable_teleop_channel_ = 5;

        /// \brief Utility to map x in range [in_min, in_max] to range [out_min, out_max].
        double map(double x, double in_min, double in_max, double out_min, double out_max) const;

        /// \brief Subscriber callback for RC channels data received on topic 'channels'. 
        void channelsCallback(const curio_msgs::Channels::ConstPtr &msg);

    };
} // namespace curio_teleop

#endif // CURIO_TELEOP_CURIO_TELEOP_RC_NODE_H_