#!/usr/bin/env python
# 
#   Software License Agreement (BSD-3-Clause)
#    
#   Copyright (c) 2019 Rhys Mainwaring
#   All rights reserved
#    
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
# 
#   1.  Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#   2.  Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#   3.  Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#  
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
# 

''' Generate /cmd_vel samples using a stepped profile in [-1, 1]
'''

import math
import numpy as np
import random
import rospy
from geometry_msgs.msg import Twist

CONTROL_FREQUENCY = 50      # Control loop frequency [Hz]

SAMPLE_DURATION   = 20.0    # Duration to run at a given level

STARTUP_CMD_VEL   = 0.25    # velocity during startup [m/s]
STARTUP_DURATION  = 2.0     # startup duration [s]

# Publisher
cmd_vel_msg = Twist()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Parameters
control_frequency = CONTROL_FREQUENCY

# Create array of evenly spaced levels in [-1, 1]
level = np.concatenate((np.linspace(-1, -0.15, 18), np.linspace(0.15, 1.0, 18)))
level_idx = 0
level_idx_inc = 1

init_t = None
prev_t = None
curr_t = None

prev_x = 0.0
curr_x = 0.0

def control_loop(event):
    global control_frequency
    global level
    global level_idx
    global level_idx_inc
    global init_t
    global prev_t
    global curr_t
    global prev_x
    global curr_x

    # Startup
    curr_t = rospy.get_rostime()
    if curr_t < init_t + rospy.Duration(STARTUP_DURATION):
        cmd_vel_msg.linear.x = STARTUP_CMD_VEL
        cmd_vel_pub.publish(cmd_vel_msg)
        return

    # Update level
    if curr_t > prev_t + rospy.Duration(SAMPLE_DURATION):
        prev_t = curr_t

        # Constrained path - reverse direction when we hit the boundaries
        level_idx = level_idx + level_idx_inc
        if level_idx == len(level) or level_idx == -1:
            level_idx_inc = -1 * level_idx_inc
            level_idx = level_idx + 2 * level_idx_inc
            curr_x = level[level_idx]

        curr_x = level[level_idx]


    # Update message and publish
    cmd_vel_msg.linear.x = curr_x
    cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':

    rospy.loginfo('Starting LX-16A cmd_vel stepped')
    rospy.init_node('lx_16a_cmd_vel_stepped')

    # Start the control loop
    init_t = rospy.get_rostime()
    prev_t = rospy.get_rostime()
    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        control_loop)

    rospy.spin()


