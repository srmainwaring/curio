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

''' Generate /cmd_vel samples using a random walk constrained to [-1, 1]
'''

import math
import random
import rospy
from geometry_msgs.msg import Twist

CONTROL_FREQUENCY = 50      # Control loop frequency [Hz]
MU = 0.0                    # Random walk mean
SIGMA = 0.8                 # Random walk stddev

STARTUP_CMD_VEL = 0.25      # velocity during startup [m/s]
STARTUP_DURATION = 10.0     # startup duration [s]

# Publisher
cmd_vel_msg = Twist()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Parameters
mu = MU
sigma = SIGMA
control_frequency = CONTROL_FREQUENCY

init_t = 0.0
prev_t = 0.0
curr_t = 0.0

prev_x = 0.0
curr_x = 0.0

def control_loop(event):
    global mu
    global sigma
    global control_frequency
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

    # Random Normal
    z = random.gauss(mu, sigma)

    # Time increment
    dt = rospy.Duration(1.0 / control_frequency).to_sec()

    # Brownian increment
    dW = math.sqrt(dt) * z

    # Increments
    dx = mu * dt + sigma * dW

    # Constrained path 
    curr_x = prev_x + dx
    if curr_x > 1.0 or curr_x < -1.0:
        curr_x = prev_x - dx 
    prev_x = curr_x

    # Update message and publish
    cmd_vel_msg.linear.x = curr_x
    cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':

    rospy.loginfo('Starting LX-16A cmd_vel random')
    rospy.init_node('lx_16a_cmd_vel_random')

    # Start the control loop
    init_t = rospy.get_rostime()
    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        control_loop)

    rospy.spin()


