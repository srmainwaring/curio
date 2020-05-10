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

''' Generate /cmd_vel samples using a sinusoid profile in [-1, 1]
'''

import math
import numpy as np
import random
import rospy
from geometry_msgs.msg import Twist

CONTROL_FREQUENCY = 50      # Control loop frequency [Hz]

SAMPLE_DURATION   = 10.0    # Duration to run at a given level

STARTUP_CMD_VEL   = 0.25    # velocity during startup [m/s]
STARTUP_DURATION  = 2.0     # startup duration [s]

# Publisher
cmd_vel_msg = Twist()
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Parameters
control_frequency = CONTROL_FREQUENCY

# Create array of evenly spaced amplitudes in [0.15, 1]
amp = np.linspace(0.15, 1.0, 18)
amp_idx = 0

# Create array of periods in [0.5, 20]
period = np.concatenate((np.linspace(0.2, 1, 5), np.linspace(2, 10, 9)))
period_idx = 0

init_t = None
prev_t = None
curr_t = None

curr_amp = amp[amp_idx]
curr_period = period[period_idx]

def update(event):
    global control_frequency
    global amp
    global amp_idx
    global amp_idx_inc
    global period
    global period_idx
    global init_t
    global prev_t
    global curr_t
    global curr_amp
    global curr_period

    # Startup
    curr_t = rospy.get_rostime()
    if curr_t - init_t < rospy.Duration(STARTUP_DURATION):
        prev_t = curr_t
        cmd_vel_msg.linear.x = STARTUP_CMD_VEL
        cmd_vel_pub.publish(cmd_vel_msg)
        return

    # Update amplitude
    if curr_t - prev_t > rospy.Duration(SAMPLE_DURATION):
        prev_t = curr_t

        # Update period whenever the amplitudes cycle round
        if amp_idx + 1 == len(amp):
            period_idx = (period_idx + 1) % len(period)
            curr_period = period[period_idx]

        # Cycle through the amplitudes
        amp_idx = (amp_idx + 1) % len(amp)
        curr_amp = amp[amp_idx]

        rospy.loginfo('amp_idx: {}, amp: {}, period_idx: {}, period: {}'
            .format(amp_idx, curr_amp, period_idx, curr_period))

    # Update message and publish
    t = (curr_t - init_t).to_sec()
    omega = 2 * math.pi / curr_period
    vel = curr_amp * math.sin(omega * t)

    cmd_vel_msg.linear.x = vel
    cmd_vel_pub.publish(cmd_vel_msg)

if __name__ == '__main__':
    rospy.init_node('lx_16a_cmd_vel_sinusoidal')
    rospy.loginfo('Starting LX-16A cmd_vel sinusoidal')

    # Start the control loop
    init_t = rospy.get_rostime()
    prev_t = rospy.get_rostime()
    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        update)

    rospy.spin()


