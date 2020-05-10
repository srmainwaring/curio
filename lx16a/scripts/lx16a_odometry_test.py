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

''' Lewansoul LX-16A odometry test.
'''

import csv
from lx16a.lx16a_driver import LX16ADriver
from lx16a.lx16a_encoder_filter import LX16AEncoderFilter
import rospy
import serial
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

SERVO_SERIAL_PORT   = '/dev/cu.usbmodem1421201'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0 # [s]
SERVO_ID            = 11

CONTROL_FREQUENCY   = 50  # [Hz]

# Constants
DATA_DIR  = './data/'
WINDOW    = 10

# Filename for persisted ML model
MODEL_FILENAME = "{0}lx16a_tree_model_all.joblib".format(DATA_DIR)

# Convert LX-16A position to angle in deg
# def pos_to_deg(pos):
#     return pos * 240.0 / 1000.0

class LX16AOdometer(object):

    def __init__(self):
        # Publisher
        self._encoder_msg = Int64()
        self._encoder_pub = rospy.Publisher('/encoder', Int64, queue_size=10)

        # Subscriber
        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Initialise encoder filter
        self._encoder_filter = LX16AEncoderFilter(MODEL_FILENAME, WINDOW)

        # Initialise servo driver
        self._servo_driver = LX16ADriver()
        self._servo_driver.set_port(SERVO_SERIAL_PORT)
        self._servo_driver.set_baudrate(SERVO_BAUDRATE)
        self._servo_driver.set_timeout(SERVO_TIMEOUT)
        self._servo_driver.open()
        
        rospy.loginfo('Open connection to servo bus board')
        rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))
        rospy.loginfo('port: {}'.format(self._servo_driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        rospy.loginfo('timeout: {}'.format(self._servo_driver.get_timeout()))

    def shutdown(self):
        # Stop servo
        rospy.loginfo('Stop servo')
        self._servo_driver.motor_mode_write(SERVO_ID, 0)

    # def encoder_callback(self, msg):
    #     self._encoder_msg = msg

    def cmd_vel_callback(self, msg):
        self._cmd_vel_msg = msg

    def update(self, event):
        # To simplify things we map the linear velocity from [-1.0, 1.0] m/s
        # to [-1000, 1000] duty, and limit it to the range.
        duty = int(1000 * self._cmd_vel_msg.linear.x)
        duty = max(duty, -1000)
        duty = min(duty, +1000)

        # Read odometry
        rostime = rospy.get_rostime()    

        # @TODO: PROFILING
        pos = 0
        for i in range(10):
            pos = self._servo_driver.pos_read(SERVO_ID)
        # @TODO: PROFILING

        # pos = self._servo_driver.pos_read(SERVO_ID)
        self._encoder_filter.update(rostime, duty, pos)
        count = self._encoder_filter.get_count()

        # Write commands
        self._servo_driver.motor_mode_write(SERVO_ID, duty)

        # Publish
        self._encoder_msg.data = count
        self._encoder_pub.publish(self._encoder_msg)

        # rospy.loginfo("duty: {}, pos: {}, count: {}".format(duty, pos, count))

if __name__ == '__main__':
    rospy.init_node('lx16a_odometry_test')
    rospy.loginfo('Starting Lewansoul LX-16A odometry test')

    # Servo odometer
    odometer = LX16AOdometer()

    # Register shutdown behaviour
    def shutdown_callback():
        rospy.loginfo('Shutdown lx16a_odometry_test...')
        odometer.shutdown()

    rospy.on_shutdown(shutdown_callback)

    # Start the control loop
    control_frequency = CONTROL_FREQUENCY

    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        odometer.update)

    rospy.spin()
