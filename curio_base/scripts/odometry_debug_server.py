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

''' Odometry debug server

ROS server to analyse outputs from the wheel odometry, IMU,
Kalman filters, etc.
'''

import rospy
import actionlib
from curio_msgs.msg import CurioImu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

CONTROL_FREQUENCY   = 10  # [Hz]

class OdometryDebugServer(object):

    def __init__(self):
        ''' Constructor
        '''
        # Base controller odometry heading
        self._odom_heading_msg = Float64()
        self._odom_heading_pub = rospy.Publisher('debug/odom_heading', Float64, queue_size=10)
        self._base_odom_sub = rospy.Subscriber('/base_controller/odom', Odometry, self._base_odom_callback)


        # IMU heading
        self._imu_heading_msg = Float64()
        self._imu_heading_pub = rospy.Publisher('debug/imu_heading', Float64, queue_size=10)
        self._curio_imu_sub = rospy.Subscriber('/sensors/curio_imu', CurioImu, self._curio_imu_callback)

    def update(self, event):
        ''' Update - logging.

        Parameters
        ----------
        event : rospy.Timer
            A rospy.Timer event.
        '''
        
        rospy.loginfo('odom: {}, imu: {}'.format(
            self._odom_heading_msg.data, self._imu_heading_msg.data))

    def _curio_imu_callback(self, msg):
        ''' Callback for curio_msgs/CurioImu on sensors/curio_imu
        '''

        # Euler angles from quaternion
        q_x = msg.orientation.x
        q_y = msg.orientation.y
        q_z = msg.orientation.z
        q_w = msg.orientation.w
        # rospy.loginfo('{}, {}, {}, {}'.format(q_x, q_y, q_z, q_w))

        roll = pitch = yaw = 0.0
        (roll, pitch, yaw) = euler_from_quaternion([q_x, q_y, q_z, q_w])
        # rospy.loginfo('imu: {}, {}, {}'.format(roll, pitch, yaw))
        
        self._imu_heading_msg.data = yaw
        self._imu_heading_pub.publish(self._imu_heading_msg)

    def _base_odom_callback(self, msg):
        ''' Callback for nav_msgs/Odometry on base_controller/odom
        '''

        # Euler angles from quaternion
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        # rospy.loginfo('{}, {}, {}, {}'.format(q_x, q_y, q_z, q_w))

        roll = pitch = yaw = 0.0
        (roll, pitch, yaw) = euler_from_quaternion([q_x, q_y, q_z, q_w])
        # rospy.loginfo('odom: {}, {}, {}'.format(roll, pitch, yaw))
        
        self._odom_heading_msg.data = yaw
        self._odom_heading_pub.publish(self._odom_heading_msg)


if __name__ == '__main__':
    rospy.init_node('odometry_debug_server')
    rospy.loginfo('Starting Odometry debug server')
    server = OdometryDebugServer()

    # Start the control loop
    control_frequency = CONTROL_FREQUENCY

    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        server.update)

    rospy.spin()
