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

''' Curio base sensors
'''

import rospy
import serial
from curio_msgs.msg import CurioImu
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

class BaseSensors(object):
    ''' Sensor node for the mobile base

    Subscribe to raw sensor data from the controller units
    and republish as standard ROS sensor messages. 

    Parameters
    ----------
    ~imu/hardware_id : str
        The hardware identifier for the IMU,
        has (default 'BNO080')
    ~imu/orientation_covariance : float
        The orientation covariance for the IMU,
        has (default 0.000001)
    ~imu/angular_velocity_covariance : float
        The angular velocity covariance for the IMU,
        has (default 0.001)
    ~imu/linear_acceleration_covariance : float
        The linear acceleration covariance for the IMU,
        has (default 0.001)
    ~imu/magnetic_field_covariance : float
        The magnetic field covariance for the IMU,
        has (default 0.001)
    '''

    def __init__(self):
        ''' Constructor
        '''

        rospy.loginfo('Initialising BaseSensors...')

        # Parameters
        self._imu_hardware_id = 'BNO080'
        self._orientation_covariance = 0.000001
        self._angular_velocity_covariance = 0.001
        self._linear_acceleration_covariance = 0.001
        self._magnetic_field_covariance = 0.001

        # Parameters
        if rospy.has_param('~imu/hardware_id'):
            self._imu_hardware_id = rospy.get_param(
                '~imu/hardware_id')
        if rospy.has_param('~imu/orientation_covariance'):
            self._orientation_covariance = rospy.get_param(
                '~imu/orientation_covariance')
        if rospy.has_param('~imu/angular_velocity_covariance'):
            self._angular_velocity_covariance = rospy.get_param(
                '~imu/angular_velocity_covariance')
        if rospy.has_param('~imu/linear_acceleration_covariance'):
            self._linear_acceleration_covariance = rospy.get_param(
                '~imu/linear_acceleration_covariance')
        if rospy.has_param('~imu/magnetic_field_covariance'):
            self._magnetic_field_covariance = rospy.get_param(
                '~imu/magnetic_field_covariance')
        
        rospy.loginfo('Using: hardware_id: {}'.format(
            self._imu_hardware_id))
        rospy.loginfo('Using: orientation_covariance: {}'.format(
            self._orientation_covariance))
        rospy.loginfo('Using: angular_velocity_covariance: {}'.format(
            self._angular_velocity_covariance))
        rospy.loginfo('Using: linear_acceleration_covariance: {}'.format(
            self._linear_acceleration_covariance))
        rospy.loginfo('Using: magnetic_field_covariance: {}'.format(
            self._magnetic_field_covariance))

        # Subscriptions
        self._imu_sub = rospy.Subscriber('sensors/curio_imu', CurioImu, self._imu_callback)

        # IMU publications
        self._imu_msg = Imu()
        self._mag_msg = MagneticField()

        # Magnetic field publications
        self._imu_pub = rospy.Publisher('sensors/imu', Imu, queue_size=10)
        self._mag_pub = rospy.Publisher('sensors/magnetic_field', MagneticField, queue_size=10)

    def _imu_callback(self, msg):
        ''' Republish IMU sensor data from the Arduino.

        Subscribe to the curio_msgs/CurioImu sensor messages
        on sensors/curio_imu and republish to:
        
        - sensors/imu as sensor_msgs/Imu
        - sensors/magnetic_field as sensor_msgs/MagneticField

        adding the covariance data loaded from the paremeter server.

        Parameters
        ----------
        msg : curio_msgs/CurioImu
            Sensor data from the base IMU.
        '''
        
        # Headers
        self._imu_msg.header.frame_id = msg.header.frame_id
        self._imu_msg.header.stamp = msg.header.stamp
        self._mag_msg.header.frame_id = msg.header.frame_id
        self._mag_msg.header.stamp = msg.header.stamp

        # Set orientation, angular velocity and linear acceleration
        self._imu_msg.orientation = msg.orientation
        self._imu_msg.angular_velocity = msg.angular_velocity
        self._imu_msg.linear_acceleration = msg.linear_acceleration

        # Set magnetic field
        self._mag_msg.magnetic_field = msg.magnetic_field

        # Set covariances
        for i in range(9):
            self._imu_msg.orientation_covariance[i] = self._orientation_covariance
            self._imu_msg.angular_velocity_covariance[i] = self._angular_velocity_covariance
            self._imu_msg.linear_acceleration_covariance[i] = self._linear_acceleration_covariance
            self._mag_msg.magnetic_field_covariance[i] = self._magnetic_field_covariance

        self._imu_pub.publish(self._imu_msg)
        self._mag_pub.publish(self._mag_msg)
