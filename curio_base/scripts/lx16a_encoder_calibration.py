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

''' Lewansoul LX-16A encoder calibration.

    This module contains a ROS node for generating training sets for 
    calibrating a LX-16A servo encoder.

    In this node the servo operates in continuous mode, so  
    must be free to rotate continuously and run without load.

    Usage:

    1. Start roscore node

    $ roscore

    2. Start the rotary encoder rosserial node

    $ roslaunch curio_base rotary_encoder.launch

    3. Start the encoder calibration node

    $ rosrun curio_base lx16a_encoder_calibration.py 

    Notes:

    It may be necessary to change the device assigned to SERVO_SERIAL_PORT
    depending upon your operating system and what USB peripherals you have
    connected. The python 'serial' package contains a command line tool
    for listing ports; an example usage running on macOS is:

    $ python -m serial.tools.list_ports -v
    /dev/cu.Bluetooth-Incoming-Port
        desc: n/a
        hwid: n/a
    /dev/cu.wchusbserialfd5110
        desc: USB2.0-Serial
        hwid: USB VID:PID=1A86:7523 LOCATION=253-5.1.1
    2 ports found

    On linux the device will usually be on /dev/ttyUSBx, x = 0,1,2,...

    The SERVO_BAUDRATE should not be changed, 115200 is the value required by the
    LX-16A datasheet.

    Set the SERVO_ID to the value assigned to your servo. The factory default is 1.

'''

import csv
import curio_base.lx16a_driver
import rospy
import serial
from std_msgs.msg import Int64

SERVO_SERIAL_PORT   = '/dev/cu.wchusbserialfd5110'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0
SERVO_ID            = 111

# Convert LX-16A position to angle in deg
def pos_to_deg(pos):
    return pos * 240.0 / 1000.0

class EncoderController(object):
    DATA_BUFFER_SIZE = 100

    def __init__(self):
        # Properties
        self.filename = "./data/lx16a_data.csv"
        self._data = []
        self._data_size = 0

        # Subscribers
        self._encoder_msg = Int64()
        self._encoder_sub = rospy.Subscriber('/encoder', Int64, self.encoder_callback)

        # Initialise servo driver
        self._servo_driver = curio_base.lx16a_driver.LX16ADriver()
        self._servo_driver.set_port(SERVO_SERIAL_PORT)
        self._servo_driver.set_baudrate(SERVO_BAUDRATE)
        self._servo_driver.set_timeout(SERVO_TIMEOUT)
        self._servo_driver.open()
        
        rospy.loginfo('Open connection to servo board')
        rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))
        rospy.loginfo('port: {}'.format(self._servo_driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        rospy.loginfo('timeout: {}'.format(self._servo_driver.get_timeout()))

    def shutdown(self):
        # Stop servo
        rospy.loginfo('Stop servo')
        self._servo_driver.motor_mode_write(SERVO_ID, 0)

        # Write remaining data
        self.write_data()

        # Leave python to clean up the servo driver serial connection.
        # Closing the servo driver manually may cause other running threads to error. 
        # self._servo_driver.close()
        # rospy.loginfo('Close connection to servo board')
        # rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))

    def encoder_callback(self, msg):
        self._encoder_msg = msg

    def control_loop(self, event):
        # Run servo in motor (continuous) mode
        duty = 250
        self._servo_driver.motor_mode_write(SERVO_ID, duty)

        pos = self._servo_driver.pos_read(SERVO_ID)
        count = self._encoder_msg.data
        rospy.loginfo("duty: {}, pos: {}, count: {}".format(duty, pos, count % 4096))

        # Buffer data        
        self._data.append([rospy.get_rostime(), duty, pos, count])
        self._data_size = self._data_size + 1
        if (self._data_size == EncoderController.DATA_BUFFER_SIZE):
            self.write_data()

    def write_data(self):
        with open(self.filename, 'ab') as csvfile:
            # Write data
            writer = csv.writer(csvfile, delimiter=',')
            for row in self._data:
                writer.writerow(row)

            # Reset buffer
            self._data_size = 0
            self._data = []

if __name__ == '__main__':
    rospy.init_node('lx_16a_encoder_calibration')
    rospy.loginfo('Lewansoul LX-16A encoder calibration')

    # Encoder controller
    encoder_controller = EncoderController()
    encoder_controller.file_name = '~/Code/robotics/curio/data/lx16a_duty_250.csv'


    # Register shutdown behaviour
    def shutdown_callback():
        rospy.loginfo('Shutdown lx_16a_encoder_calibration...')
        encoder_controller.shutdown()

    rospy.on_shutdown(shutdown_callback)

    # Start the encoder control loop
    control_frequency = 50.0

    rospy.loginfo('Starting encoder control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        encoder_controller.control_loop)

    rospy.spin()
