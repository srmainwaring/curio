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

''' Test serial relay.

    Notes:

    - [Arduino blocked when receiving serial](https://forum.arduino.cc/index.php?topic=424406.0)
'''

import rospy
import serial

# PORT = '/dev/cu.usbmodem7197691'
# PORT = '/dev/cu.usbmodemFD5121'
PORT = '/dev/cu.SLAB_USBtoUART'
BAUDRATE = 115200
TIMEOUT  = 1

if __name__ == '__main__':
    # Initialise
    rospy.init_node('serial_test')
    rospy.loginfo('Starting serial_test')

    # Initialise serial connection
    serial = serial.Serial()
    serial.port = PORT
    serial.baudrate = BAUDRATE
    serial.timeout = TIMEOUT
    serial.open()

    # Check connection
    rospy.loginfo('Opening Serial connection...')
    rospy.loginfo('port: {}'.format(serial.port))
    rospy.loginfo('baudrate: {}'.format(serial.baudrate))
    rospy.loginfo('is_open: {}'.format(serial.is_open))

    # Arduino reboots when a serial connection is established
    # wait for bootloader to complete scanning the serial port
    # before sending data.
    rospy.sleep(1)

    # TX test
    count = 0
    rate = rospy.Rate(50) # Hz
    while not rospy.is_shutdown():
        msg = 'Count: ' + str(count)
        rospy.loginfo('TX: {}'.format(msg))
        outgoing = bytearray(msg + '\n')
        serial.write(outgoing)
        count = count + 1
        rate.sleep()

