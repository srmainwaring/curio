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

''' Lewansoul LX-16A calibration.

    This module contains a set of tests for the LX-16A servo driver.
    The tests operate in both position and continuous mode, so the servo 
    should be free to rotate continuously and run without load.

    The module runs as a ROS node, this is because the LX-16A driver uses
    rospy logging to report warnings and errors so we need to have roscore
    running in order to see these on the terminal.

    Usage:

    1. Start roscore node

    $ roscore

    2. Start driver test

    $ rosrun curio_base lx16a_driver_test.py 

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

import curio_base.lx16a_driver
import io
import rospy
import serial

SERVO_SERIAL_PORT   = '/dev/cu.wchusbserialfd5110'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0
SERVO_ID            = 111

ENCODER_SERIAL_PORT   = '/dev/cu.usbmodemFD5121'
ENCODER_BAUDRATE      = 115200
ENCODER_TIMEOUT       = 1.0

# Convert LX-16A position to angle in deg
def pos_to_deg(pos):
    return pos * 240.0 / 1000.0

class EncoderDriver(object):
    def __init__(self):
        self._serial = serial.Serial()
        self._sio = io.TextIOWrapper(io.BufferedRWPair(self._serial, self._serial))
        self._count = 0
        self._count_a = 0
        self._count_b = 0
        self._count_z = 0
    
    # Serial port 
    def open(self):
        self._serial.open()

    def close(self):
        if self._serial.is_open:
            self._serial.close()

    def is_open(self):
        return self._serial.is_open

    def get_port(self):
        return self._serial.port

    def set_port(self, port):
        self._serial.port = port

    def get_baudrate(self):
        return self._serial.baudrate

    def set_baudrate(self, baudrate):
        self._serial.baudrate = baudrate

    def get_timeout(self):
        return self._serial.timeout

    def set_timeout(self, timeout):
        self._serial.timeout = timeout

    def reset_input_buffer(self):
        self._serial.reset_input_buffer()

    def update(self):
        # self._sio.flush()
        # line = self._sio.readline()
        # self._serial.reset_input_buffer()
        line = self._serial.readline().rstrip()
        self._count = line

        # sep = line.split(',')
        # rospy.loginfo(line)
        # rospy.loginfo(sep)
        # if len(sep) > 0:
        #     self._count = sep[0]
        #     self._count_a = int(sep[1])
        #     self._count_b = int(sep[2])
        #     self._count_z = int(sep[3])

    def count(self):
        return self._count

    def count_a(self):
        return self._count_a

    def count_a(self):
        return self._count_b

    def count_z(self):
        return self._count_z

def run(servo_driver, encoder_driver):
    rospy.loginfo('Running...')

    # Run servo in motor (continuous) mode
    rospy.loginfo('Set motor duty')
    duty = 250
    servo_driver.motor_mode_write(SERVO_ID, duty)
    duty_freq = 0.25
    duty_max = 300
    run_duration = rospy.Duration(10.0)

    start = rospy.get_rostime()
    last_time = start
    last_pos  = servo_driver.pos_read(SERVO_ID)

    # encoder_driver.reset_input_buffer()
    count = 0
    while rospy.get_rostime() < start + run_duration:
        count = count + 1
        current_time = rospy.get_rostime()
        dt = current_time - last_time
        last_time = current_time

        #  Time varying duty
        # duty = int(duty_max * math.sin(2.0 * math.pi * duty_freq * t)) 
        # servo_driver.motor_mode_write(SERVO_ID, duty)

        pos = servo_driver.pos_read(SERVO_ID)
        # encoder_driver.update()
        # count = encoder_driver.count()
        # rospy.loginfo("time: {}, dt: {}, position: {}".format(current_time, dt.to_sec(), pos))

    # Average update time
    rospy.loginfo("av. update time: {}".format(run_duration.to_sec()/count))


    # Stop
    servo_driver.motor_mode_write(SERVO_ID, 0)
    pos = servo_driver.pos_read(SERVO_ID)
 

if __name__ == '__main__':
    rospy.init_node('lx_16a_calibration_node')
    rospy.loginfo('Lewansoul LX-16A calibration')
 
    # Initialise servo driver
    servo_driver = curio_base.lx16a_driver.LX16ADriver()
    servo_driver.set_port(SERVO_SERIAL_PORT)
    servo_driver.set_baudrate(SERVO_BAUDRATE)
    servo_driver.set_timeout(SERVO_TIMEOUT)
    servo_driver.open()
    
    rospy.loginfo('Open connection to servo board')
    rospy.loginfo('is_open: {}'.format(servo_driver.is_open()))
    rospy.loginfo('port: {}'.format(servo_driver.get_port()))
    rospy.loginfo('baudrate: {}'.format(servo_driver.get_baudrate()))
    rospy.loginfo('timeout: {}'.format(servo_driver.get_timeout()))

    # Initialise encoder driver
    encoder_driver = EncoderDriver()
    # encoder_driver.set_port(ENCODER_SERIAL_PORT)
    # encoder_driver.set_baudrate(ENCODER_BAUDRATE)
    # encoder_driver.set_timeout(ENCODER_TIMEOUT)
    # encoder_driver.open()
    
    # rospy.loginfo('Open connection to encoder')
    # rospy.loginfo('is_open: {}'.format(encoder_driver.is_open()))
    # rospy.loginfo('port: {}'.format(encoder_driver.get_port()))
    # rospy.loginfo('baudrate: {}'.format(encoder_driver.get_baudrate()))
    # rospy.loginfo('timeout: {}'.format(encoder_driver.get_timeout()))

    # Run.
    run(servo_driver, encoder_driver)

    # Shutdown
    servo_driver.close()
    rospy.loginfo('Close connection to servo board')
    rospy.loginfo('is_open: {}'.format(servo_driver.is_open()))

    # encoder_driver.close()
    # rospy.loginfo('Close connection to encoder')
    # rospy.loginfo('is_open: {}'.format(encoder_driver.is_open()))
