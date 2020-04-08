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

''' Lewansoul LX-16A failsafe test.

This test node connects to the Lewansoul Servo BusLinker board via USB
and attempts to stop all servos in the WHEEL_SERVO_IDS list at the
rate given by CONTROL_FREQUENCY.

See curio_base/base_failsafe.py for motivation and more detail.

Here is a guide to Arduino bootloaders:
https://www.instructables.com/id/Overview-the-Arduino-sketch-uploading-process-and-/
'''

import curio_base.lx16a_driver
import rospy
import serial

SERVO_SERIAL_PORT   = '/dev/cu.wchusbserialfd5110'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0 # [s]
NUM_WHEELS          = 6
WHEEL_SERVO_IDS     = [11, 12, 13, 21, 22, 23]
CONTROL_FREQUENCY   = 20  # [Hz]

class LX16AFailsafe(object):

    def __init__(self):
        ''' Constructor
        '''

        # Initialise servo driver
        self._servo_driver = curio_base.lx16a_driver.LX16ADriver()
        self._servo_driver.set_port(SERVO_SERIAL_PORT)
        self._servo_driver.set_baudrate(SERVO_BAUDRATE)
        self._servo_driver.set_timeout(SERVO_TIMEOUT)
        self._servo_driver.open()
        
        rospy.loginfo('Open connection to servo bus board')
        rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))
        rospy.loginfo('port: {}'.format(self._servo_driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        rospy.loginfo('timeout: {}'.format(self._servo_driver.get_timeout()))

    def update(self, event):
        ''' Update will stop all wheel servos.

        Parameters
        ----------
        event : rospy.Timer
            A rospy.Timer event.
        '''

        for i in range(NUM_WHEELS):
            servo_id = WHEEL_SERVO_IDS[i]
            self._servo_driver.motor_mode_write(servo_id, 0)

if __name__ == '__main__':
    rospy.init_node('lx16a_failsafe')
    rospy.loginfo('Starting Lewansoul LX-16A failsafe')

    # Servo failsafe
    lx16a_failsafe = LX16AFailsafe()

    # Start the control loop
    control_frequency = CONTROL_FREQUENCY

    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        lx16a_failsafe.update)

    rospy.spin()
