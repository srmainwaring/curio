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

''' Lewansoul LX-16A driver odometry.
'''

import math
import time
import curio_base.lx16a_driver

SERVO_SERIAL_PORT   = '/dev/cu.wchusbserialfd5110'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0
SERVO_ID            = 11

# Current millis
def millis():
    return int(round(time.time() * 1000))

# Convert LX-16A position to angle in deg
def pos_to_deg(pos):
    return pos * 240.0 / 1000.0


class LX16AEncoder(object):
    ''' Pseudo encoder for LX-16A servo

    Counts per revolution:  1500
    Resolution:             0.24 deg
    Deadzone:               pos > 1194 or pos < -193 (27 deg)
    '''

    BUFFER_SIZE = 8
    POS_MIN = -190 # -194
    POS_MAX = 1190 # 1194

    def __init__(self):
        self._index = 0
        self._time  = [0.0 for x in range(LX16AEncoder.BUFFER_SIZE)]
        self._duty = [0 for x in range(LX16AEncoder.BUFFER_SIZE)]
        self._pos = [0 for x in range(LX16AEncoder.BUFFER_SIZE)]
        self._rev_count = 0
        self._invalid_zone = None

    def update(self, time, duty, pos):
        self._index = (self._index + 1) %  LX16AEncoder.BUFFER_SIZE
        self._time[self._index] = time
        self._duty[self._index] = duty
        self._pos[self._index] = pos

        # Current gradient
        dpos = self._pos[self._index] - self._pos[self._index-1]

        if dpos > 0:
            if duty >= 0:
                # Gradient and duty positive - valid zone
                pass
            if duty <= 0:
                pass

        # self._count = self._count + dpos
        # print('count: {}, dpos: {}, index: {}, pos: {}'.format(self._count, dpos, self._index, self._pos))
        print('{}, {}, {}'.format(time, duty, pos))


if __name__ == '__main__':
    rospy.init_node('lx_16a_driver_test_node')
    rospy.loginfo('Lewansoul LX-16A odometry test')

    # Initialise servo driver
    servo_driver = curio_base.lx16a_driver.LX16ADriver()
    servo_driver.set_port(SERVO_SERIAL_PORT)
    servo_driver.set_baudrate(SERVO_BAUDRATE)
    servo_driver.set_timeout(SERVO_TIMEOUT)
    servo_driver.open()
    
    rospy.loginfo('\nOpen connection to servo board')
    rospy.loginfo('is_open: {}'.format(servo_driver.is_open()))
    rospy.loginfo('port: {}'.format(servo_driver.get_port()))
    rospy.loginfo('baudrate: {}'.format(servo_driver.get_baudrate()))
    rospy.loginfo('timeout: {}'.format(servo_driver.get_timeout()))

    # Run servo in motor (continuous) mode
    rospy.loginfo('\nSet motor duty')
    duty = 200
    servo_driver.motor_mode_write(SERVO_ID, duty)
    time.sleep(1)
    duty_freq = 0.25
    duty_max = 300
    run_time = 10000
    servo_driver.motor_mode_write(SERVO_ID, duty)
    rospy.loginfo('duty: {}'.format(duty))

    start = millis()
    last_time = start
    last_pos  = servo_driver.pos_read(SERVO_ID)

    encoder = LX16AEncoder()

    while millis() < start + run_time:
        #  Time varying duty
        current_time = millis()
        t = (current_time - start)/1000.0
        last_time = current_time

        duty = int(duty_max * math.sin(2.0 * math.pi * duty_freq * t)) 
        servo_driver.motor_mode_write(SERVO_ID, duty)

        current_pos = servo_driver.pos_read(SERVO_ID)
        encoder.update(current_time - start, duty, current_pos)




    # Stop
    servo_driver.motor_mode_write(SERVO_ID, 0)
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    rospy.loginfo("position: {}, angle: {}".format(pos, angle))

    # Shutdown
    servo_driver.close()
    rospy.loginfo('\nClose connection to servo board')
    rospy.loginfo('is_open: {}'.format(servo_driver.is_open()))
