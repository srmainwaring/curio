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

''' Lewansoul LX-16A driver test.
'''

import time
import lewansoul_driver
# import lewansoul_wrapper

SERVO_SERIAL_PORT   = '/dev/cu.wchusbserialfd5140'
SERVO_BAUDRATE      = 115200
SERVO_TIMEOUT       = 1.0
SERVO_ID            = 111


if __name__ == '__main__':
    print('Lewansoul LX-16A driver test')

    servo_driver = lewansoul_driver.LewansoulDriver()
    servo_driver.set_port(SERVO_SERIAL_PORT)
    servo_driver.set_baudrate(SERVO_BAUDRATE)
    servo_driver.set_timeout(SERVO_TIMEOUT)
    servo_driver.open()
    
    print('\nOpen connection to servo board')
    print('is_open: {}'.format(servo_driver.is_open()))
    print('port: {}'.format(servo_driver.get_port()))
    print('baudrate: {}'.format(servo_driver.get_baudrate()))
    print('timeout: {}'.format(servo_driver.get_timeout()))

    print('\nServo properties')
    angle_offset = servo_driver.angle_offset_read(SERVO_ID)
    print("angle_offset: {}".format(angle_offset))

    min_angle, max_angle = servo_driver.angle_limit_read(SERVO_ID)
    print("angle_limit: {}, {}".format(min_angle, max_angle))

    min_vin, max_vin = servo_driver.vin_limit_read(SERVO_ID)
    print("vin_limit: {}, {}".format(min_vin, max_vin))

    print('\nSet motor speed')
    print("position: {}".format(servo_driver.pos_read(SERVO_ID)))
    servo_driver.motor_mode_write(SERVO_ID, 500)
    time.sleep(1)
    print("position: {}".format(servo_driver.pos_read(SERVO_ID)))

    servo_driver.motor_mode_write(SERVO_ID, 0)
    time.sleep(1)
    print("position: {}".format(servo_driver.pos_read(SERVO_ID)))

    servo_driver.motor_mode_write(SERVO_ID, -500)
    time.sleep(1)
    print("position: {}".format(servo_driver.pos_read(SERVO_ID)))

    servo_driver.motor_mode_write(SERVO_ID, 0)
    time.sleep(1)
    print("position: {}".format(servo_driver.pos_read(SERVO_ID)))

    servo_driver.close()
    print('\nClose connection to servo board')
    print('is_open: {}'.format(servo_driver.is_open()))
