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

if __name__ == '__main__':
    print('Lewansoul LX-16A driver test')

    # Initialise servo driver
    servo_driver = curio_base.lx16a_driver.LX16ADriver()
    servo_driver.set_port(SERVO_SERIAL_PORT)
    servo_driver.set_baudrate(SERVO_BAUDRATE)
    servo_driver.set_timeout(SERVO_TIMEOUT)
    servo_driver.open()
    
    print('\nOpen connection to servo board')
    print('is_open: {}'.format(servo_driver.is_open()))
    print('port: {}'.format(servo_driver.get_port()))
    print('baudrate: {}'.format(servo_driver.get_baudrate()))
    print('timeout: {}'.format(servo_driver.get_timeout()))

    # Display servo properties
    print('\nServo properties')
    angle_offset = servo_driver.angle_offset_read(SERVO_ID)
    print("angle_offset: {}".format(angle_offset))

    min_angle, max_angle = servo_driver.angle_limit_read(SERVO_ID)
    print("angle_limit: {}, {}".format(min_angle, max_angle))

    min_vin, max_vin = servo_driver.vin_limit_read(SERVO_ID)
    print("vin_limit: {}, {}".format(min_vin, max_vin))

    temp_max_limit = servo_driver.temp_max_limit_read(SERVO_ID)
    print("temp_max_limit: {}".format(temp_max_limit))

    temp = servo_driver.temp_read(SERVO_ID)
    print("temp: {}".format(temp))

    vin = servo_driver.vin_read(SERVO_ID)
    print("vin: {}".format(vin))

    # load_or_unload = servo_driver.load_or_unload_read(SERVO_ID)
    # print("load_or_unload: {}".format(load_or_unload))

    # Run servo in motor (continuous) mode
    print('\nSet motor speed')
    speed = 800
    run_time = 2000
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    print("position: {}, angle: {}".format(pos, angle))
    servo_driver.motor_mode_write(SERVO_ID, speed)

    start = millis()
    while millis() < start + run_time:
        pos = servo_driver.pos_read(SERVO_ID)
        angle = pos_to_deg(pos)
        print("position: {}, angle: {}".format(pos, angle))


    servo_driver.motor_mode_write(SERVO_ID, 0)
    pos = servo_driver.pos_read(SERVO_ID)
    angle = pos_to_deg(pos)
    print("position: {}, angle: {}".format(pos, angle))


    # Shutdown
    servo_driver.close()
    print('\nClose connection to servo board')
    print('is_open: {}'.format(servo_driver.is_open()))
