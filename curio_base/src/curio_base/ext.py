# 
#   Software License Agreement (BSD-3-Clause)
#    
#   Copyright (c) 2020 Rhys Mainwaring
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

from lx16a.ext import Time
from _curio_base import RoverBaseHAL
import math
import rospy

class MockRoverBaseHAL(RoverBaseHAL):
    '''A mock implementation of a rover base HAL
    '''
    def __init__(self):
        RoverBaseHAL.__init__(self)
        self._num_wheels = 6
        self._num_steers = 4
        self._wheel_last_times = [Time() for x in range(6)]
        self._wheel_last_positions = [0.0 for x in range(6)]
        self._wheel_velocities = [0.0 for x in range(6)]
        self._steer_angles = [0.0 for x in range(4)]

        rospy.loginfo('Initialising Mock Rover HAL...')
        rospy.loginfo('Num Wheels: {}'.format(self._num_wheels))
        rospy.loginfo('Num Steers: {}'.format(self._num_steers))

    def get_num_wheels(self):
        return self._num_wheels

    def get_num_steers(self):
        return self._num_steers
        
    def get_wheel_position(self, time, i):
        dt = self._elapsed_time(self._wheel_last_times[i], time)
        ds = self._wheel_velocities[i] * dt
        s = self._wheel_last_positions[i] + ds
        rospy.logdebug('i: {}, dt: {:.2f}, ds: {:.2f}, s: {:.2f}'.format(i, dt, ds, s))
        return s

    def get_wheel_velocity(self, time, i):
        v = self._wheel_velocities[i]
        rospy.logdebug('i: {}, v: {:.2f}'.format(i, v))
        return v

    def set_wheel_velocity(self, time, i, velocity):
        dt = self._elapsed_time(self._wheel_last_times[i], time)
        ds = self._wheel_velocities[i] * dt
        self._wheel_last_times[i] = time
        self._wheel_last_positions[i] = self._wheel_last_positions[i] + ds
        self._wheel_velocities[i] = velocity

    def get_steer_angle(self, time, i):
        s = self._steer_angles[i]
        rospy.logdebug('i: {}, s: {:.2f}'.format(i, s))
        return s

    def set_steer_angle(self, time, i, angle):
        self._steer_angles[i] = angle

    def _elapsed_time(self, start, end):
        start_secs = start.sec + start.nsec * 1.0E-9
        end_secs = end.sec + end.nsec * 1.0E-9 
        return end_secs - start_secs

    def _to_sec(self, t):
        return t.sec + t.nsec * 1.0E-9

from lx16a.ext import LX16ADriver
import time

class LX16AMinimalRoverBaseHAL(RoverBaseHAL):
    '''Minimal rover base HAL implementation

    This version uses the LX16ADriver extension module to set the
    wheel and steer positions. The odometry is a simple mock up using
     the velocity and position set points to approximate actual positions.

    Tested to run at control_frequency: 15 Hz with a direct USB connection
    to the Lewansoul BusLinker board.

    Notes:
        - mock odometry (no encoder filter integration)
        - no adjustment of steering trim
    '''
    def __init__(self):
        RoverBaseHAL.__init__(self)

        # Configure and open the servo driver
        self._driver = LX16ADriver()
        self._driver.set_port('/dev/cu.usbserial-142110')
        self._driver.set_baudrate(115200)
        self._driver.set_timeout(1000)
        self._driver.set_response_timeout(50)
        self._driver.open()

        rospy.loginfo('Starting LX16ADriver...')
        rospy.loginfo('port: {}'.format(self._driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._driver.get_baudrate()))
        rospy.loginfo('is_open: {}'.format(self._driver.is_open()))

        # The wheel and steering servo mappings must match 
        # those used in the controller config file.
        self._num_wheels = 6
        self._num_steers = 4
        self._wheel_servo_ids = [11, 12, 13, 21, 22, 23]
        self._wheel_servo_orientation = [1, 1, 1, -1, -1, -1]
        self._steer_servo_ids = [111, 131, 211, 231]
        self._steer_servo_orientation = [-1, -1, -1, -1]
 
        self._wheel_last_times = [Time() for x in range(6)]
        self._wheel_last_positions = [0.0 for x in range(6)]
        self._wheel_velocities = [0.0 for x in range(6)]
        self._steer_angles = [0.0 for x in range(4)]    

        try:
            rospy.loginfo('Initialising steer servos...')
            for servo_id in self._steer_servo_ids:
                self._driver.set_servo_mode(servo_id)
        except RuntimeError as e:
            rospy.logerr('Failed to initialise steer servos: {}'.format(e))

    def get_num_wheels(self):
        return self._num_wheels

    def get_num_steers(self):
        return self._num_steers
        
    def get_wheel_position(self, time, i):
        # Optional - add a wheel encoder + filter here.
        if False:
            # Only lookup values for i = 1 or i = 4 (middle wheels)
            # (no filter so values will skip)
            servo_id = self._wheel_servo_ids[i]
            orientation = self._wheel_servo_orientation[i]
            pos = self._driver.get_position(servo_id)
            angle = self._map_pos2angle(pos) * orientation
            # adjust for revolutions / filter invalid

        # Use mock values derived from wheel velocity set-points
        dt = self._elapsed_time(self._wheel_last_times[i], time)
        ds = self._wheel_velocities[i] * dt
        return self._wheel_last_positions[i] + ds

    def get_wheel_velocity(self, time, i):
        return self._wheel_velocities[i]

    def set_wheel_velocity(self, time, i, velocity):
        # Capture velocity and update mock wheel positions
        dt = self._elapsed_time(self._wheel_last_times[i], time)
        ds = self._wheel_velocities[i] * dt
        self._wheel_last_times[i] = time
        self._wheel_last_positions[i] = self._wheel_last_positions[i] + ds
        self._wheel_velocities[i] = velocity

        # Set wheel duty.
        servo_id = self._wheel_servo_ids[i]
        orientation = self._wheel_servo_orientation[i]
        duty = self._map_vel2duty(velocity * orientation) 

        try:
            rospy.logdebug('set_wheel_velocity: servo_id: {}, duty: {}, velocity: {}'.format(servo_id, duty, velocity))
            self._driver.set_motor_mode(servo_id, duty)
        except RuntimeError as e:
            rospy.logerr('Failed to update wheel: servo_id: {}, {}'.format(servo_id, e))

    def get_steer_angle(self, time, i):
        # Optional - look up actual steering positions here
        if False:
            servo_id = self._wheel_servo_ids[i]
            orientation = self._wheel_servo_orientation[i]
            pos = self._driver.get_position(servo_id)
            angle = self._map_pos2angle(pos) * orientation
            return angle

        # Use mock values derived from the steering set-point.
        return self._steer_angles[i]

    def set_steer_angle(self, time, i, angle):
        # Capture steering set-point
        self._steer_angles[i] = angle
        
        # Move the steering servo
        servo_id = self._steer_servo_ids[i]
        orientation = self._steer_servo_orientation[i]
        pos = self._map_angle2pos(angle * orientation) 

        try:
            rospy.logdebug('set_steer_angle: servo_id: {}, pos: {}, angle: {}'.format(servo_id, pos, angle))
            self._driver.move(servo_id, pos, 50)
        except RuntimeError as e:
            rospy.logerr('Failed to update steer: servo_id: {}, {}'.format(servo_id, e))

    def _elapsed_time(self, start, end):
        start_secs = start.sec + start.nsec * 1.0E-9
        end_secs = end.sec + end.nsec * 1.0E-9 
        return end_secs - start_secs
    
    def _map_angle2pos(self, angle):
        mid = 500
        pos = int(1500 * angle / (2 * math.pi))
        return mid + pos
    
    def _map_pos2angle(self, pos):
        pos = pos % 1500
        angle = 2 * math.pi * pos / 1500
        return angle

    def _map_vel2duty(self, vel):
        max_vel = 2.0 * math.pi
        duty = int(1000 * min(1, max(-1, vel / max_vel)))
        return duty
