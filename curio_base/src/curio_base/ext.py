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

from _curio_base import RoverBaseHAL
from _curio_base import Time
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

    def get_num_wheels(self):
        return self._num_wheels

    def get_num_steers(self):
        return self._num_steers
        
    def get_wheel_position(self, time, i):
        dt = self._elapsed_time(self._wheel_last_times[i], time)
        ds = self._wheel_velocities[i] * dt
        return self._wheel_last_positions[i] + ds

    def get_wheel_velocity(self, time, i):
        return self._wheel_velocities[i]

    def set_wheel_velocity(self, time, i, velocity):
        dt = self._elapsed_time(self._wheel_last_times[i], time)
        ds = self._wheel_velocities[i] * dt
        self._wheel_last_times[i] = time
        self._wheel_last_positions[i] = self._wheel_last_positions[i] + ds
        self._wheel_velocities[i] = velocity

    def get_steer_angle(self, time, i):
        return self._steer_angles[i]

    def set_steer_angle(self, time, i, angle):
        self._steer_angles[i] = angle

    def _elapsed_time(self, start, end):
        '''Calculate the elapsed time in seconds'''

        start_secs = start.sec + start.nsec * 1.0E-9
        end_secs = end.sec + end.nsec * 1.0E-9 
        return end_secs - start_secs