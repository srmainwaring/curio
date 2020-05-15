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

''' Scikit-learn implementation of the _lx16a.LX16AEncoderFilter interface
'''

from _lx16a import LX16AEncoderFilter as LX16AEncoderFilterBase
from _lx16a import Time
from .encoder import LX16AEncoderFilter
import rospy

class LX16AEncoderFilterSklearn(LX16AEncoderFilterBase):
    def __init__(self, classifier_filename, regressor_filename=None, window=10):
        LX16AEncoderFilterBase.__init__(self)
        self._classifier_filename = classifier_filename
        self._regressor_filename = regressor_filename
        self._window = window
        self._encoder_filters = {}

    def init(self):
        pass

    def add(self, servo_id):
        filter =  LX16AEncoderFilter(
            self._classifier_filename,
            self._regressor_filename,
            self._window)
        self._encoder_filters[servo_id] = filter

    # Note: the Python overload expects a lx16a.Time
    def update(self, servo_id, time, duty, position):
        ros_time = rospy.Time(time.sec, time.nsec)
        filter = self._encoder_filters[servo_id]
        filter.update(ros_time, duty, position)

    def get_revolutions(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_revolutions()

    def get_count(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_count()

    def get_duty(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_duty()

    def get_angular_position(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_angular_position()

    # Note: the Python overload returns a tuple (position, is_valid)
    def get_servo_position(self, servo_id, map_position=True):
        filter = self._encoder_filters[servo_id]
        position, is_valid = filter.get_servo_pos(map_position)
        return position, is_valid

    def get_invert(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_invert()

    def set_invert(self, servo_id, is_inverted):
        filter = self._encoder_filters[servo_id]
        filter.set_invert(is_inverted)

    def reset(self, servo_id, position):
        filter = self._encoder_filters[servo_id]
        filter.reset(position)

    def add_v(self, servo_ids):
        for id in servo_ids:
            self.add(servo_id)

    # Note: the Python overload expects a lx16a.Time and returns a list
    def update_v(self, servo_ids, time, duties, positions):
        ros_time = rospy.Time(time.sec, time.nsec)
        i = 0
        angular_positions = [0 for x in servo_ids]
        for id in servo_ids:
            filter = self._encoder_filters[servo_id]
            filter.update_v(ros_time, duties[i], positions[i])
            angular_positions[i] = filter.get_angular_position()
            i = i + 1
        return angular_positions
        
