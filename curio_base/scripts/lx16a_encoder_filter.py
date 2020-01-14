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

''' Lewansoul LX-16A encoder filter.
'''

import math
import numpy as np
import pandas as pd
import rospy

WINDOW_SIZE = 10
LOWER  = 1190
UPPER  = 1310

class LX16AEncoderFilter(object):
    ''' Encoder filter for the LX16A.
    
        The feature row vector contains:
        WINDOW_SIZE time deltas (increasingly negative) back to the previous encoder positions
        WINDOW_SIZE positions - the previous values of the LX-16A servo
        WINDOW_SIZE duty - the commanded duty at each previous time
    '''
    def __init__(self, window_size):        
        '''Constructor
        
            Initialise rotating buffers that store the encoder history.
        '''
        self._window_size = window_size
        self._index     = 0
        self._ros_time  = [0 for x in range(WINDOW_SIZE)]
        self._duty      = [0 for x in range(WINDOW_SIZE)]
        self._pos       = [0 for x in range(WINDOW_SIZE)]
        self._X         = [0 for x in range(3 * self._window_size)]

    def update(self, ros_time, duty, pos):
        '''Update the feature vector X
        
            ros_time    the ros_time of the encoder reading
            duty        the commanded duty to the LX-16A
            pos         the measured position on the LX-16A
            label       supervised learning label. True if pos is in the valid region.  
        '''
        self._index = (self._index + 1) % self._window_size
        self._ros_time[self._index] = time
        self._duty[self._index] = duty
        self._pos[self._index] = pos
        
        # array for the next row of the training set          
        self._X = [0 for x in range(3 * self._window_size)]

        # times         
        for i in range(self._window_size):
            idx = (self._index - i) % self._window_size
            dt = (self._ros_time[idx] - self._ros_time[self._index]) * 1.0E-9
            self._X[i] = dt
        
        # duty         
        for i in range(self._window_size):
            idx = (self._index - i)
            duty_i = self._duty[idx]
            self._X[self._window_size + i] = duty_i

        # positions         
        for i in range(self._window_size):
            idx = (self._index - i)
            pos_i = self._pos[idx]
            self._X[2 * self._window_size + i] = pos_i

    def get_pos(self):
        ''' Get the current position (no filter)
        '''
        return self._pos[self._index]

    def get_count(self):
        ''' Get the current count (filtered)
        '''
        return self._pos[self._index]
    

if __name__ == '__main__':
    rospy.loginfo('Lewansoul LX-16A encoder filter')
    rospy.init_node('lx_16a_encoder_filter')

    # Load sample data
