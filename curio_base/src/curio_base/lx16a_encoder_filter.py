#!/usr/bin/env python
# 
# coding: latin-1
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

import joblib
import rospy

from sklearn.tree import DecisionTreeClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

ENCODER_MIN    = 0      # minimum servo position reading
ENCODER_MAX    = 1500   # maximum servo position reading
ENCODER_LOWER  = 1190   # lower bound of the invalid range
ENCODER_UPPER  = 1310   # upper bound of the invalid range
ENCODER_STEP   = 1400   # threshold for determining the encoder has completed a revolution

class LX16AEncoderFilter(object):
    ''' Encoder filter for the LX-16A servo.
    '''
    def __init__(self, filename, window=10):        
        '''Constructor        

        Parameters
            filename    filename for the scikit-learn decision tree classifier
            window      the size of the sample window used in the classifier
        '''
        # Initialise rotating buffers that store the encoder history.
        self._window = window
        self._index     = 0
        self._ros_time  = [rospy.Time() for x in range(window)]
        self._duty      = [0.0 for x in range(window)]
        self._pos       = [0.0 for x in range(window)]
        self._X         = [0.0 for x in range(3 * window)]
        self._filename       = filename
        self._classifier     = None
        self._revolutions    = 0
        self._prev_valid_pos = 0

        # Load the ML classifier pipeline
        self._load_classifier()

    def update(self, ros_time, duty, pos):
        '''Update the encoder.

        Update the encoder and estimate whether or not the new servo
        position is in the valid range.

        The feature vector X contains 3 * window entries:
            dt[window]     the change in ros_time between servo postion readings
            duty[window]   the commanded duty to the LX-16A
            pos[window]    the measured position on the LX-16A
        '''
        # Update the history buffers
        self._index = (self._index + 1) % self._window
        self._ros_time[self._index] = ros_time
        self._duty[self._index] = duty
        self._pos[self._index] = pos
                
        # times
        for i in range(self._window):
            idx = (self._index - i) % self._window
            dt = (self._ros_time[idx] - self._ros_time[self._index]).to_sec()
            self._X[i] = dt
        
        # duty 
        for i in range(self._window):
            idx = (self._index - i)
            duty_i = self._duty[idx]
            self._X[self._window + i] = duty_i

        # positions
        for i in range(self._window):
            idx = (self._index - i)
            pos_i = self._pos[idx]
            self._X[2 * self._window + i] = pos_i

        # Apply the filter and update the encoder counters
        pos = self._pos[self._index] % ENCODER_MAX
        is_valid = self._classifier.predict([self._X])[0]
        if is_valid:
            # If the absolute change in the servo position is 
            # greater than ENCODER_STEP then we increment / decrement
            # the revolution counter.
            delta = pos - self._prev_valid_pos
            if delta > ENCODER_STEP:
                self._revolutions = self._revolutions - 1
            if delta < -ENCODER_STEP:
                self._revolutions = self._revolutions + 1

            # Update the previous valid position
            self._prev_valid_pos = pos

    def get_revolutions(self):
        ''' Get the number of revoutions since reset.
        '''
        return self._revolutions

    def get_count(self):
        ''' Get the current encoder count since reset (filtered).
        '''
        return self._prev_valid_pos + ENCODER_MAX * self._revolutions

    def get_servo_pos(self, zero_offset=True):
        ''' Get the current servo position and an estimate if it is valid.

        Parameters:
            zero_offset If True map the position to the range [0, 1500].
        
        Returns:
            pos, is_valid
        '''
        is_valid = self._classifier.predict([self._X])[0]
        if zero_offset:
            pos = self._pos[self._index] % ENCODER_MAX
            return pos, is_valid
        else:
            pos = self._pos[self._index]
            return pos, is_valid    
    
    def reset(self):
        ''' Reset the encoder counters to zero.
        '''
        self._revolutions    = 0
        self._prev_valid_pos = 0

    def _load_classifier(self):
        ''' Load classifier

        Note that this function uses joblib (and so pickle). It is sensitive
        to the version of Python and a number of packages including:
        
        - numpy
        - scipy
        - scikit-learn
        
        '''
        rospy.loginfo('loading classifier from {}'.format(self._filename))
        self._classifier = joblib.load(self._filename)

