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
import math
import numpy as np
import pandas as pd
import rospy
from std_msgs.msg import Int64

from sklearn.tree import DecisionTreeClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

# Constants
DATA_DIR = './data/'
SAMPLE_ID = '05'

# Raw data produced by servo and encoder 
RAW_DATA_FILENAME = "{0}lx16a_raw_data_{1}.csv".format(
    DATA_DIR, SAMPLE_ID)

# Filename for persisted ML model
# MODEL_FILENAME = "{0}lx16a_mpl_model_all.joblib".format(DATA_DIR)

# Filename for persisted ML model
MODEL_FILENAME = "{0}lx16a_tree_model_all.joblib".format(DATA_DIR)

ENCODER_MIN    = 0      # minimum servo position reading
ENCODER_MAX    = 1500   # maximum servo position reading
ENCODER_LOWER  = 1190   # lower bound of the invalid range
ENCODER_UPPER  = 1310   # upper bound of the invalid range
ENCODER_STEP   = 1400   # threshold for determining the encoder has completed a revolution
ENCODER_WINDOW = 10     # length of servo history used in the ML classifier

class LX16AEncoderFilter(object):
    ''' Encoder filter for the LX-16A servo.
    '''
    def __init__(self, window):        
        '''Constructor        
        '''
        # Initialise rotating buffers that store the encoder history.
        self._window = window
        self._index     = 0
        self._ros_time  = [0 for x in range(window)]
        self._duty      = [0 for x in range(window)]
        self._pos       = [0 for x in range(window)]
        self._X         = [0 for x in range(3 * window)]
        self._clf_pipe  = None
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
            dt = (self._ros_time[idx] - self._ros_time[self._index]) * 1.0E-9
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
        is_valid = self._clf_pipe.predict([self._X])[0]
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
        is_valid = self._clf_pipe.predict([self._X])[0]
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
        rospy.loginfo('loading MPL classifier from {}'.format(MODEL_FILENAME))
        self._clf_pipe = joblib.load(MODEL_FILENAME)


if __name__ == '__main__':
    ''' A test node for the LX-16A encoder filter.

    This example replays a raw data file containing ros_time, duty, pos, count
    data for the servo through the encoder filter.
    The encoder count is published to the topic /encoder.
    The data is processes as fast as possible, so timestamps are not correct. 
    '''
    rospy.loginfo('Lewansoul LX-16A encoder filter (test)')
    rospy.init_node('lx_16a_encoder_filter_test')

    # Publisher
    encoder_msg = Int64()
    encoder_pub = rospy.Publisher('/encoder', Int64, queue_size=10)

    # Encoder filter
    filter = LX16AEncoderFilter(ENCODER_WINDOW)

    # Load data from CSV and assign names to column headings
    df = pd.read_csv(RAW_DATA_FILENAME, header=None, names=['ros_time', 'duty', 'pos', 'count'])
    rospy.loginfo('num. samples = {}'.format(len(df)))
    rospy.loginfo('shape = {}'.format(df.shape))

    start_time = df.loc[0]['ros_time']

    for i in range(0, len(df)):
        # Current dataset row
        row = df.loc[i]

        # Time since start [s]
        dt = (row['ros_time'] - start_time) * 1.0E-9

        # Update the filter
        filter.update(row['ros_time'], row['duty'], row['pos'])
        servo_pos, is_valid = filter.get_servo_pos()
        count               = filter.get_count()
        revolutions         = filter.get_revolutions()

        rospy.logdebug('time: {:.2f}, duty: {}, pos: {}, is_valid: {}'
            .format(dt, row['duty'], servo_pos, is_valid))

        rospy.loginfo('time: {:.2f}, duty: {}, count: {}, rev: {}'
            .format(dt, row['duty'], count, revolutions))

        encoder_pub.publish(count)
