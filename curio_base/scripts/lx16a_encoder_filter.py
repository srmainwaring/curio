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
WINDOW_SIZE = 10

# Raw data produced by servo and encoder 
RAW_DATA_FILENAME = "{0}lx16a_raw_data_{1}.csv".format(
    DATA_DIR, SAMPLE_ID)

# Filename for persisted ML model
# MODEL_FILENAME = "{0}lx16a_mpl_model_all.joblib".format(DATA_DIR)

# Filename for persisted ML model
MODEL_FILENAME = "{0}lx16a_tree_model_all.joblib".format(DATA_DIR)

ENCODER_MIN    = 0
ENCODER_MAX    = 1500
ENCODER_LOWER  = 1190
ENCODER_UPPER  = 1310
ENCODER_OFFSET = ENCODER_MAX - ENCODER_UPPER

class LX16AEncoderFilter(object):
    ''' Encoder filter for the LX-16A servo.
    '''
    def __init__(self, window_size):        
        '''Constructor        

        The feature row vector contains:
        window_size time deltas (increasingly negative) back to the previous encoder positions
        window_size positions - the previous values of the LX-16A servo
        window_size duty - the commanded duty at each previous time
        '''
        # Initialise rotating buffers that store the encoder history.
        self._window_size = window_size
        self._index     = 0
        self._ros_time  = [0 for x in range(window_size)]
        self._duty      = [0 for x in range(window_size)]
        self._pos       = [0 for x in range(window_size)]
        self._X         = [0 for x in range(3 * window_size)]
        self._clf_pipe  = None

        # Load the ML classifier pipeline
        self._load_classifier()

    def update(self, ros_time, duty, pos):
        '''Update the feature vector X
        
            ros_time    the ros_time of the encoder reading
            duty        the commanded duty to the LX-16A
            pos         the measured position on the LX-16A
            label       supervised learning label. True if pos is in the valid region.  
        '''
        self._index = (self._index + 1) % self._window_size
        self._ros_time[self._index] = ros_time
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

            Returns a tuple: position, is_valid
        '''
        pos = self._pos[self._index] % ENCODER_MAX
        is_valid = self._clf_pipe.predict([self._X])[0]
        rospy.logdebug('pos: {}, is_valid: {}'.format(pos, is_valid)) 
        return pos, is_valid
    
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
    rospy.loginfo('Lewansoul LX-16A encoder filter')
    rospy.init_node('lx_16a_encoder_filter')

    # Publisher
    encoder_msg = Int64()
    encoder_pub = rospy.Publisher('/encoder', Int64, queue_size=10)

    # Encoder filter
    filter = LX16AEncoderFilter(WINDOW_SIZE)

    # Load data from CSV and assign names to column headings
    df = pd.read_csv(RAW_DATA_FILENAME, header=None, names=['ros_time', 'duty', 'pos', 'count'])
    rospy.loginfo('num. samples = {}'.format(len(df)))
    rospy.loginfo('shape = {}'.format(df.shape))

    start_time = df.loc[0]['ros_time']

    prev_count = 0
    for i in range(0, len(df)):
        # Current dataset row
        row = df.loc[i]

        # Time since start [s]
        dt = (row['ros_time'] - start_time) * 1.0E-9

        # Update the filter
        filter.update(row['ros_time'], row['duty'], row['pos'])
        count, is_valid = filter.get_count()

        rospy.logdebug('time: {:.2f}, duty: {}, count: {}, is_valid: {}'
            .format(dt, row['duty'], count, is_valid))

        if is_valid:
            rospy.loginfo('time: {:.2f}, duty: {}, count: {}'.format(dt, row['duty'], count))
            encoder_pub.publish(count)
            prev_count = count
        else:
            encoder_pub.publish(prev_count)
