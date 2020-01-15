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

from curio_base.lx16a_encoder_filter import LX16AEncoderFilter
import math
import pandas as pd
import rospy
from std_msgs.msg import Int64

# Constants
DATA_DIR  = './data/'
SAMPLE_ID = '05'
WINDOW    = 10

# Raw data produced by servo and encoder 
RAW_DATA_FILENAME = "{0}lx16a_raw_data_{1}.csv".format(DATA_DIR, SAMPLE_ID)

# Filename for persisted ML model
MODEL_FILENAME = "{0}lx16a_tree_model_all.joblib".format(DATA_DIR)

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
    filter = LX16AEncoderFilter(MODEL_FILENAME, WINDOW)

    # Load data from CSV and assign names to column headings
    df = pd.read_csv(RAW_DATA_FILENAME, header=None, names=['ros_time', 'duty', 'pos', 'count'])
    rospy.loginfo('num. samples = {}'.format(len(df)))
    rospy.loginfo('shape = {}'.format(df.shape))

    start_time = df.loc[0]['ros_time']

    for i in range(0, len(df)):
        # Current dataset row
        row = df.loc[i]

        # Time since start [s]
        dt = (row['ros_time'] - start_time).to_sec()

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
