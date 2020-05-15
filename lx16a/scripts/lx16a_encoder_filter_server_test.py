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

''' Lewansoul LX-16A encoder filter server test.
'''

from lx16a_msgs.srv import EncoderFilterAdd, EncoderFilterAddResponse
from lx16a_msgs.srv import EncoderFilterGetAngularPosition, EncoderFilterGetAngularPositionResponse
from lx16a_msgs.srv import EncoderFilterGetCount, EncoderFilterGetCountResponse
from lx16a_msgs.srv import EncoderFilterGetDuty, EncoderFilterGetDutyResponse
from lx16a_msgs.srv import EncoderFilterGetInvert, EncoderFilterGetInvertResponse
from lx16a_msgs.srv import EncoderFilterGetRevolutions, EncoderFilterGetRevolutionsResponse
from lx16a_msgs.srv import EncoderFilterGetServoPos, EncoderFilterGetServoPosResponse
from lx16a_msgs.srv import EncoderFilterReset, EncoderFilterResetResponse
from lx16a_msgs.srv import EncoderFilterSetInvert, EncoderFilterSetInvertResponse
from lx16a_msgs.srv import EncoderFilterUpdate, EncoderFilterUpdateResponse
import rospy
import math
from os import path
import pandas as pd
import rospy

# Constants
SERVO_ID = 21
WINDOW = 10

# Raw data produced by servo and encoder 

# File locations for persisted ML models
DATA_DIR = "./src/curio/lx16a/data"
RAW_DATA_FILENAME = "lx16a_raw_data_05.csv"
CLASSIFIER_FILENAME = "lx16a_tree_classifier.joblib"
REGRESSOR_FILENAME  = "lx16a_tree_regressor.joblib"

if __name__ == '__main__':
    ''' A test node for the LX-16A encoder filter server.

    '''
    rospy.init_node('lx16a_encoder_filter_server_test')

    rospy.wait_for_service('lx16a/encoder_filter/add')
    rospy.loginfo('Lewansoul LX-16A encoder filter server (test)')
    try:
        # Set up service proxies
        filter_add = rospy.ServiceProxy('lx16a/encoder_filter/add', EncoderFilterAdd)
        filter_get_angular_position = rospy.ServiceProxy('lx16a/encoder_filter/get_angular_position', EncoderFilterGetAngularPosition)
        filter_get_count = rospy.ServiceProxy('lx16a/encoder_filter/get_count', EncoderFilterGetCount)
        filter_get_duty = rospy.ServiceProxy('lx16a/encoder_filter/get_duty', EncoderFilterGetDuty)
        filter_get_invert = rospy.ServiceProxy('lx16a/encoder_filter/get_invert', EncoderFilterGetInvert)
        filter_get_revolutions = rospy.ServiceProxy('lx16a/encoder_filter/get_revolutions', EncoderFilterGetRevolutions)
        filter_get_servo_pos = rospy.ServiceProxy('lx16a/encoder_filter/get_servo_pos', EncoderFilterGetServoPos)
        filter_reset = rospy.ServiceProxy('lx16a/encoder_filter/reset', EncoderFilterReset)
        filter_set_invert = rospy.ServiceProxy('lx16a/encoder_filter/set_invert', EncoderFilterSetInvert)
        filter_update = rospy.ServiceProxy('lx16a/encoder_filter/update', EncoderFilterUpdate)

        # Add an encoder filter for servo_id 11
        status = filter_add(
            SERVO_ID,
            path.join(DATA_DIR, CLASSIFIER_FILENAME),
            path.join(DATA_DIR, REGRESSOR_FILENAME),
            WINDOW)
        rospy.loginfo('status: {}'.format(status))

        # Load data from CSV and assign names to column headings
        df = pd.read_csv(path.join(DATA_DIR, RAW_DATA_FILENAME), header=None, names=['ros_time', 'duty', 'pos', 'count'])
        rospy.loginfo('num. samples = {}'.format(len(df)))
        rospy.loginfo('shape = {}'.format(df.shape))

        start_time = rospy.Time(0, df.loc[0]['ros_time'])

        # for i in range(0, len(df)):
        for i in range(0, 100):
            # Current dataset row
            row = df.loc[i]

            # Time since start [s]
            dt = (rospy.Time(0, row['ros_time']) - start_time).to_sec()

            # Update the filter
            filter_update(SERVO_ID, rospy.Time(0, row['ros_time']), row['duty'], row['pos'])
            resp = filter_get_servo_pos(SERVO_ID)
            servo_pos = resp.position
            is_valid = resp.is_valid 
            count = filter_get_count(SERVO_ID).count
            revolutions = filter_get_revolutions(SERVO_ID).revolutions

            rospy.logdebug('time: {:.2f}, duty: {}, pos: {}, is_valid: {}'
                .format(dt, row['duty'], servo_pos, is_valid))

            rospy.loginfo('time: {:.2f}, duty: {}, count: {}, rev: {}'
                .format(dt, row['duty'], count, revolutions))


    except rospy.ServiceException as err:
        print('Service called failed: {}'.format(err))

    