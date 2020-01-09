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

''' LX-16A Training Data

    Create labelled training data for a ML routine to predict gaps
    in the position data for a LX-16A servo.

    Notes:

    1. A E6B2-CWZ3E rotary encoder records 4096 counts per revolution.
    2. A LX-16A servo records 1000 counts in 240 deg, so 1500 counts per revolution.
    3. The electrical range of the ALPS ALPINE RDC50 rotary encoder used in the LX-16A is 333 deg.

    This means that LX-16A servo positions between -194 and 1194 should be in the valid range.
    If we scale the rotary encoder count to lie between 0 and 1500, and align the positions
    at 0 using an offset, it means the invalid range lies between 1194 and 1306 as measured by
    the rotary encoder. We set bounds at 1190 and 1310.  

    To calibrate the offset for each data set we would ideally have at least one revolution
    at constant duty at the start of the timeseries.
'''

import math
import numpy as np
import pandas as pd

INPUT_FILENAME = '~/Code/robotics/curio/curio_ws/data/lx16a_data.csv'
OUTPUT_FILENAME = '~/Code/robotics/curio/curio_ws/data/lx16a_training_set.csv'

OFFSET = 1160
LOWER  = 1190
UPPER  = 1310

class TrainingSetBuilder(object):
    ''' Utility to build a ML training set from a timeseries of labelled encoder data.
    
        Each row of  the training data contains:
        WINDOW_SIZE time deltas (increasingly negative) back to the previous encoder positions
        WINDOW_SIZE positions - the previous values of the LX-16A servo
        WINDOW_SIZE duty - the commanded duty at each previous time
        1           label - the classification label (True or False) whether the position is valid
    '''

    WINDOW_SIZE = 10

    def __init__(self):        
        '''Constructor
        
            Initialise rotating buffers that store the encoder history.
        '''
        self._index = 0
        self._time  = [0 for x in range(TrainingSetGen.WINDOW_SIZE)]
        self._duty  = [0 for x in range(TrainingSetGen.WINDOW_SIZE)]
        self._pos   = [0 for x in range(TrainingSetGen.WINDOW_SIZE)]
    
    def _make_row(self, time, duty, pos, label):
        '''Make a single row of the training set
        
            time    the time of the encoder reading
            duty    the commanded duty to the LX-16A
            pos     the measured position on the LX-16A
            label   supervised learning label. True if pos is in the valid region.  
        '''
        self._index = (self._index + 1) % TrainingSetGen.WINDOW_SIZE
        self._time[self._index] = time
        self._duty[self._index] = duty
        self._pos[self._index] = pos
        
        # array for the next row of the training set          
        row = [0 for x in range(3 * TrainingSetGen.WINDOW_SIZE + 1)]

        # times         
        for i in range(TrainingSetGen.WINDOW_SIZE):
            idx = (self._index - i) % TrainingSetGen.WINDOW_SIZE
            dt = (self._time[idx] - self._time[self._index]) * 1.0E-9
            row[i] = dt
        
        # duty         
        for i in range(TrainingSetGen.WINDOW_SIZE):
            idx = (self._index - i)
            duty_i = self._duty[idx]
            row[TrainingSetGen.WINDOW_SIZE + i] = duty_i

        # positions         
        for i in range(TrainingSetGen.WINDOW_SIZE):
            idx = (self._index - i)
            pos_i = self._pos[idx]
            row[2 * TrainingSetGen.WINDOW_SIZE + i] = pos_i
        
        row[3 * TrainingSetGen.WINDOW_SIZE] = label
        
        return row
    
    def make_training_set(self, encoder_df):
        ''' Make a training set from a DataFrame containing labelled encoder data.
                
            Returns: a DataFrame containing the training set.
        '''
        # Load the first TrainingSetGen.WINDOW_SIZE rows of history
        for i in range(0, TrainingSetGen.WINDOW_SIZE):
            self._make_row(
                encoder_df.loc[i]['ros_time'],
                encoder_df.loc[i]['duty'],
                encoder_df.loc[i]['pos'],
                encoder_df.loc[i]['label'])

        # Generate training set
        ts_array = []
        for i in range(TrainingSetGen.WINDOW_SIZE, len(df)):
            ts_array.append(self._make_row(
                encoder_df.loc[i]['ros_time'],
                encoder_df.loc[i]['duty'],
                encoder_df.loc[i]['pos'],
                encoder_df.loc[i]['label']))

        # Convert to a DataFrame
        ts_df = pd.DataFrame(ts_array,
                             index=range(len(ts)),
                             columns=['dt0','dt1','dt2','dt3','dt4',
                                      'dt5','dt6','dt7','dt8','dt9',
                                      'duty0','duty1','duty2','duty3','duty4',
                                      'duty5','duty6','duty7','duty8','duty9',
                                      'pos0','pos1','pos2','pos3','pos4',
                                      'pos5','pos6','pos7','pos8','pos9',
                                      'label'])
        return ts_df
        

if __name__ == '__main__':

    # Load data from CSV and assign names to column headings 
    df = pd.read_csv(INPUT_FILENAME, header=None, names=['ros_time', 'duty', 'pos', 'count'])

    # Generate the label column. This assumes the OFFSET has been calibrated corectly. 
    encoder = ((df['count'] - OFFSET) % 4096) * 1500 / 4096
    label = np.logical_not(np.logical_and(encoder > LOWER, encoder < UPPER))
    df['encoder'] = pd.Series(encoder)
    df['label']   = pd.Series(label)

    # Generate the training set
    builder = TrainingSetBuilder()
    ts_df = builder.make_training_set(df)

    # Write the training set to csv
    ts_df.to_csv(OUT_FILENAME)