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
import rospy

from sklearn.tree import DecisionTreeClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline

ENCODER_MIN    = 0      # minimum servo position reading
ENCODER_MAX    = 1500   # maximum servo position reading
ENCODER_LOWER  = 1190   # lower bound of the invalid range
ENCODER_UPPER  = 1310   # upper bound of the invalid range
ENCODER_STEP   = 1000   # threshold for determining the encoder has
                        # completed a revolution

class LX16AEncoderFilter(object):
    ''' An encoder filter for the LX-16A servo.

    This class loads a `scikit-learn` decision tree classifier
    which is used to predict whether or not a position
    obtained from a LX-16A servo lies within its valid measurement
    region which covers a range of about 330 deg.

    The LX-16A has 1500 counts per revolution giving an
    angular resolution of 0.24 deg.

    The class maintains a count of the number of full revolutions
    made by the servo (positive and negative) and uses this and
    the servo postion to determine the overall encoder count.

    There is also an optional facilty to estimate the servo position
    in the invalid region using a decision tree regressor. This is
    enabled by supplying the constructor with a filename for the
    regressor model. 
    '''

    def __init__(self, classifier_filename, regressor_filename=None, window=10):
        '''Constructor        

        Parameters
        ----------
        classifier_filename : str
            The file name of the scikit-learn decision tree classifier
        regressor_filename : str
            The file name of the scikit-learn decision tree regressor,
            has (default None)
        window : int
            The size of the sample window used in the classifier,
            has default 10)
        '''

        # Initialise ring buffers that store the encoder history.
        self._window = window
        self._index     = 0         # index for the ring buffers
        self._ros_time  = [rospy.Time() for x in range(window)]
        self._duty      = [0.0 for x in range(window)]
        self._pos       = [0.0 for x in range(window)]
        self._X         = [0.0 for x in range(3 * window)]
        self._classifier_filename = classifier_filename
        self._regressor_filename  = regressor_filename
        self._classifier     = None  # scikit-learn classifier
        self._regressor      = None  # scikit-learn regressor
        self._count_offset   = 0     # set to ensure count=0 when reset
        self._revolutions    = 0     # number of revolutions since reset
        self._prev_valid_pos = 0     # the previous valid position
        self._invert         = 1     # 1 or -1 depending on the desired
                                     # direction for increasing count

        # Load the ML classifier pipeline
        self._load_classifier()

        # Load the ML regressor pipeline
        self._load_regressor()

    def update(self, ros_time, duty, pos):
        '''Update the encoder filter.

        Update the encoder and estimate whether or not the new servo
        position is in the valid range.

        The feature vector X contains 3 * window entries:
            dt[window]     the change in ros_time between servo
                           position readings
            duty[window]   the commanded duty to the LX-16A
            pos[window]    the measured position on the LX-16A
        
        Parameters
        ----------
        ros_time: rospy.Time
            The time from rospy.get_rostime()
        duty : int
            The servo duty
        pos : int
            The servo position        
        '''

        # Update the ring buffers
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

        # @TODO: this section would benefit from restructuring,
        #        once the regression logic has been finalised.
        #  
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

        elif self._regressor is not None:
            # Not valid - so we'll try to use the regressor to predict
            # a value for the encoder count.
            pos_est = int(self._regressor.predict([self._X])[0]) % ENCODER_MAX

            # @DEBUG_INFO
            count_est = pos_est + ENCODER_MAX * self._revolutions 
            rospy.logdebug("count_est: {}".format(count_est))

            # @TODO: the acceptance criteria may need further tuning.
            # The classifier will sometimes report false positives.
            # To limit the impact of using a regressed value in this
            # case we check that the previous position is 'close' to
            # one of the boundaries. 
            dist1 = abs(ENCODER_LOWER - self._prev_valid_pos)
            dist2 = abs(ENCODER_UPPER - self._prev_valid_pos)
            dist  = min(dist1, dist2)
            DIST_MAX = (ENCODER_UPPER - ENCODER_LOWER)/2 + 5

            # Accept the estimated position if in the range where the
            # encoder does not report valid values: [1190, 1310]
            if dist < DIST_MAX and pos_est >= ENCODER_LOWER and pos_est <= ENCODER_UPPER:
                # If the absolute change in the servo position is 
                # greater than ENCODER_STEP then we increment
                # (or decrement) the revolution counter.
                delta = pos_est - self._prev_valid_pos
                if delta > ENCODER_STEP:
                    self._revolutions = self._revolutions - 1
                if delta < -ENCODER_STEP:
                    self._revolutions = self._revolutions + 1

                # Update the previous valid position
                self._prev_valid_pos = pos_est

    def get_revolutions(self):
        ''' Get the number of revoutions since reset.

        Returns
        -------
        int
            The number of revolutions since the count was reset.
        '''

        return self._revolutions

    def get_count(self):
        ''' Get the current encoder count since reset (filtered).

        Note that the encoder count is offset from the servo position
        so that the count is zero when the encoder filter is reset.

        Returns
        -------
        int
            The current encoder count.
        '''

        count = self._prev_valid_pos + ENCODER_MAX * self._revolutions 
        return self._invert * (count - self._count_offset) 

    def get_duty(self):
        ''' Get the current encoder duty.

        Returns
        -------
        int
            The current encoder duty.
        '''

        duty = pos = self._duty[self._index] 
        return duty 

    def get_angular_position(self):
        ''' Get the angular position of the encoder (filtered)

        Returns
        -------
        float
            The angular position of the encoder [rad].
        '''

        return 2.0 * math.pi * self.get_count() / ENCODER_MAX
    
    def get_servo_pos(self, map_pos=True):
        ''' Get the current (no-filtered) servo position and an
        estimate whether it is valid.

        Parameters
        ----------
        map_pos : bool
            If True map the position to the range [0, 1500],
            has (default True)
        
        Returns
        -------
        list
            A two element list containing the position and a
            bool which is True for valid, False otherwise.
        '''
        
        is_valid = self._classifier.predict([self._X])[0] != 0
        if map_pos:
            pos = self._pos[self._index] % ENCODER_MAX
            return pos, is_valid
        else:
            pos = self._pos[self._index]
            return pos, is_valid    

    def get_invert(self):
        ''' Get the invert state: whether the encoder count is inverted.

        Returns
        -------
        int
            -1 if the count is inverted, 1 otherwise. 
        '''

        return self._invert

    def set_invert(self, is_inverted):
        ''' Invert the direction of the encoder count.

        Parameters
        ----------
        is_inverted : bool
            Set to True if the encoder count is reversed.
        '''

        self._invert = -1 if is_inverted else 1

    def reset(self, pos):
        ''' Reset the encoder counters to zero.

        Parameters
        ----------
        pos : int
            The (assumed valid) position of the servo when the
            encoder is reset.  
        '''

        # Back-populate the ring buffers with zero duty entries.
        now = rospy.get_rostime()
        for i in range(self._window):
            t = now - rospy.Duration((self._window - i)/50.0)
            self.update(t, 0, pos)  

        # Calculate the offset to zero the counter
        pos, is_valid = self.get_servo_pos()
        if is_valid:
            self._count_offset = pos

        # Initialise remaining variables
        self._revolutions    = 0
        self._prev_valid_pos = pos

    def _load_classifier(self):
        ''' Load classifier

        Load a `scikit-learn` classifier from file.

        Note that this function uses joblib (and so pickle). It is
        sensitive to the version of Python and a number of packages
        including:
        
        - numpy
        - scipy
        - scikit-learn
        
        '''
        
        rospy.loginfo('Loading classifier from {}'.format(self._classifier_filename))
        self._classifier = joblib.load(self._classifier_filename)

    def _load_regressor(self):
        ''' Load regressor

        See comments for _load_classifier.        
        '''
        
        if self._regressor_filename is not None:
            rospy.loginfo('Loading regressor from {}'.format(self._regressor_filename))
            self._regressor = joblib.load(self._regressor_filename)
