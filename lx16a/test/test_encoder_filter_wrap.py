#!/usr/bin/env python

PKG='lx16a'


import csv
import math
import os
import rospy
import sys
import unittest
from lx16a.lx16a_encoder_filter_wrap import LX16AEncoderFilterWrap

# Constants
SERVO_ID = 11
WINDOW = 10

# Raw data produced by servo and encoder (in lx16a/data)
RAW_DATA_FILENAME = os.path.join(
    os.path.dirname(__file__),
    '..', 'data', 'lx16a_raw_data_05.csv')

# File locations for persisted ML models (in lx16a/data)
CLASSIFIER_FILENAME = os.path.join(
    os.path.dirname(__file__),
    '..', 'data', 'lx16a_tree_classifier.joblib')

REGRESSOR_FILENAME = os.path.join(
    os.path.dirname(__file__),
    '..', 'data', 'lx16a_tree_regressor.joblib')

def read_n(filter, SERVO_ID, n):
    ''' Utility to read the first n lines from the CSV file.

    Read the first n lines from the CSV file, update the filter,
    and return the data.
    '''
    
    data = []
    with open(RAW_DATA_FILENAME) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        head = [next(reader) for x in range(n)]
        for row in head:
            ros_time = rospy.Time(0, int(row[0]))
            duty = int(row[1])
            pos = int(row[2])
            count = int(row[3])
            data.append([ros_time, duty, pos, count])
            filter.update(SERVO_ID, ros_time, duty, pos)
    return data

class TestEncoderFilteWrap(unittest.TestCase):
    ''' Test cases for LX16AEncoderFilterWrap
    '''

    def test_constructor(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)

        # Values should initialise to zero
        self.assertEquals(filter.get_revolutions(SERVO_ID), 0, 'get_revolutions != 0')
        self.assertEquals(filter.get_count(SERVO_ID), 0, 'get_count != 0')
        self.assertEquals(filter.get_duty(SERVO_ID), 0, 'get_duty != 0')
        self.assertEquals(filter.get_angular_position(SERVO_ID), 0, 'get_angular_position != 0')

        # Values should initialise to tuple (0, True)
        # self.assertEquals(filter.get_servo_position(SERVO_ID), (0, True), 'get_servo_pos != (0 ,True)')

        # Values should initialise to one
        self.assertEquals(filter.get_invert(SERVO_ID), 1, 'get_invert != 1')

    def test_valid_region(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)
        data = read_n(filter, SERVO_ID, 400)

        self.assertEquals(filter.get_revolutions(SERVO_ID), 1, 'get_revolutions != 1')
        self.assertEquals(filter.get_count(SERVO_ID), 2134, 'get_count != 2134')
        self.assertEquals(filter.get_duty(SERVO_ID), 250, 'get_duty != 250')
        # self.assertEquals(filter.get_servo_pos(SERVO_ID), (634, True), 'get_servo_pos != 634')
        self.assertEquals(filter.get_invert(SERVO_ID), 1, 'get_invert != 1')

    def test_invalid_region(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)
        data = read_n(filter, SERVO_ID, 100)            

        expected = (1306, False)
        # self.assertEquals(filter.get_servo_pos(SERVO_ID), expected, 'get_servo_pos != {}'.format(expected))

    def test_angular_position(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)
        data = read_n(filter, SERVO_ID, 200)

        count = filter.get_count(SERVO_ID)
        expected = 2 * math.pi * count / 1500.0 
        self.assertEquals(filter.get_angular_position(SERVO_ID), expected, 'get_angular_position != {}'.format(expected))

    def test_duty(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)
        data = read_n(filter, SERVO_ID, 100)

        expected = data[-1][1]
        self.assertEquals(filter.get_duty(SERVO_ID), expected, 'get_duty != {}'.format(expected))

    # Disable this test as it requires a ROS node to be available...
    def notest_reset(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)
        data = read_n(filter, SERVO_ID, 300)
        filter.reset(SERVO_ID, 0)

        # Values should reset to zero
        self.assertEquals(filter.get_revolutions(SERVO_ID), 0, 'get_revolutions != 0')
        self.assertEquals(filter.get_count(SERVO_ID), 0, 'get_count != 0')
        self.assertEquals(filter.get_duty(SERVO_ID), 0, 'get_duty != 0')
        self.assertEquals(filter.get_angular_position(SERVO_ID), 0, 'get_angular_position != 0')

        # Values should reset to tuple (0, True)
        # self.assertEquals(filter.get_servo_pos(SERVO_ID), (0, True), 'get_servo_pos != (0 ,True)')

        # Values should reset to one
        self.assertEquals(filter.get_invert(SERVO_ID), 1, 'get_invert != 1')

    def test_invert(self):
        # Load the model files
        filter = LX16AEncoderFilterWrap(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)
        filter.add(SERVO_ID)
        data = read_n(filter, SERVO_ID, 200)
        filter.set_invert(SERVO_ID, True)

        expected = -1
        self.assertEquals(filter.get_invert(SERVO_ID), expected, 'get_invert != {}'.format(expected))

        count = filter.get_count(SERVO_ID)
        expected = 2 * math.pi * count / 1500.0
        self.assertEquals(filter.get_angular_position(SERVO_ID), expected, 'get_angular_position != {}'.format(expected))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_encoder_filter_wrap', TestEncoderFilterWrap)

