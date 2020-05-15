#!/usr/bin/env python

PKG='lx16a'

from lx16a.encoder import LX16AEncoderFilter

import csv
import math
import os
import rospy
import sys
import unittest

class TestEncoderFilter(unittest.TestCase):
    ''' Test cases for LX16AEncoderFilter
    '''

    def setUp(self):
        # Constants
        self.window = 10

        # Raw data produced by servo and encoder (in lx16a/data)
        self.data_filename = os.path.join(
            os.path.dirname(__file__),
            '..', 'data', 'lx16a_raw_data_05.csv')

        # File locations for persisted ML models (in lx16a/data)
        self.classifier_filename = os.path.join(
            os.path.dirname(__file__),
            '..', 'data', 'lx16a_tree_classifier.joblib')

        self.regressor_filename = os.path.join(
            os.path.dirname(__file__),
            '..', 'data', 'lx16a_tree_regressor.joblib')

    def read_n(self, filter, n):
        ''' Utility to read the first n lines from the CSV file.

        Read the first n lines from the CSV file, update the filter,
        and return the data.
        '''
        
        data = []
        with open(self.data_filename) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            head = [next(reader) for x in range(n)]
            for row in head:
                ros_time = rospy.Time(0, int(row[0]))
                duty = int(row[1])
                pos = int(row[2])
                count = int(row[3])
                data.append([ros_time, duty, pos, count])
                filter.update(ros_time, duty, pos)
        return data

    def test_constructor(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)

        # Values should initialise to zero
        self.assertEquals(filter.get_revolutions(), 0, 'revolutions != 0')
        self.assertEquals(filter.get_count(), 0, 'count != 0')
        self.assertEquals(filter.get_duty(), 0, 'duty != 0')
        self.assertEquals(filter.get_angular_position(), 0, 'angular_position != 0')

        # Values should initialise to tuple (0, True)
        self.assertEquals(filter.get_servo_pos(), (0, True), 'servo_pos != (0 ,True)')

        # Values should initialise to one
        self.assertEquals(filter.get_invert(), 1, 'invert != 1')

    def test_valid_region(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)
        data = self.read_n(filter, 400)

        self.assertEquals(filter.get_revolutions(), 1, 'revolutions != 1')
        self.assertEquals(filter.get_count(), 2134, 'count != 2134')
        self.assertEquals(filter.get_duty(), 250, 'duty != 250')
        self.assertEquals(filter.get_servo_pos(), (634, True), 'servo_pos != 634')
        self.assertEquals(filter.get_invert(), 1, 'invert != 1')

    def test_invalid_region(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)
        data = self.read_n(filter, 100)            

        expected = (1306, False)
        self.assertEquals(filter.get_servo_pos(), expected, 'servo_pos != {}'.format(expected))

    def test_angular_position(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)
        data = self.read_n(filter, 200)

        count = filter.get_count()
        expected = 2 * math.pi * count / 1500.0 
        self.assertEquals(filter.get_angular_position(), expected, 'angular_position != {}'.format(expected))

    def test_duty(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)
        data = self.read_n(filter, 100)

        expected = data[-1][1]
        self.assertEquals(filter.get_duty(), expected, 'duty != {}'.format(expected))

    # Disable this test as it requires a ROS node to be available...
    def notest_reset(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)
        data = self.read_n(filter, 300)
        filter.reset(0)

        # Values should reset to zero
        self.assertEquals(filter.get_revolutions(), 0, 'revolutions != 0')
        self.assertEquals(filter.get_count(), 0, 'count != 0')
        self.assertEquals(filter.get_duty(), 0, 'duty != 0')
        self.assertEquals(filter.get_angular_position(), 0, 'angular_position != 0')

        # Values should reset to tuple (0, True)
        self.assertEquals(filter.get_servo_pos(), (0, True), 'servo_pos != (0 ,True)')

        # Values should reset to one
        self.assertEquals(filter.get_invert(), 1, 'get_invert != 1')

    def test_invert(self):
        # Load the model files
        filter = LX16AEncoderFilter(
            self.classifier_filename, self.regressor_filename, self.window)
        data = self.read_n(filter, 200)
        filter.set_invert(True)

        expected = -1
        self.assertEquals(filter.get_invert(), expected, 'invert != {}'.format(expected))

        count = filter.get_count()
        expected = 2 * math.pi * count / 1500.0
        self.assertEquals(filter.get_angular_position(), expected, 'angular_position != {}'.format(expected))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_encoder_filter', TestEncoderFilter)

