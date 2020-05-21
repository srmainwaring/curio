#!/usr/bin/env python

PKG='lx16a'

from lx16a.ext import LX16AEncoderFilterSklearn
from lx16a.ext import Time

import csv
import math
import os
import rospy
import sys
import unittest

class TestExtEncoderFilter(unittest.TestCase):
    ''' Test cases for extension LX16AEncoderFilterSklearn
    '''

    def setUp(self):
        # Constants
        self.servo_id = 11
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
        ''' Utility to read the first n lines from a CSV file.

        Read the first n lines from the CSV file, update the filter,
        and return the data.

        Use the vector version filter.update_v.
        '''
        
        data = []
        with open(self.data_filename) as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            head = [next(reader) for x in range(n)]
            for row in head:
                nsec = int(row[0])
                secs = nsec * 1E-9 
                time = Time(secs)
                duty = int(row[1])
                pos = int(row[2])
                count = int(row[3])
                data.append([time, duty, pos, count])
                filter.update_v([self.servo_id], time, [duty], [pos])
        return data

    def test_has_method_attributes(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)

        self.assertTrue(hasattr(filter, 'add'), "missing attr 'add'")
        self.assertTrue(hasattr(filter, 'update'), "missing attr 'update'")
        self.assertTrue(hasattr(filter, 'get_revolutions'), "missing attr 'get_revolutions'")
        self.assertTrue(hasattr(filter, 'get_count'), "missing attr 'get_count'")
        self.assertTrue(hasattr(filter, 'get_duty'), "missing attr 'get_duty'")
        self.assertTrue(hasattr(filter, 'get_angular_position'), "missing attr 'get_angular_position'")
        self.assertTrue(hasattr(filter, 'get_servo_position'), "missing attr 'get_servo_position'")
        self.assertTrue(hasattr(filter, 'get_invert'), "missing attr 'get_invert'")
        self.assertTrue(hasattr(filter, 'set_invert'), "missing attr 'set_invert'")
        self.assertTrue(hasattr(filter, 'reset'), "missing attr 'reset'")
        self.assertTrue(hasattr(filter, 'add_v'), "missing attr 'add_v'")
        self.assertTrue(hasattr(filter, 'update_v'), "missing attr 'update_v'")

    def test_constructor(self):
        # Load the model files
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)

        # Values should initialise to zero
        self.assertEquals(filter.get_revolutions(self.servo_id), 0, 'revolutions != 0')
        self.assertEquals(filter.get_count(self.servo_id), 0, 'count != 0')
        self.assertEquals(filter.get_duty(self.servo_id), 0, 'duty != 0')
        self.assertEquals(filter.get_angular_position(self.servo_id), 0, 'angular_position != 0')

        # Values should initialise to tuple (0, True)
        self.assertEquals(filter.get_servo_position(self.servo_id), (0, True), 'servo_pos != (0 ,True)')

        # Values should initialise to one
        self.assertEquals(filter.get_invert(self.servo_id), 1, 'invert != 1')

    def test_add_v(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add_v([self.servo_id])

        # Values should initialise to zero
        self.assertEquals(filter.get_revolutions(self.servo_id), 0, 'revolutions != 0')
        self.assertEquals(filter.get_count(self.servo_id), 0, 'count != 0')
        self.assertEquals(filter.get_duty(self.servo_id), 0, 'duty != 0')
        self.assertEquals(filter.get_angular_position(self.servo_id), 0, 'angular_position != 0')

        # Values should initialise to tuple (0, True)
        self.assertEquals(filter.get_servo_position(self.servo_id), (0, True), 'servo_pos != (0 ,True)')

        # Values should initialise to one
        self.assertEquals(filter.get_invert(self.servo_id), 1, 'invert != 1')

    def test_valid_region(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)
        data = self.read_n(filter, 400)

        self.assertEquals(filter.get_revolutions(self.servo_id), 1, 'revolutions != 1')
        self.assertEquals(filter.get_count(self.servo_id), 2134, 'count != 2134')
        self.assertEquals(filter.get_duty(self.servo_id), 250, 'duty != 250')
        self.assertEquals(filter.get_servo_position(self.servo_id), (634, True), 'servo_pos != 634')
        self.assertEquals(filter.get_invert(self.servo_id), 1, 'invert != 1')

    def test_invalid_region(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)
        data = self.read_n(filter, 100)            

        expected = (1306, False)
        self.assertEquals(filter.get_servo_position(self.servo_id), expected, 'servo_pos != {}'.format(expected))

    def test_angular_position(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)
        data = self.read_n(filter, 200)

        count = filter.get_count(self.servo_id)
        expected = 2 * math.pi * count / 1500.0 
        self.assertEquals(filter.get_angular_position(self.servo_id), expected, 'angular_position != {}'.format(expected))

    def test_duty(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)
        data = self.read_n(filter, 100)

        expected = data[-1][1]
        self.assertEquals(filter.get_duty(self.servo_id), expected, 'duty != {}'.format(expected))

    # Disable this test as it requires a ROS node to be available...
    def notest_reset(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)
        data = self.read_n(filter, 300)
        filter.reset(self.servo_id, 0)

        # Values should reset to zero
        self.assertEquals(filter.get_revolutions(self.servo_id), 0, 'revolutions != 0')
        self.assertEquals(filter.get_count(self.servo_id), 0, 'count != 0')
        self.assertEquals(filter.get_duty(self.servo_id), 0, 'duty != 0')
        self.assertEquals(filter.get_angular_position(self.servo_id), 0, 'angular_position != 0')

        # Values should reset to tuple (0, True)
        self.assertEquals(filter.get_servo_position(self.servo_id), (0, True), 'servo_pos != (0 ,True)')

        # Values should reset to one
        self.assertEquals(filter.get_invert(self.servo_id), 1, 'invert != 1')

    def test_invert(self):
        filter = LX16AEncoderFilterSklearn(
            self.classifier_filename, self.regressor_filename, self.window)
        filter.add(self.servo_id)
        data = self.read_n(filter, 200)
        filter.set_invert(self.servo_id, True)

        expected = -1
        self.assertEquals(filter.get_invert(self.servo_id), expected, 'invert != {}'.format(expected))

        count = filter.get_count(self.servo_id)
        expected = 2 * math.pi * count / 1500.0
        self.assertEquals(filter.get_angular_position(self.servo_id), expected, 'angular_position != {}'.format(expected))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_ext_encoder_filter', TestExtEncoderFilter)

