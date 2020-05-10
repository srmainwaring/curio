#!/usr/bin/env python

PKG='lx16a'

import os
import sys
import unittest
from lx16a.lx16a_encoder_filter import LX16AEncoderFilter

# Constants
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

class TestEncoderFilter(unittest.TestCase):
    ''' Test cases for lx16a.LX16AEncoderFilter
    '''

    def test_constructor(self):
        # Load the model files
        encoder_filter = LX16AEncoderFilter(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)

        # Values should initialise to zero
        self.assertEquals(encoder_filter.get_revolutions(), 0, 'get_revolutions')
        self.assertEquals(encoder_filter.get_count(), 0, 'get_count')
        self.assertEquals(encoder_filter.get_duty(), 0, 'get_duty')
        self.assertEquals(encoder_filter.get_angular_position(), 0, 'get_angular_position')

        # Values should initialise to tuple (0, True)
        self.assertEquals(encoder_filter.get_servo_pos(), (0, True), 'get_servo_pos')

        # Values should initialise to one
        self.assertEquals(encoder_filter.get_invert(), 1, 'get_invert')

    def test_valid_region(self):
        # Load the model files
        encoder_filter = LX16AEncoderFilter(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)

        # Values should initialise to zero
        self.assertEquals(encoder_filter.get_revolutions(), 0, 'get_revolutions')
        self.assertEquals(encoder_filter.get_count(), 0, 'get_count')
        self.assertEquals(encoder_filter.get_duty(), 0, 'get_duty')
        self.assertEquals(encoder_filter.get_angular_position(), 0, 'get_angular_position')

        # Values should initialise to tuple (0, True)
        self.assertEquals(encoder_filter.get_servo_pos(), (0, True), 'get_servo_pos')

        # Values should initialise to one
        self.assertEquals(encoder_filter.get_invert(), 1, 'get_invert')

    def test_invalid_region(self):
        # Load the model files
        encoder_filter = LX16AEncoderFilter(
            CLASSIFIER_FILENAME, REGRESSOR_FILENAME, WINDOW)

        # Values should initialise to zero
        self.assertEquals(encoder_filter.get_revolutions(), 0, 'get_revolutions')
        self.assertEquals(encoder_filter.get_count(), 0, 'get_count')
        self.assertEquals(encoder_filter.get_duty(), 0, 'get_duty')
        self.assertEquals(encoder_filter.get_angular_position(), 0, 'get_angular_position')

        # Values should initialise to tuple (0, True)
        self.assertEquals(encoder_filter.get_servo_pos(), (0, True), 'get_servo_pos')

        # Values should initialise to one
        self.assertEquals(encoder_filter.get_invert(), 1, 'get_invert')


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_encoder_filter', TestEncoderFilter)

