PKG='lx16a'

from lx16a.ext import Time

import unittest
import rospy

class TestExtTime(unittest.TestCase):
    ''' Test cases for C++ extension lx16a.ext.Time
    '''

    def test_constructor_default(self):
        time = Time()
        self.assertEquals(time.sec, 0, 'sec != 0')
        self.assertEquals(time.nsec, 0, 'nsec != 0')

    def test_constructor(self):
        time = Time(1, 4560)
        self.assertEquals(time.sec, 1, 'sec != 1')
        self.assertEquals(time.nsec, 4560, 'nsec != 4560')

        time = Time(1, int(4560.0))
        self.assertEquals(time.sec, 1, 'sec != 1')
        self.assertEquals(time.nsec, 4560, 'nsec != 4560')

    def test_constructor_double(self):
        nsec = 1578874221323091983
        secs = nsec * 1E-9
        time = Time(secs)
        self.assertEquals(time.sec, 1578874221, 'sec != 1578874221')
        self.assertEquals(time.nsec, 323091984, 'nsec != 323091984')

    def test_constructor_error(self):
        with self.assertRaises(TypeError):
            time = Time(1, 4560.0)

        with self.assertRaises(TypeError):
            time = Time(1.0, 4560)

        with self.assertRaises(TypeError):
            time = Time(1.0, 4560.0)

if __name__ == '__main__':
    import rosunit
    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rosunit.unitrun(PKG, 'test_ext_time', TestExtTime)

