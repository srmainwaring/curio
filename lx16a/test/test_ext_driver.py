PKG='lx16a'

from lx16a.ext import LX16ADriver

import time
import unittest

class TestExtDriver(unittest.TestCase):
    ''' Test cases for extension LX16ADriver

    This test requires hardware to be available and will move servos.
    '''

    def setUp(self):
        # Constants
        self.driver = None
        self.servo_id = 11
        self.port = '/dev/cu.usbserial-142110'
        self.baudrate = 115200
        self.timeout = 1000          # [ms]
        self.response_timeout = 15   # [ms]

    def tearDown(self):
        # If the driver is open ensure the motor is shutdown 
        if (self.driver is not None) and (self.driver.is_open()):
            self.driver.set_motor_mode(self.servo_id, 0)
            self.driver.close()

    def test_has_method_attributes(self):
        self.driver = LX16ADriver()

        self.assertTrue(hasattr(self.driver, 'open'), "missing attr 'open'")
        self.assertTrue(hasattr(self.driver, 'close'), "missing attr 'close'")
        self.assertTrue(hasattr(self.driver, 'is_open'), "missing attr 'is_open'")
        self.assertTrue(hasattr(self.driver, 'get_port'), "missing attr 'get_port'")
        self.assertTrue(hasattr(self.driver, 'set_port'), "missing attr 'set_port'")
        self.assertTrue(hasattr(self.driver, 'get_baudrate'), "missing attr 'get_baudrate'")
        self.assertTrue(hasattr(self.driver, 'set_baudrate'), "missing attr 'set_baudrate'")
        # self.assertTrue(hasattr(self.driver, 'get_timeout'), "missing attr 'get_timeout'")
        self.assertTrue(hasattr(self.driver, 'set_timeout'), "missing attr 'set_timeout'")
        self.assertTrue(hasattr(self.driver, 'get_response_timeout'), "missing attr 'get_response_timeout'")
        self.assertTrue(hasattr(self.driver, 'set_response_timeout'), "missing attr 'set_response_timeout'")
        self.assertTrue(hasattr(self.driver, 'move'), "missing attr 'move'")
        self.assertTrue(hasattr(self.driver, 'get_move'), "missing attr 'get_move'")
        self.assertTrue(hasattr(self.driver, 'set_prepared_move'), "missing attr 'set_prepared_move'")
        self.assertTrue(hasattr(self.driver, 'get_prepared_move'), "missing attr 'get_prepared_move'")
        self.assertTrue(hasattr(self.driver, 'move_start'), "missing attr 'move_start'")
        self.assertTrue(hasattr(self.driver, 'move_stop'), "missing attr 'move_stop'")
        self.assertTrue(hasattr(self.driver, 'set_position_offset'), "missing attr 'set_position_offset'")
        self.assertTrue(hasattr(self.driver, 'save_position_offset'), "missing attr 'save_position_offset'")
        self.assertTrue(hasattr(self.driver, 'get_position_offset'), "missing attr 'get_position_offset'")
        self.assertTrue(hasattr(self.driver, 'get_position_limits'), "missing attr 'get_position_limits'")
        self.assertTrue(hasattr(self.driver, 'set_position_limits'), "missing attr 'set_position_limits'")
        self.assertTrue(hasattr(self.driver, 'get_voltage_limits'), "missing attr 'get_voltage_limits'")
        self.assertTrue(hasattr(self.driver, 'set_voltage_limits'), "missing attr 'set_voltage_limits'")
        self.assertTrue(hasattr(self.driver, 'set_max_temperature_limit'), "missing attr 'set_max_temperature_limit'")
        self.assertTrue(hasattr(self.driver, 'get_max_temperature_limit'), "missing attr 'get_max_temperature_limit'")
        self.assertTrue(hasattr(self.driver, 'get_temperature'), "missing attr 'get_temperature'")
        self.assertTrue(hasattr(self.driver, 'get_voltage'), "missing attr 'get_voltage'")
        self.assertTrue(hasattr(self.driver, 'get_position'), "missing attr 'get_position'")
        self.assertTrue(hasattr(self.driver, 'set_motor_mode'), "missing attr 'set_motor_mode'")
        self.assertTrue(hasattr(self.driver, 'set_servo_mode'), "missing attr 'set_servo_mode'")
        self.assertTrue(hasattr(self.driver, 'get_mode'), "missing attr 'get_mode'")
        self.assertTrue(hasattr(self.driver, 'set_motor_on'), "missing attr 'set_motor_on'")
        self.assertTrue(hasattr(self.driver, 'set_motor_off'), "missing attr 'set_motor_off'")
        self.assertTrue(hasattr(self.driver, 'is_motor_on'), "missing attr 'is_motor_on'")
        self.assertTrue(hasattr(self.driver, 'set_led_on'), "missing attr 'set_led_on'")
        self.assertTrue(hasattr(self.driver, 'set_led_off'), "missing attr 'set_led_off'")
        self.assertTrue(hasattr(self.driver, 'is_led_on'), "missing attr 'is_led_on'")
        self.assertTrue(hasattr(self.driver, 'set_led_errors'), "missing attr 'set_led_errors'")
        self.assertTrue(hasattr(self.driver, 'get_led_errors'), "missing attr 'get_led_errors'")

    def test_constructor(self):
        self.driver = LX16ADriver()

        self.assertIsNotNone(self.driver, "driver is not None")
        self.assertEqual(self.driver.get_port(), '', "port is not ''")
        self.assertEqual(self.driver.get_baudrate(), 9600, "baudrate is not '9600'")
        self.assertFalse(self.driver.is_open(), "driver is not closed")

    def test_open(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()

        self.assertTrue(self.driver.is_open(), "port is not open")
        self.assertEqual(self.driver.get_port(), self.port, "port is not ' " + self.port + "'")
        self.assertEqual(self.driver.get_baudrate(), self.baudrate, "baudrate is not ' " + str(self.baudrate) + "'")
        self.assertEqual(self.driver.get_response_timeout(), self.response_timeout, "response timeout is not ' " + str(self.response_timeout) + "'")

    def test_move(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        self.assertTrue(self.driver.is_open(), "port is not open")

        self.driver.set_servo_mode(self.servo_id)
        self.driver.move(self.servo_id, 0)
        time.sleep(1.0)
        pos = self.driver.get_position(self.servo_id)
        self.assertGreaterEqual(pos, -30, "position ! >= -30")
        self.assertLessEqual(pos, 30, "position ! <= 30")

        self.driver.move(self.servo_id, 500)
        time.sleep(0.5)  
        pos = self.driver.get_position(self.servo_id)
        self.assertGreaterEqual(pos, 470, "position ! >= 470")
        self.assertLessEqual(pos, 530, "position ! <= 530")

    def test_position_offset(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        self.assertTrue(self.driver.is_open(), "port is not open")

        self.driver.set_position_offset(self.servo_id, 50)
        offset = self.driver.get_position_offset(self.servo_id)
        self.assertEqual(offset, 50, "offset != 50")

        self.driver.save_position_offset(self.servo_id)
        time.sleep(0.2)

        offset = self.driver.get_position_offset(self.servo_id)
        self.assertEqual(offset, 50, "offset != 50")

        self.driver.set_position_offset(self.servo_id, 0)
        offset = self.driver.get_position_offset(self.servo_id)
        self.assertEqual(offset, 0, "offset != 0")

        self.driver.save_position_offset(self.servo_id)
        time.sleep(0.2)

        offset = self.driver.get_position_offset(self.servo_id)
        self.assertEqual(offset, 0, "offset != 0")

    def test_voltage_limits(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        min_voltage = 4.5
        max_voltage = 12.0
        self.driver.set_voltage_limits(self.servo_id, min_voltage, max_voltage)
        time.sleep(0.2)

        min_voltage, max_voltage = self.driver.get_voltage_limits(self.servo_id)
        self.assertEqual(min_voltage, 4.5, "min_voltage ! = 4.5")
        self.assertEqual(max_voltage, 12.0, "max_voltage ! = 12.0")

    def test_voltage(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        min_voltage, max_voltage = self.driver.get_voltage_limits(self.servo_id)
        voltage = self.driver.get_voltage(self.servo_id)
        self.assertGreaterEqual(voltage, min_voltage, "voltage ! >= " + str(min_voltage))
        self.assertLessEqual(voltage, max_voltage, "voltage ! <= " + str(max_voltage))

    # TEST_FAIL:
    def notest_temperature_limits(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        max_temp = 90
        self.driver.set_max_temperature_limit(self.servo_id, max_temp)
        time.sleep(0.2)

        max_temp = self.driver.get_max_temperature_limit(self.servo_id)
        self.assertEqual(max_temp, 90.0, "max_temperature ! = 90.0")

    def test_temperature(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        max_temp = self.driver.get_max_temperature_limit(self.servo_id)
        temp = self.driver.get_temperature(self.servo_id)
        self.assertGreaterEqual(temp, 0, "temp ! >= 0")
        self.assertLessEqual(temp, max_temp, "temp ! <= " + str(max_temp))

    def test_motor_mode(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        duty_sp = 500
        self.driver.set_motor_mode(self.servo_id, duty_sp)
        time.sleep(0.5)

        mode, duty = self.driver.get_mode(self.servo_id)
        self.assertEqual(duty, duty_sp, "duty != " + str(duty_sp))
        self.assertEqual(mode, 1, "mode != 1")

        self.driver.set_motor_mode(self.servo_id, 0)

    def test_servo_mode(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        self.driver.set_servo_mode(self.servo_id)
        time.sleep(0.5)

        mode, duty = self.driver.get_mode(self.servo_id)
        self.assertEqual(duty, 0, "duty != 0")
        self.assertEqual(mode, 0, "mode != 0")

    # TEST_FAIL:
    def notest_motor_on_off(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        self.driver.set_motor_off(self.servo_id)
        time.sleep(0.5)

        is_motor_on = self.driver.is_motor_on(self.servo_id)
        self.assertFalse(is_motor_on, "motor is not off")

        self.driver.set_motor_on(self.servo_id)
        time.sleep(0.5)

        is_motor_on = self.driver.is_motor_on(self.servo_id)
        self.assertTrue(is_motor_on, "motor is not on")

    def test_led_on_off(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        self.driver.set_led_off(self.servo_id)
        time.sleep(0.5)

        is_led_on = self.driver.is_led_on(self.servo_id)
        self.assertFalse(is_led_on, "led is not off")

        self.driver.set_led_on(self.servo_id)
        time.sleep(0.5)

        is_led_on = self.driver.is_led_on(self.servo_id)
        self.assertTrue(is_led_on, "led is not on")

    # TEST_INCOMPLETE:
    def test_led_errors(self):
        self.driver = LX16ADriver()
        self.driver.set_port(self.port)
        self.driver.set_baudrate(self.baudrate)
        self.driver.set_timeout(self.timeout)
        self.driver.set_response_timeout(self.response_timeout)
        self.driver.open()
 
        fault_codes = self.driver.get_led_errors(self.servo_id)

        # has_temp_alarm = fault_codes & 1
        # has_volt_alarm = fault_codes & 2
        # has_lock_alarm = fault_codes & 4

        # self.assertTrue(has_temp_alarm, "has temp alarm is not true")
        # self.assertTrue(has_volt_alarm, "has volt alarm is not true")
        # self.assertTrue(has_lock_alarm, "has lock alarm is not true")

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_ext_driver', TestExtDriver)

