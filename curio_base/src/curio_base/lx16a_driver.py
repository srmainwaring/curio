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

''' Lewansoul LX-16A servo driver.

    Serial driver for the Lewansoul LX-16A servo bus board.

    The LX16ADriver does not explicitly raise exceptions, but may raise
    exceptions implicitly via the modules it uses (i.e. serial).
    Instead warnings and errors are logged to ROS and an error state -1
    is returned for the caller to test and handle or ignore
    as they wish.    

    The protocol specifications may be found here:
        http://www.lewansoul.com/product/detail-17.html
        http://www.lewansoul.com/uploads/attachment/20171204/93cd398b7aa583824a0acd5a23fb8ebb.zip


    Acknowledgments:

    Serial communication python code adapted from Roger Chen's lewansoul_wrapper.py
    https://github.com/Roger-random/SGVHAK_Rover/blob/master/SGVHAK_Rover/lewansoul_wrapper.py


    MIT License

    Copyright (c) 2018 Roger Cheng

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation files
    (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so,
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
    BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
    ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
    CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
'''

import rospy
import serial
import struct

class LX16ADriver(object):
    ''' LewanSoul Bus Servo Communication Protocol

    A hardware interface to the LewanSoul LX-16A servo.
    This driver is a fairly literal implementation of the commands
    specified in the LewanSoul Bus Servo Communication Protocol.
    '''

    SERVO_BUS_HEADER            = 0x55
    SERVO_BUS_MIN_ID            = 0x00
    SERVO_BUS_MAX_ID            = 0xfe
    SERVO_BUS_BROADCAST_ID      = 0xfe

    SERVO_MOVE_TIME_WRITE       = 1
    SERVO_MOVE_TIME_READ        = 2
    SERVO_MOVE_TIME_WAIT_WRITE  = 7
    SERVO_MOVE_TIME_WAIT_READ   = 8
    SERVO_MOVE_START            = 11
    SERVO_MOVE_STOP             = 12
    SERVO_ID_WRITE              = 13
    SERVO_ID_READ               = 14
    SERVO_ANGLE_OFFSET_ADJUST   = 17
    SERVO_ANGLE_OFFSET_WRITE    = 18
    SERVO_ANGLE_OFFSET_READ     = 19
    SERVO_ANGLE_LIMIT_WRITE     = 20
    SERVO_ANGLE_LIMIT_READ      = 21
    SERVO_VIN_LIMIT_WRITE       = 22
    SERVO_VIN_LIMIT_READ        = 23
    SERVO_TEMP_MAX_LIMIT_WRITE  = 24
    SERVO_TEMP_MAX_LIMIT_READ   = 25
    SERVO_TEMP_READ             = 26
    SERVO_VIN_READ              = 27
    SERVO_POS_READ              = 28
    SERVO_OR_MOTOR_MODE_WRITE   = 29
    SERVO_OR_MOTOR_MODE_READ    = 30
    SERVO_LOAD_OR_UNLOAD_WRITE  = 31
    SERVO_LOAD_OR_UNLOAD_READ   = 31
    SERVO_LED_CTRL_WRITE        = 33
    SERVO_LED_CTRL_READ         = 34
    SERVO_LED_ERROR_WRITE       = 35
    SERVO_LED_ERROR_READ        = 36

    ERROR_VALUE_POS             = -2048
    ERROR_VALUE_VIN             = -2048
    ERROR_VALUE_TEMP            = -2048


    def __init__(self):
        ''' Constructor
        '''

        self._serial = serial.Serial()
    
    # Serial port 
    def open(self):
        ''' Open the serial port for communication.
        '''

        self._serial.open()

    def close(self):
        ''' Close the serial port immediatelty.
        '''

        if self._serial.is_open:
            self._serial.close()

    def is_open(self):
        ''' Return True if the serial port is open.

        Returns
        -------
        bool
            True if the port is open
        '''

        return self._serial.is_open

    def get_port(self):
        ''' Get the serial device.

        Returns
        -------
        string
            The name of the serial device.
        '''
        
        return self._serial.port

    def set_port(self, port):
        ''' Set the serial device.

        Parameters
        ----------
        port : str
            The name of the serial device.
        '''

        self._serial.port = port

    def get_baudrate(self):
        ''' Get the baudrate.

        Returns
        -------
        int
            The baudrate for the serial connection.
        '''

        return self._serial.baudrate

    def set_baudrate(self, baudrate):
        ''' Set the baudrate.

        Parameters
        ----------
        baudrate : int
            The baudrate for the serial connection.
        '''

        self._serial.baudrate = baudrate

    def get_timeout(self):
        ''' Get the serial timeout [s]

        Returns
        -------
        float
            The serial port timeout.
        '''

        return self._serial.timeout

    def set_timeout(self, timeout):
        ''' Set the serial timeout [s]

        Parameters
        -------
        timeout : float
            The serial port timeout in seconds.
        '''

        self._serial.timeout = timeout

    # Commands
    def move_time_write(self, servo_id, servo_pos, move_time=0):
        ''' Move a servo to the commanded position in a given time.

        Parameters
        ----------
        servo_id : int
            The servo serial identifier, in [0, 253]
        servo_pos : int
            The commanded servo position, in [0, 1000]
        move_time : int
            The desired time for the servo to move from its
            position to the commanded position, in [0, 30000] ms
        '''

        pos_lsb = servo_pos & 0xff
        pos_hsb = (servo_pos >> 8) & 0xff
        move_time_lsb = move_time & 0xff
        move_time_hsb = (move_time >> 8) & 0xff
        if self.send_command(servo_id, 7, LX16ADriver.SERVO_MOVE_TIME_WRITE,
            (pos_lsb, pos_hsb, move_time_lsb, move_time_hsb)) == -1:
            rospy.logwarn('Servo command error: move_time_write')

    def move_time_read(self, servo_id):
        ''' Read the current commanded position and move time.

        Parameters
        ----------
        servo_id : int
            The servo serial identifier, in [0, 253]

        Returns:
        list
            A two element list containing:
            servo_pos : int
                The commanded servo position, in [0, 1000]
            move_time : int
                The desired time for the servo to move from its
                position to the commanded position, in [0, 30000] ms
        '''

        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_MOVE_TIME_READ) == -1:
            rospy.logwarn('Servo command error: move_time_read')
            return -1, -1
        data = self.read_response(servo_id, 7, LX16ADriver.SERVO_MOVE_TIME_READ)
        if data == -1:
            rospy.logwarn('Servo read error: move_time_read')
            return -1, -1
        pos  = data[0] + (data[1] << 8)
        move_time = data[2] + (data[3] << 8)
        return pos, move_time

    def move_time_wait_write(self, servo_id, servo_pos, move_time):
        pos_lsb = servo_pos & 0xff
        pos_hsb = (servo_pos >> 8) & 0xff
        move_time_lsb = move_time & 0xff
        move_time_hsb = (move_time >> 8) & 0xff
        if self.send_command(servo_id, 7, LX16ADriver.SERVO_MOVE_TIME_WAIT_WRITE,
            (pos_lsb, pos_hsb, move_time_lsb, move_time_hsb)) == -1:
            rospy.logwarn('Servo command error: move_time_wait_write')

    # @TOOD: there appears to be a problem with this command
    #        - the response from the servo is badly formed (and it also
    #        affects the execution of the write version of this command)
    def move_time_wait_read(self, servo_id):
        # @TODO investigate issue further
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_MOVE_TIME_WAIT_READ) == -1:
            rospy.logwarn('Servo command error: move_time_wait_read')
            return -1, -1
        data = self.read_response(servo_id, 7, LX16ADriver.SERVO_MOVE_TIME_WAIT_READ)
        if data == -1:
            rospy.logwarn('Servo read error: move_time_wait_read')
            return -1, -1
        pos  = data[0] + (data[1] << 8)
        move_time = data[2] + (data[3] << 8)
        return pos, move_time

    def move_start(self, servo_id):
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_MOVE_START) == -1:
            rospy.logwarn('Servo command error: move_start')

    # @NOTE: The servo must be in servo mode for the move_stop to be
    #        effective. When in motor_mode you must set the speed to
    #        zero instead.
    def move_stop(self, servo_id):
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_MOVE_STOP) == -1:
            rospy.logwarn('Servo command error: move_stop')

    def id_write(self, servo_id):
        # @TODO implement
        pass

    def id_read(self, servo_id):
        # @TODO implement
        pass

    def angle_offset_adjust(self, servo_id, deviation):
        deviation_lsb = deviation & 0xff 
        if self.send_command(servo_id, 4, LX16ADriver.SERVO_ANGLE_OFFSET_ADJUST,
            (deviation_lsb,)) == -1:
            rospy.logwarn('Servo command error: angle_offset_adjust')

    def angle_offset_write(self, servo_id):
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_ANGLE_OFFSET_WRITE) == -1:
            rospy.logwarn('Servo command error: angle_offset_write')

    def angle_offset_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_ANGLE_OFFSET_READ) == -1:
            rospy.logwarn('Servo command error: angle_offset_read')
            return -1
        data = self.read_response(servo_id, 4, LX16ADriver.SERVO_ANGLE_OFFSET_READ)
        if data == -1:
            rospy.logwarn('Servo read error: angle_offset_read')
            return -1
        angle_offset = struct.unpack('b', data)[0]
        return angle_offset

    def angle_limit_write(self, servo_id, min_angle, max_angle):
        min_angle_lsb = min_angle & 0xff
        min_angle_hsb = (min_angle >> 8) & 0xff
        max_angle_lsb = max_angle & 0xff
        max_angle_hsb = (max_angle >> 8) & 0xff
        if self.send_command(servo_id, 7, LX16ADriver.SERVO_ANGLE_LIMIT_WRITE,
            (min_angle_lsb, min_angle_hsb, max_angle_lsb, max_angle_hsb)) == -1:
            rospy.logwarn('Servo command error: angle_limit_write')

    def angle_limit_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_ANGLE_LIMIT_READ) == -1:
            rospy.logwarn('Servo command error: angle_limit_read')
            return -1, -1
        data = self.read_response(servo_id, 7, LX16ADriver.SERVO_ANGLE_LIMIT_READ)
        if data == -1:
            rospy.logwarn('Servo read error: angle_limit_read')
            return -1, -1
        min_angle = data[0] + (data[1] << 8)
        max_angle = data[2] + (data[3] << 8)
        return min_angle, max_angle 

    def vin_limit_write(self, servo_id, min_vin, max_vin):
        min_vin = int(min_vin * 1000)
        max_vin = int(max_vin * 1000)
        min_vin_lsb = min_vin & 0xff
        min_vin_hsb = (min_vin >> 8) & 0xff
        max_vin_lsb = max_vin & 0xff
        max_vin_hsb = (max_vin >> 8) & 0xff
        if self.send_command(servo_id, 7, LX16ADriver.SERVO_VIN_LIMIT_WRITE,
            (min_vin_lsb, min_vin_hsb, max_vin_lsb, max_vin_hsb)) == -1:
            rospy.logwarn('Servo command error: vin_limit_write')

    def vin_limit_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_VIN_LIMIT_READ) == -1:
            rospy.logwarn('Servo command error: vin_limit_read')
            return -1, -1
        data = self.read_response(servo_id, 7, LX16ADriver.SERVO_VIN_LIMIT_READ)
        if data == -1:
            rospy.logwarn('Servo read error: vin_limit_read')
            return -1, -1
        min_vin = data[0] + (data[1] << 8)
        max_vin = data[2] + (data[3] << 8)
        min_vin = min_vin / 1000.0
        max_vin = max_vin / 1000.0
        return min_vin, max_vin 

    def temp_max_limit_write(self, servo_id, max_temp):
        max_temp_lsb = max_temp & 0xff
        if self.send_command(servo_id, 4, LX16ADriver.SERVO_TEMP_MAX_LIMIT_WRITE,
            (max_temp_lsb,)) == -1:
            rospy.logwarn('Servo command error: temp_max_limit_write')

    def temp_max_limit_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_TEMP_MAX_LIMIT_READ) == -1:
            rospy.logwarn('Servo command error: temp_max_limit_read')
            return -1
        data = self.read_response(servo_id, 4, LX16ADriver.SERVO_TEMP_MAX_LIMIT_READ)
        if data == -1:
            rospy.logwarn('Servo read error: temp_max_limit_read')
            return -1
        temp_max_limit = data[0]
        return temp_max_limit  

    def temp_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_TEMP_READ) == -1:
            rospy.logwarn('Servo command error: temp_read')
            return -1
        data = self.read_response(servo_id, 4, LX16ADriver.SERVO_TEMP_READ)
        if data == -1:
            rospy.logwarn('Servo read error: temp_read')
            return -1
        temp = data[0]
        return temp  

    def vin_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_VIN_READ) == -1:
            rospy.logwarn('Servo command error: vin_read')
            return -1

        # @TEENSY_TEST
        # return 0

        data = self.read_response(servo_id, 5, LX16ADriver.SERVO_VIN_READ)
        if data == -1:
            rospy.logwarn('Servo read error: vin_read')
            return -1
        vin = data[0] + (data[1] << 8)
        vin = vin / 1000.0
        return vin

    def pos_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_POS_READ) == -1:
            rospy.logwarn('Servo command error: pos_read')
            return LX16ADriver.ERROR_VALUE_POS
        data = self.read_response(servo_id, 5, LX16ADriver.SERVO_POS_READ)
        if data == -1:
            rospy.logwarn('Servo read error: pos_read')
            return LX16ADriver.ERROR_VALUE_POS
        pos = struct.unpack('h', data)[0]
        return pos

    def motor_mode_write(self, servo_id, duty):
        duty_lsb = duty & 0xff 
        duty_hsb = (duty >> 8) & 0xff 
        if self.send_command(servo_id, 7, LX16ADriver.SERVO_OR_MOTOR_MODE_WRITE,
            (1, 0, duty_lsb, duty_hsb)) == -1:
            rospy.logwarn('Servo command error: motor_mode_write')

    def servo_mode_write(self, servo_id):
        if self.send_command(servo_id, 7, LX16ADriver.SERVO_OR_MOTOR_MODE_WRITE,
            (0, 0, 0, 0)) == -1:
            rospy.logwarn('Servo command error: servo_mode_write')

    def mode_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_OR_MOTOR_MODE_READ) == -1:
            rospy.logwarn('Servo command error: mode_read')
            return -1
        data = self.read_response(servo_id, 7, LX16ADriver.SERVO_OR_MOTOR_MODE_READ)
        if data == -1:
            rospy.logwarn('Servo read error: mode_read')
            return -1
        mode = data[0]
        duty = struct.unpack('h', data[2:4])[0]
        return mode, duty

    def load_or_unload_write(self, servo_id, is_loaded):
        is_loaded_lsb = is_loaded & 0xff 
        if self.send_command(servo_id, 4, LX16ADriver.SERVO_LOAD_OR_UNLOAD_WRITE,
            (is_loaded_lsb,)) == -1:
            rospy.logwarn('Servo command error: load_or_unload_write')

    # @TODO: there appears to be a problem with this command
    #        - there is no response to the read request command. 
    def load_or_unload_read(self, servo_id):
        # @TODO investigate issue further
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_LOAD_OR_UNLOAD_READ) == -1:
            rospy.logwarn('Servo command error: load_or_unload_read')
            return -1
        data = self.read_response(servo_id, 4, LX16ADriver.SERVO_LOAD_OR_UNLOAD_READ)
        if data == -1:
            rospy.logwarn('Servo read error: load_or_unload_read')
            return -1
        load_or_unload = data[0]
        return load_or_unload  

    def led_ctrl_write(self, servo_id, is_light_off):
        is_light_off_lsb = is_light_off & 0xff 
        if self.send_command(servo_id, 4, LX16ADriver.SERVO_LED_CTRL_WRITE,
            (is_light_off_lsb,)) == -1:
            rospy.logwarn('Servo command error: led_ctrl_write')

    def led_ctrl_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_LED_CTRL_READ) == -1:
            rospy.logwarn('Servo command error: led_ctrl_read')
            return -1
        data = self.read_response(servo_id, 4, LX16ADriver.SERVO_LED_CTRL_READ)
        if data == -1:
            rospy.logwarn('Servo read error: led_ctrl_read')
            return -1
        is_light_off = data[0]
        return is_light_off  

    def led_error_write(self, servo_id, fault_code):
        fault_code_lsb = fault_code & 0xff 
        if self.send_command(servo_id, 4, LX16ADriver.SERVO_LED_ERROR_WRITE,
            (fault_code_lsb,)) == -1:
            rospy.logwarn('Servo command error: led_error_write')

    def led_error_read(self, servo_id):
        self._serial.reset_input_buffer()
        if self.send_command(servo_id, 3, LX16ADriver.SERVO_LED_ERROR_READ) == -1:
            rospy.logwarn('Servo command error: led_error_read')
            return -1
        data = self.read_response(servo_id, 4, LX16ADriver.SERVO_LED_ERROR_READ)
        if data == -1:
            rospy.logwarn('Servo read error: led_error_read')
            return -1
        fault_code = data[0]
        return fault_code  

    # Serial communication protocol
    def checksum(self, servo_id, length, command, data):
        checksum = servo_id + length + command
        if data:
            for d in data:
                checksum = checksum + d
        checksum = (~checksum) & 0xff
        return checksum

    def read_byte(self):
        bytes_read = self._serial.read()
        if len(bytes_read) == 0:
            # rospy.logerr('Serial read error, expecting 1 byte: got none')
            return -1
        else:
            return ord(bytes_read[0])

    def read_bytearray(self, length):
        return bytearray(self._serial.read(length))

    def read_response(self, servo_id, length, command):
        # State machine states
        READ_WAIT_FOR_RESPONSE = 0
        READ_HEADER_1 = 1
        READ_HEADER_2 = 2
        READ_SERVO_ID = 3
        READ_LENGTH = 4
        READ_COMMAND = 5
        READ_DATA = 6
        READ_CHECKSUM = 7
        READ_COMPLETE = 8
        READ_ERROR = 9
        READ_TIMED_OUT = 10

        # Check port is open
        if not self.is_open():
            rospy.logwarn("Serial port not open")
            return -1

        # Timeout
        READ_TIMEOUT = 0.01 # [s]
        read_timeout = rospy.Duration.from_sec(READ_TIMEOUT)
        start = rospy.Time.now()

        state = READ_WAIT_FOR_RESPONSE
        error_msg = 'No response'
        data = []
        while True:
            # Check for timeout
            if rospy.Time.now() - start > read_timeout:
               state = READ_TIMED_OUT               

            # State machine
            if state == READ_WAIT_FOR_RESPONSE:
                if self._serial.in_waiting > 0:
                    state = READ_HEADER_1
            elif state == READ_HEADER_1:
                byte = self.read_byte()
                if byte == LX16ADriver.SERVO_BUS_HEADER:
                    state = READ_HEADER_2
                else:
                    state = READ_HEADER_1
                    error_msg = 'Invalid 1st header byte: expecting: {}, got: {}'.format(LX16ADriver.SERVO_BUS_HEADER, byte)
            elif state == READ_HEADER_2:
                byte = self.read_byte()
                if byte == LX16ADriver.SERVO_BUS_HEADER:
                    state = READ_SERVO_ID
                else:
                    state = READ_HEADER_1
                    error_msg = 'Invalid 2nd header byte: expecting: {}, got: {}'.format(LX16ADriver.SERVO_BUS_HEADER, byte)
            elif state == READ_SERVO_ID:
                byte = self.read_byte()
                if byte == servo_id:
                    state = READ_LENGTH
                else:
                    state = READ_HEADER_1
                    error_msg = 'Invalid servo_id: expecting: {}, got: {}'.format(servo_id, byte)
            elif state == READ_LENGTH:
                byte = self.read_byte()
                if byte == length:
                    state = READ_COMMAND
                else:
                    state = READ_HEADER_1
                    error_msg = 'Invalid length: expecting: {}, got: {}'.format(length, byte)
            elif state == READ_COMMAND:
                byte = self.read_byte()
                if byte == command:
                    state = READ_DATA
                else:
                    state = READ_HEADER_1
                    error_msg = 'Invalid command: expecting: {}, got: {}'.format(command, byte)
            elif state == READ_DATA:
                data = self.read_bytearray(length - 3)
                if len(data) == length - 3:
                    state = READ_CHECKSUM
                else:
                    state = READ_ERROR
                    error_msg = 'Invalid data size: expecting: {}, got: {}'.format(length - 3, len(data))
            elif state == READ_CHECKSUM:
                checksum = self.checksum(servo_id, length, command, data)
                byte = self.read_byte()
                if byte == checksum:
                    state = READ_COMPLETE
                else:
                    state = READ_ERROR
                    error_msg = 'Invalid checksum: expecting: {}, got: {}'.format(checksum, byte)
            elif state == READ_COMPLETE:
                # rospy.loginfo('OK: id: {}, cmd: {}'.format(servo_id, command))
                return data
            elif state == READ_ERROR:
                rospy.logwarn('ERROR: id: {}, cmd: {}, msg: {}'.format(servo_id, command, error_msg))
                return -1
            elif state == READ_TIMED_OUT:
                rospy.logwarn('TIMED_OUT: id: {}, cmd: {}, msg: {}'.format(servo_id, command, error_msg))
                return -1

    def send_command(self, servo_id, length, command, data=None):
        # Check the port is open
        if not self.is_open():
            return -1

        # Packet header [0x55, 0x55]
        packet = [LX16ADriver.SERVO_BUS_HEADER, LX16ADriver.SERVO_BUS_HEADER]

        # Check the servo id is in range
        if servo_id < LX16ADriver.SERVO_BUS_MIN_ID or servo_id > LX16ADriver.SERVO_BUS_MAX_ID:
            return -1
        packet.append(servo_id)

        # Check the data is consistent with the specfied packet length
        if length != 3 + (len(data) if data else 0): 
            return -1
        packet.append(length)

        # Append command to packet
        packet.append(command)

        # Append parameter data to packet
        if data:
            for d in data:
                packet.append(d)

        # Calculate checksum and append to packet
        checksum = self.checksum(servo_id, length, command, data)
        packet.append(checksum)

        # Send command
        packet_bytes = bytearray(packet)
        bytes_sent = self._serial.write(packet_bytes)

        # DEBUG INFO
        # rospy.loginfo("servo_id: {}, commmand: {}, data: {}".format(servo_id, command, data))
        # rospy.loginfo("checksum: {}".format(checksum))
        # rospy.loginfo("packet: {}".format(packet))
        # rospy.loginfo("bytes_sent: {}".format(bytes_sent))
        # rospy.loginfo("out_waiting: {} bytes".format(self._serial.out_waiting))

        return bytes_sent