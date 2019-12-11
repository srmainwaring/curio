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

''' Lewansoul LX-16A servo driver.

    Based on Roger Chens lewansould_wrapper.py
    https://github.com/Roger-random/SGVHAK_Rover/blob/master/SGVHAK_Rover/lewansoul_wrapper.py

'''

import serial
import struct

class LewansoulDriver(object):
    '''
    LewanSoul Bus Servo Communication Protocol
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

    def __init__(self):
        pass
        self._serial = serial.Serial()
    
    # Serial port 
    def open(self):
        self._serial.open()

    def close(self):
        if self._serial.is_open:
            self._serial.close()

    def is_open(self):
        return self._serial.is_open

    def get_port(self):
        return self._serial.port

    def set_port(self, port):
        self._serial.port = port

    def get_baudrate(self):
        return self._serial.baudrate

    def set_baudrate(self, baudrate):
        self._serial.baudrate = baudrate

    def get_timeout(self):
        return self._serial.timeout

    def set_timeout(self, timeout):
        self._serial.timeout = timeout

    # Commands 
    def move_time_write(self, servo_id):
        pass
    
    def move_time_read(self, servo_id):
        pass

    def move_time_wait_write(self, servo_id):
        pass

    def move_time_wait_read(self, servo_id):
        pass

    def move_start(self, servo_id):
        pass

    def move_stop(self, servo_id):
        pass

    def id_write(self, servo_id):
        pass

    def id_read(self, servo_id):
        pass

    def angle_offset_adjust(self, servo_id):
        pass

    def angle_offset_write(self, servo_id):
        pass

    def angle_offset_read(self, servo_id):
        self._serial.reset_input_buffer()
        self.send_command(servo_id, 3, LewansoulDriver.SERVO_ANGLE_OFFSET_READ)
        data = self.read_response(servo_id, 4, LewansoulDriver.SERVO_ANGLE_OFFSET_READ)
        if data == -1:
            return -1
        angle_offset = data[0]
        return angle_offset

    def angle_limit_write(self, servo_id):
        pass

    def angle_limit_read(self, servo_id):
        self._serial.reset_input_buffer()
        self.send_command(servo_id, 3, LewansoulDriver.SERVO_ANGLE_LIMIT_READ)
        data = self.read_response(servo_id, 7, LewansoulDriver.SERVO_ANGLE_LIMIT_READ)
        if data == -1:
            return -1, -1
        min_angle = data[0] + (data[1] << 8)
        max_angle = data[2] + (data[3] << 8)
        return min_angle, max_angle 

    def vin_limit_write(self, servo_id):
        pass

    def vin_limit_read(self, servo_id):
        self._serial.reset_input_buffer()
        self.send_command(servo_id, 3, LewansoulDriver.SERVO_VIN_LIMIT_READ)
        data = self.read_response(servo_id, 7, LewansoulDriver.SERVO_VIN_LIMIT_READ)
        if data == -1:
            return -1, -1
        min_vin = data[0] + (data[1] << 8)
        max_vin = data[2] + (data[3] << 8)
        return min_vin, max_vin 

    def temp_max_limit_write(self, servo_id):
        pass

    def temp_max_limit_read(self, servo_id):
        pass

    def temp_read(self, servo_id):
        pass

    def vin_read(self, servo_id):
        pass

    def pos_read(self, servo_id):
        self._serial.reset_input_buffer()
        self.send_command(servo_id, 3, LewansoulDriver.SERVO_POS_READ)
        data = self.read_response(servo_id, 5, LewansoulDriver.SERVO_POS_READ)
        if data == -1:
            return -1
        pos = data[0] + (data[1] << 8) 
        return pos

    def motor_mode_write(self, servo_id, speed):
        lsb = speed & 0xff 
        hsb = (speed >> 8) & 0xff 
        self.send_command(servo_id, 7, LewansoulDriver.SERVO_OR_MOTOR_MODE_WRITE, (1, 0, lsb, hsb))

    def servo_mode_write(self, servo_id):
        pass

    def mode_read(self, servo_id):
        pass

    def load_or_unload_write(self, servo_id):
        pass

    def load_or_unload_read(self, servo_id):
        pass

    def led_ctrl_write(self, servo_id):
        pass

    def led_ctrl_read(self, servo_id):
        pass

    def led_error_write(self, servo_id):
        pass

    def led_error_read(self, servo_id):
        pass

    # Serial communication protocol
    def checksum(self, servo_id, length, command, data):
        checksum = servo_id + length + command
        if data:
            for d in data:
                checksum = checksum + d
        checksum = (~checksum) & 0xff
        return checksum

    def read_byte(self):
        return ord(self._serial.read())

    def read_bytearray(self, length):
        return bytearray(self._serial.read(length))

    def read_response(self, servo_id, length, command):
        # Check port is open
        if not self.is_open():
            print("Serial port not open")
            return -1

        # Read header (2 bytes)
        byte = self.read_byte()
        if byte != LewansoulDriver.SERVO_BUS_HEADER:
            print("Invalid 1st header byte: expecting: {}, got: {}".format(LewansoulDriver.SERVO_BUS_HEADER, byte))
            return -1

        byte = self.read_byte()
        if byte != LewansoulDriver.SERVO_BUS_HEADER:
            print("Invalid 2nd header byte: expecting: {}, got: {}".format(LewansoulDriver.SERVO_BUS_HEADER, byte))
            return -1

        # Read id
        byte = self.read_byte()
        if byte != servo_id:
            print("Invalid servo_id: expecting: {}, got: {}".format(servo_id, byte))
            return -1

        # Read length
        byte = self.read_byte()
        if byte != length:
            print("Invalid length: expecting: {}, got: {}".format(length, byte))
            return -1

        # Read command
        byte = self.read_byte()
        if byte != command:
            print("Invalid command: expecting: {}, got: {}".format(command, byte))
            return -1

        # Read data. There should be length - 3 parameters in the data block
        data = self.read_bytearray(length - 3)
        if len(data) != length - 3:
            print("Invalid len(data): expecting: {}, got: {}".format(length - 3, len(data)))
            return -1

        # Calculate checksum
        checksum = self.checksum(servo_id, length, command, data)

        # Read checksum
        byte = self.read_byte()
        if byte != checksum:
            print("Invalid checksum: expecting: {}, got: ".format(checksum, byte))
            return -1

        # Read OK - return data (bytearray)
        return data

    def send_command(self, servo_id, length, command, data=None):
        # Check the port is open
        if not self.is_open():
            return -1

        # Packet header [0x55, 0x55]
        packet = [LewansoulDriver.SERVO_BUS_HEADER, LewansoulDriver.SERVO_BUS_HEADER]

        # Check the servo id is in range
        if servo_id < LewansoulDriver.SERVO_BUS_MIN_ID or servo_id > LewansoulDriver.SERVO_BUS_MAX_ID:
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
        num_bytes = self._serial.write(packet_bytes)

        # DEBUG INFO
        # print("servo_id: {}, commmand: {}, data: {}".format(servo_id, command, data))
        # print("checksum: {}".format(checksum))
        # print("packet: {}".format(packet))
        # print("sent: {} bytes".format(num_bytes))
