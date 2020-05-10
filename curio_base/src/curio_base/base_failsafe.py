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

''' Curio base failsafe
'''

from lx16a.lx16a_driver import LX16ADriver
import rospy
import serial

class BaseFailsafe(object):
    ''' Failsafe for the mobile base

    This class connects to the Lewansoul Servo BusLinker board
    via USB serial and attempts to stop all the wheel servos every time
    `update` is called.

    Motivation: the rover uses an Arduino to connect to the
    servo bus board using its UART header. The Arduino in turn
    is connected to the rover's on board computer using its USB port.
    Communication between the computer and the Arduino is managed by
    the rosserial stack: we run rosserial_python
    on the computer and rosserial_arduino on the Arduino.

    By and large this works well, but we have found a few cases
    where the controller can leave the servos in a runaway state:

    - hardware reset of the Arduino
    - watchdog timer reset of the Arduino
    - rosserial error: 'Lost sync with device, restarting...' 

    The Arduino may fail to run the program in flash after the reset
    (i.e. setup() is not called). This happens when rosserial continues
    to publish data to the Arduino via USB serial after the reset
    (i.e. rosserial_arduino has registered one or more subscribers).
    rosserial makes no attempt to stop pushing data while it attempts
    to resync, and this seems to be confusing the Arduino so that it
    never moves on from its bootloader sequence. 

    As a result the servos are left powered on and running at
    whatever duty was last set in the Arduino control loop. This class
    takes advantage of the fact that the serial header on the
    Lewansoul BusLinker board appears to have priority over the
    USB-to-TTL circuit. As a result we are able to run a background
    process that attempts to stop the servos which is ignored under
    usual operating conditions. However as soon as the Arduino is reset 
    and stops transmitting and receiving to the servo board the failsafe
    becomes effective and stops the servos. 

    Attributes
    ----------
    NUM_WHEELS : int
        The number of wheel servos), has (constant 6)

    Parameters
    ----------
    ~wheel_servo_ids : list
        An array of integer wheel servo serial ids : 0 - 253
    ~port : str
        The device name for the serial port (e.g. /dev/ttyUSB0)
    ~baudrate : int
        The baudrate, has default (115200).
    ~timeout : float
        The time in seconds out for the serial connection,
        has (default 1.0)
    '''

    # 6 wheels
    NUM_WHEELS = 6

    def __init__(self):
        ''' Constructor
        '''

        rospy.loginfo('Initialising BaseFailsafe...')

        # LX-16A servo driver - all parameters are required
        rospy.loginfo('Opening connection to servo bus board...')
        port      = rospy.get_param('~port')
        baudrate  = rospy.get_param('~baudrate')
        timeout   = rospy.get_param('~timeout')
        self._servo_driver = LX16ADriver()
        self._servo_driver.set_port(port)
        self._servo_driver.set_baudrate(baudrate)
        self._servo_driver.set_timeout(timeout)
        self._servo_driver.open()        
        rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))
        rospy.loginfo('port: {}'.format(self._servo_driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        rospy.loginfo('timeout: {:.2f}'.format(self._servo_driver.get_timeout()))

        # Utility for validating servo parameters
        def validate_servo_param(param, name, expected_length):
            if len(param) != expected_length:
                rospy.logerr("Parameter '{}' must be an array length {}, got: {}"
                    .format(name, expected_length, len(param)))
                exit()

        # Wheel servo parameters - required
        self._wheel_servo_ids = rospy.get_param('~wheel_servo_ids')
        validate_servo_param(self._wheel_servo_ids, 'wheel_servo_ids', BaseFailsafe.NUM_WHEELS)

    def update(self, event):
        ''' Update will stop all wheel servos.

        Parameters
        ----------
        event : rospy.Timer
            A rospy.Timer event.
        '''
        for i in range(BaseFailsafe.NUM_WHEELS):
            servo_id = self._wheel_servo_ids[i]
            self._servo_driver.motor_mode_write(servo_id, 0)

def main():
    rospy.init_node('curio_base_failsafe')
    rospy.loginfo('Starting Curio base failsafe')

    # Base failsafe
    base_failsafe = BaseFailsafe()

    # Start the control loop
    control_frequency = 20.0
    if rospy.has_param('~failsafe_control_frequency'):
        control_frequency = rospy.get_param('~failsafe_control_frequency')

    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        base_failsafe.update)

    # Wait for shutdown
    rospy.spin()
