#!/usr/bin/env python

# Python implementation of the Arduino firmware code for testing. 
# 
# 

import rospy
from lx16a.driver import LX16ADriver
from curio_msgs.msg import CurioServoCommands
from curio_msgs.msg import CurioServoPositions
from curio_msgs.msg import CurioServoStates
from curio_msgs.msg import LX16AState

SERVO_SERIAL_PORT   = '/dev/cu.usbmodemFD5121'
SERVO_BAUDRATE      = 115200    # [bits/second]
SERVO_TIMEOUT       = 1.0       # [s]
SERVO_COMMAND_TIMEOUT = 0.75    # [s]

class ServoController(object):

    NUM_WHEELS = 6
    NUM_STEERS = 4

    def __init__(self):
        # Initialise servo driver
        self._servo_driver = LX16ADriver()
        self._servo_driver.set_port(SERVO_SERIAL_PORT)
        self._servo_driver.set_baudrate(SERVO_BAUDRATE)
        self._servo_driver.set_timeout(SERVO_TIMEOUT)
        self._servo_driver.open()
        
        rospy.loginfo('Open connection to servo bus board')
        rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))
        rospy.loginfo('port: {}'.format(self._servo_driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        rospy.loginfo('timeout: {}'.format(self._servo_driver.get_timeout()))

        # Arduino reboots when a serial connection is established
        # wait for bootloader to complete scanning the serial port
        # before sending data.
        rospy.loginfo('Wait for bootloader to complete scan...')
        rospy.sleep(1)

        # Get parameters
        self._wheel_ids = [ 11, 12, 13, 21, 22, 23 ]
        self._steer_ids = [ 111, 131, 211, 231 ]
        self._steer_angle_offsets = [ 0, 0, 0, 0 ]

        if rospy.has_param('~wheel_servo_ids'):
            self._wheel_ids = rospy.get_param('~wheel_servo_ids')
            rospy.loginfo('Loaded params: wheel_servo_ids')

        if rospy.has_param('~steer_servo_ids'):
            self._steer_ids = rospy.get_param('~steer_servo_ids')
            rospy.loginfo('Loaded params: steer_servo_ids')

        if rospy.has_param('~steer_angle_offsets'):
            self._steer_angle_offsets = rospy.get_param('~steer_angle_offsets')
            rospy.loginfo('Loaded params: steer_angle_offsets')

        # Publications
        self._servo_pos_msg = CurioServoPositions()
        self._servo_pos_msg.wheel_positions = [0 for i in range(ServoController.NUM_WHEELS)]
        self._servo_pos_msg.steer_positions = [500 for i in range(ServoController.NUM_STEERS)]
        self._curio_pos_pub = rospy.Publisher('/servo/positions', CurioServoPositions, queue_size=10)

        self._states_msg = CurioServoStates()
        self._states_msg.wheel_states = [LX16AState() for x in range(ServoController.NUM_WHEELS)]
        self._states_msg.steer_states = [LX16AState() for x in range(ServoController.NUM_STEERS)]
        self._states_pub = rospy.Publisher('servo/states', CurioServoStates, queue_size=10)

        # Subscriptions
        self._last_cmd_rec_time = rospy.Time()
        self._commands_msg = CurioServoCommands()
        self._commands_msg.wheel_commands = [0 for i in range(ServoController.NUM_WHEELS)]
        self._commands_msg.steer_commands = [500 for i in range(ServoController.NUM_STEERS)]
        self._commands_sub = rospy.Subscriber('/servo/commands', CurioServoCommands, self._servo_cmd_callback)

        # Set steering trim and servo mode
        rospy.loginfo('Setting steering trim')
        for i in range(ServoController.NUM_STEERS):
            servo_id = self._steer_ids[i]
            deviation = self._steer_angle_offsets[i]
            self._servo_driver.servo_mode_write(servo_id)
            self._servo_driver.angle_offset_adjust(servo_id, deviation)
            self._servo_driver.angle_offset_write(servo_id)


    def update(self, event):
        # Get the current real time (just before this function was called)
        time = event.current_real

        # Read wheel servos
        for i in range(ServoController.NUM_WHEELS):
            servo_id = self._wheel_ids[i]
            pos = self._servo_driver.pos_read(servo_id)
            if pos != LX16ADriver.ERROR_VALUE_POS:
                self._servo_pos_msg.wheel_positions[i] = pos
            # rospy.loginfo('servo_id: {}, pos: {}'.format(servo_id, pos))

        # Read steer servos
        for i in range(ServoController.NUM_STEERS):
            servo_id = self._steer_ids[i]
            pos = self._servo_driver.pos_read(servo_id)
            if pos != LX16ADriver.ERROR_VALUE_POS:
                self._servo_pos_msg.steer_positions[i] = pos
            # rospy.loginfo('servo_id: {}, pos: {}'.format(servo_id, pos))

        # Publish servo positions
        self._servo_pos_msg.header.stamp = time
        self._servo_pos_msg.header.frame_id = 'base_link'
        self._curio_pos_pub.publish(self._servo_pos_msg)

        # Publish servo states
        self._states_pub.publish(self._states_msg)

        # Send move commands to wheel and steering servos
        servo_command_timeout = rospy.Duration.from_sec(SERVO_COMMAND_TIMEOUT)
        has_timed_out = rospy.Time.now() - self._last_cmd_rec_time > servo_command_timeout

        for i in range(ServoController.NUM_WHEELS):
            servo_id = self._wheel_ids[i]
            cmd_duty = 0 if has_timed_out else self._commands_msg.wheel_commands[i]
            self._servo_driver.motor_mode_write(servo_id, cmd_duty)
            # rospy.loginfo('servo_id: {}, duty: {}'.format(servo_id, cmd_duty))

        for i in range(ServoController.NUM_STEERS):
            servo_id = self._steer_ids[i]
            cmd_pos = self._commands_msg.steer_commands[i]
            self._servo_driver.move_time_write(servo_id, cmd_pos)
            # rospy.loginfo('servo_id: {}, pos: {}'.format(servo_id, cmd_pos))


    def shutdown(self):
        # Stop all servos

        # Close port
        self._servo_driver.close()

    def _servo_cmd_callback(self, msg):
        # Update watchdog for servo commands
        self._last_cmd_rec_time = rospy.Time.now()

        # Set the current command message
        self._commands_msg = msg

if __name__ == '__main__':
    rospy.init_node('curio_servo_controller')
    rospy.loginfo('Starting Curio servo controller')

    # Base controller
    servo_controller = ServoController()

    # Register shutdown behaviour
    def shutdown_callback():
        rospy.loginfo('Shutdown Curio base controller...')
        servo_controller.shutdown()

    rospy.on_shutdown(shutdown_callback)

    # Start the control loop
    control_frequency = 15.0
    if rospy.has_param('~control_frequency'):
        control_frequency = rospy.get_param('~control_frequency')

    rospy.loginfo('Starting control loop at {} Hz'.format(control_frequency))
    control_timer = rospy.Timer(
        rospy.Duration(1.0 / control_frequency),
        servo_controller.update)

    # Wait for shutdown
    rospy.spin()
