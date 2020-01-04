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

import math
import rospy
from geometry_msgs.msg import Twist
import curio_base.lx16a_driver

def degree(rad):
    return rad * 180.0 / math.pi

def radian(deg):
    return deg * math.pi / 180.0

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clamp(x, lower, upper):
    return min(max(x, lower), upper)

def turning_radius_and_rate(v_b, omega_b, d):
    ''' Calculate the turning radius and rate of turn about
    the instantaneous centre of curvature (ICC).

    Conventions are specifiied according to ROS REP 103:
    Standard Units of Measure and Coordinate Conventions
    https://www.ros.org/reps/rep-0103.html.

    x forward
    y left
    z up

    Example:
    v_b >= 0, omega_b > 0 => r_p > 0    the turn is positive (anti-clockwise),
    v_b >= 0, omega_b < 0 => r_p < 0    the turn is negative (clockwise),
    v_b >= 0, omega_b = 0 => r_p = inf  there is no turn.

    Parameters
    v_b       linear velocity of the base (m/s).
    omega_b   angular velocity of the base (rad/s).
    d         distance between the fixed wheels (m).

    Returns
    r_p, omega_p  turning radius (m) and rate of turn (rad/s).
    '''

    vl = v_b - d * omega_b / 2.0
    vr = v_b + d * omega_b / 2.0
    if vl == vr:
        return float('Inf'), 0.0
    else:
        r_p = d * (vr + vl) / (vr - vl) / 2.0
        omega_p = (vr - vl) / d
        return r_p, omega_p

class Servo(object):
    '''Servo properties.
    
    Store information about the servo:
    id                  servo serial id: 0 - 253.
    lon_label           label for the longitudinal direction: FRONT, MID, BACK.
    lat_label           label for the lateral direction: LEFT, RIGHT.
    orientation         indicated whether the servo is installed for positive or negtive rotation: 1, -1.
    offset              servo position offset (for servo rather than motor mode). Use to centre servos.
    position            servo position in the robot base. Two dimensional coordinate vector.    
    '''
    # Lateral labels
    LEFT  = 0
    RIGHT = 1

    # Longitudinal labels
    FRONT = 2
    MID   = 3
    BACK  = 4

    def __init__(self, id, lon_label, lat_label, orientation):
        self.id = id
        self.lon_label = lon_label
        self.lat_label = lat_label
        self.orientation = orientation
        self.offset = None
        self.position = None

class MotorController(object):
    ''' Motor controller for 6-wheel powered Ackerman steering.

    A 6-wheel Ackerman steering motor controller where each wheel
    is driven by a servo and there are 4-steering servos for each
    of the corner wheels.

    The LX-16A servos may be position controlled through an
    angle of 240 deg. This range is not enough to allow in-place
    steering, so we specify an angle (REVERSE_SERVO_ANGLE) at which
    the servos are reversed, so for example an angle of +130 deg is
    reversed to an angle of 130 - 180 = -50 deg.

    REVERSE_SERVO_ANGLE should be set somewhere around 90 deg.
    '''

    # Velocity limits
    VEL_MAX = 1.0
    SERVO_SPEED_MAX = 1000.0

    # Steering angle limits
    REVERSE_SERVO_ANGLE = 90.0      # The angle at which we reverse the servo by 180 deg.
    SERVO_ANGLE_MAX = 120.0         # Maximum (abs) angle at which the servo can be set.
    SERVO_POS_MIN = 0.0             # Minimum servo position (servo units).
    SERVO_POS_MAX = 1000.0          # Maximum servo position (servo units).

    def __init__(self):
        rospy.loginfo('Initialising motor controller...')

        # Steering offsets (offsets are in servo position units)
        self._steer_servo_offsets = [
            {'id': 111, 'offset': 0},
            {'id': 121, 'offset': 0},
            {'id': 211, 'offset': 0},
            {'id': 231, 'offset': 0},
        ]
        if rospy.has_param('~steer_servo_offsets'):
            self._steer_servo_offsets = rospy.get_param('~steer_servo_offsets')

        for offset in self._steer_servo_offsets:
            rospy.loginfo('steer offset: id: {}, offset: {}'.format(offset['id'], offset['offset']))
            
        # Wheel geometry on a flat surface - all parameters required
        self._wheel_radius                = rospy.get_param('~wheel_radius')
        self._wheel_width                 = rospy.get_param('~wheel_width')
        self._mid_wheel_lat_separation    = rospy.get_param('~mid_wheel_lat_separation')
        self._front_wheel_lat_separation  = rospy.get_param('~front_wheel_lat_separation') 
        self._front_wheel_lon_separation  = rospy.get_param('~front_wheel_lon_separation')
        self._back_wheel_lat_separation   = rospy.get_param('~back_wheel_lat_separation')
        self._back_wheel_lon_separation   = rospy.get_param('~back_wheel_lon_separation')
        rospy.loginfo('wheel_radius: {:.2f}'.format(self._wheel_radius))
        rospy.loginfo('wheel_width: {:.2f}'.format(self._wheel_width))
        rospy.loginfo('mid_wheel_lat_separation: {:.2f}'.format(self._mid_wheel_lat_separation))
        rospy.loginfo('front_wheel_lat_separation: {:.2f}'.format(self._front_wheel_lat_separation))
        rospy.loginfo('front_wheel_lon_separation: {:.2f}'.format(self._front_wheel_lon_separation))
        rospy.loginfo('back_wheel_lat_separation: {:.2f}'.format(self._back_wheel_lat_separation))
        rospy.loginfo('back_wheel_lon_separation: {:.2f}'.format(self._back_wheel_lon_separation))

        # Location of each wheel joint in the base link frame (x-forward, y-left, z-up)
        self._wheel_joint_locations = [
            {'id': 11, 'pos': [self._front_wheel_lon_separation, self._front_wheel_lat_separation/2.0]},
            {'id': 12, 'pos': [0.0, self._mid_wheel_lat_separation/2.0]},
            {'id': 13, 'pos': [-self._back_wheel_lon_separation, self._back_wheel_lat_separation/2.0]},
            {'id': 21, 'pos': [self._front_wheel_lon_separation, -self._front_wheel_lat_separation/2.0]},
            {'id': 22, 'pos': [0.0, -self._mid_wheel_lat_separation/2.0]},
            {'id': 23, 'pos': [-self._back_wheel_lon_separation, -self._back_wheel_lat_separation/2.0]}
        ]
        for location in self._wheel_joint_locations:
            rospy.loginfo('wheel joint location: id: {}, pos: {}'.format(location['id'], location['pos']))

        # Location of each steer joint in the base link frame (x-forward, y-left, z-up)
        self._steer_joint_locations = [
            {'id': 111, 'pos': [self._front_wheel_lon_separation, self._front_wheel_lat_separation/2.0]},
            {'id': 131, 'pos': [-self._back_wheel_lon_separation, self._back_wheel_lat_separation/2.0]},
            {'id': 211, 'pos': [self._front_wheel_lon_separation, -self._front_wheel_lat_separation/2.0]},
            {'id': 231, 'pos': [-self._back_wheel_lon_separation, -self._back_wheel_lat_separation/2.0]}
        ]
        for location in self._steer_joint_locations:
            rospy.loginfo('steer joint location: id: {}, pos: {}'.format(location['id'], location['pos']))

        # Configure servos
        self._wheel_servos = [
            Servo(11, Servo.FRONT, Servo.LEFT, 1),
            Servo(12, Servo.MID, Servo.LEFT, 1),
            Servo(13, Servo.BACK, Servo.LEFT, 1),
            Servo(21, Servo.FRONT, Servo.RIGHT, -1),
            Servo(22, Servo.MID, Servo.RIGHT, -1),
            Servo(23, Servo.BACK, Servo.RIGHT, -1)
        ]
        self._steer_servos = [
            Servo(111, Servo.FRONT, Servo.LEFT, -1),
            Servo(131, Servo.BACK, Servo.LEFT, -1),
            Servo(211, Servo.FRONT, Servo.RIGHT, -1),
            Servo(231, Servo.BACK, Servo.RIGHT, -1)
        ]

        # LX-16A servo driver - all parameters are required
        rospy.loginfo('Opening connection to servo board...')
        port      = rospy.get_param('~port')
        baudrate  = rospy.get_param('~baudrate')
        timeout   = rospy.get_param('~timeout')
        self._servo_driver = curio_base.lx16a_driver.LX16ADriver()
        self._servo_driver.set_port(port)
        self._servo_driver.set_baudrate(baudrate)
        self._servo_driver.set_timeout(timeout)
        self._servo_driver.open()        
        rospy.loginfo('is_open: {}'.format(self._servo_driver.is_open()))
        rospy.loginfo('port: {}'.format(self._servo_driver.get_port()))
        rospy.loginfo('baudrate: {}'.format(self._servo_driver.get_baudrate()))
        rospy.loginfo('timeout: {:.2f}'.format(self._servo_driver.get_timeout()))

        # Subscriber
        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub =  rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Tuning / calibration
        rospy.loginfo('Setting steer servo offsets...')
        self.set_steer_servo_offsets()

    def move(self, lin_vel, ang_vel):
        ''' Move the robot given linear and angular velocities for the base.

        The linear and angular velocity arguments refer to the robot's base_link
        reference frame. We assume that the base_link origin is located at the
        mid-point between the two middle (non-steering) wheels. 
        
        The velocities and separation of the middle wheels are used to determine
        a turning radius and rate of turn. Given this the velocity and steering
        angle are then calculated for each wheel.

        Parameters

        lin_vel     linear velocity of base_link frame (m/s).
        ang_vel     angular velocity of base_link frame (rad/s).
        '''
        # Calculate the turning radius and rate 
        r_p, omega_p = turning_radius_and_rate(lin_vel, ang_vel, self._mid_wheel_lat_separation)
        rospy.logdebug('r_p: {:.2f}, omega_p: {:.2f}'.format(r_p, omega_p))

        # Calculate velocity and steering angle for each wheel
        wheel_vel_max = 0.0
        wheel_vel = []
        steer_angle = []
        if omega_p == 0:
            # No rotation - set wheel velocity directly
            wheel_vel_max = math.fabs(lin_vel)
            for i in range(len(self._wheel_servos)):
                joint = self._wheel_joint_locations[i]
                servo = self._wheel_servos[i]
                wheel_vel.append(lin_vel)

            for i in range(len(self._steer_servos)):
                steer_angle.append(0.0)

        else:
            for joint in self._wheel_joint_locations:
                # Wheel position
                id = joint['id']
                x = joint['pos'][0]
                y = joint['pos'][1]

                # Wheel turn radius
                r = math.sqrt(x*x + (r_p - y)*(r_p - y))

                # Wheel velocity
                sgn = -1 if (r_p - y) < 0 else 1
                vel = sgn * r * omega_p 
                wheel_vel_max = max(wheel_vel_max, math.fabs(vel))
                wheel_vel.append(vel)
                rospy.logdebug("id: {}, r: {:.2f}, wheel_vel: {:.2f}".format(joint['id'], r, vel))

            for joint in self._steer_joint_locations:
                # Wheel position
                id = joint['id']
                x = joint['pos'][0]
                y = joint['pos'][1]

                # Wheel angle
                angle = math.atan2(x, (r_p - y))
                steer_angle.append(angle)
                # rospy.loginfo("id: {}, angle: {:.2f}".format(joint['id'], degree(angle)))

        # Apply speed limiter - preserving turning radius
        if wheel_vel_max > MotorController.VEL_MAX:
            speed_limiter_sf = MotorController.VEL_MAX / wheel_vel_max
            for i in range(len(self._wheel_servos)):
                wheel_vel[i] = wheel_vel[i] * speed_limiter_sf

        # Update steer servos
        rospy.logdebug('Updating steer servos')
        for i in range(len(self._steer_servos)):
            servo = self._steer_servos[i]

            # Input angles are in radians
            angle_deg = degree(steer_angle[i])

            # Transition from turning radius outside the base footprint to inside
            # (i.e in-place turning) 
            if angle_deg > MotorController.REVERSE_SERVO_ANGLE:
                angle_deg = angle_deg - 180

            if angle_deg < -MotorController.REVERSE_SERVO_ANGLE:
                angle_deg = 180 + angle_deg

            # Map steering angle degrees [-120, 120] to servo position [0, 1000]
            servo_pos = int(map(angle_deg * servo.orientation,
                -MotorController.SERVO_ANGLE_MAX, MotorController.SERVO_ANGLE_MAX,
                MotorController.SERVO_POS_MIN, MotorController.SERVO_POS_MAX))

            rospy.logdebug('id: {}, angle: {:.2f}, servo_pos: {}'.format(servo.id, angle_deg, servo_pos))
            self._servo_driver.servo_mode_write(servo.id)
            self._servo_driver.move_time_write(servo.id, servo_pos)

        # Update wheel servos
        rospy.logdebug('Updating wheel servos')
        for i in range(len(self._wheel_servos)):
            servo = self._wheel_servos[i]

            # Map speed to servo speed [-1000, 1000]
            servo_speed = int(map(wheel_vel[i] * servo.orientation,
                -MotorController.VEL_MAX, MotorController.VEL_MAX, 
                -MotorController.SERVO_SPEED_MAX, MotorController.SERVO_SPEED_MAX))

            # Set servo speed
            rospy.logdebug('id: {}, vel: {:.2f}, servo_vel: {}'.format(servo.id, wheel_vel[i], servo_speed))
            self._servo_driver.motor_mode_write(servo.id, servo_speed)

    def set_steer_servo_offsets(self):
        ''' Set angle offsets for the steering servos.
        
        The offsets are specified in the node parameters are are
        adjusted to ensure each corner wheel is centred when the robot
        is commanded to move with no turn.  
        '''
        # @TODO add check that offset has the same servo id as servo object. 

        # Set the steering servo offsets to centre the corner wheels
        for i in range(len(self._steer_servos)):
            servo = self._steer_servos[i]

            offset = self._steer_servo_offsets[i]['offset']

            rospy.logdebug('id: {}, offset: {}'.format(servo.id, offset))
            self._servo_driver.angle_offset_adjust(servo.id, offset)
            # self._servo_driver.angle_offset_write(servo.id)

    def stop(self):
        ''' Stop all servos
        '''
        rospy.loginfo('Stopping all servos')
        for servo in self._wheel_servos:
            self._servo_driver.motor_mode_write(servo.id, 0)
    
    def cmd_vel_callback(self, msg):
        ''' Callback for the subscription to /cmd_vel.

        Parameters

        msg     geometry_msgs/Twist message for the commanded velocity.
        '''
        rospy.logdebug('cmd_vel: linear: {}, angular: {}'.format(msg.linear.x, msg.angular.z))
        self._cmd_vel_msg = msg

    def control_loop(self, event):
        ''' Callback for the motor control loop.
        
        This to be called at the control_loop frequency by the node's main function,
        usually managed by a rospy.Timer.

        Parameters

        event       A rospy.Timer event. See http://wiki.ros.org/rospy/Overview/Time for details.
        '''
        # Write
        self.move(self._cmd_vel_msg.linear.x, self._cmd_vel_msg.angular.z)

    def shutdown(self):
        ''' Called by the node shutdown hook on exit.
        '''
        # Stop all servos - @TODO add e-stop with latch.
        self.stop()
