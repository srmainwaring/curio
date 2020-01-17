#!/usr/bin/env python
# 
# coding: latin-1
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

from curio_base.lx16a_driver import LX16ADriver
from curio_base.lx16a_encoder_filter import LX16AEncoderFilter
from curio_msgs.msg import CurioState
from curio_msgs.msg import LX16AState

import math
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf import TransformBroadcaster

def degree(rad):
    return rad * 180.0 / math.pi

def radian(deg):
    return deg * math.pi / 180.0

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clamp(x, lower, upper):
    return min(max(x, lower), upper)

def caseless_equal(left, right):
    return left.upper() == right.upper()

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

class MeanWindowFilter(object):
    ''' Simple rolling window filter.
    '''
    def __init__(self, window=5):
        ''' Constructor

        Parameters:
            window  size of the rolling window (default = 5)
        '''
        self._window = window
        self._index  = 0
        self._buffer = [0.0 for i in range(window)]        
        self._sum    = 0.0 
        self._mean   = 0.0

    def update(self, value):
        ''' Update the filter with the the next value

        Parameters:
            value  a float value
        '''
        # Update the ring buffer
        self._index = (self._index + 1) % self._window
        old_value = self._buffer[self._index] 
        self._buffer[self._index] = value

        # Update the stats
        self._sum  = self._sum + value - old_value
        self._mean = self._sum / self._window

    def get_mean(self):
        ''' Get the rolling mean

        Returns:
            the rolling mean
        '''
        return self._mean

    def get_window(self):
        ''' Get the size of the rolling window

        Returns:
            the size of the rolling window
        '''
        return self._window

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
        self.offset = 0.0
        self.position = [0.0, 0.0]

    @staticmethod
    def to_lat_label(label_str):
        ''' Convert a lateral label string to a enumerated value. 
        '''
        if caseless_equal(label_str, 'LEFT'):
            return Servo.LEFT
        if caseless_equal(label_str, 'RIGHT'):
            return Servo.RIGHT
        else:
            return -1
 
    @staticmethod
    def to_lon_label(label_str):
        ''' Convert a longitudinal label string to a enumerated value. 
        '''
        if caseless_equal(label_str, 'FRONT'):
            return Servo.FRONT
        if caseless_equal(label_str, 'MID'):
            return Servo.MID
        if caseless_equal(label_str, 'BACK'):
            return Servo.BACK
        else:
            return -1

class AckermannOdometry(object):
    ''' Odometry for the base controller (6 wheel Ackermann)
    
    This class based on its C++ equivalent in the ackermann_drive_controller module
    which in turn was derived from the diff_drive_controller in ros_controllers.

    Original odometry code:
    https://github.com/ros-controls/ros_controllers/diff_drive_controller

    License: BSD-3-Clause

    Copyright (c) 2013, PAL Robotics, S.L.
    All rights reserved.

    Authors
        Luca Marchionni
        Bence Magyar
        Enrique Fernandez
        Paul Mathieu
    '''

    def __init__(self, velocity_filter_window=10):
        ''' Constructor

        Parameters
            velocity_filter_window  size of the window used in the velocity filter
        '''
        self._timestamp = rospy.Time() 
        self._heading = 0.0     # [rad]
        self._x       = 0.0     # [m]
        self._y       = 0.0     # [m]
        self._lin_vel_filter = MeanWindowFilter(window=5) # [m/s]
        self._ang_vel_filter = MeanWindowFilter(window=5) # [rad/s]
        self._wheel_radius = 0.06               # [m]
        self._mid_wheel_lat_separation = 0.52   # [m]
        self._num_wheels = 6
        self._wheel_cur_pos = [0.0 for x in range(self._num_wheels)]    # [m]
        self._wheel_old_pos = [0.0 for x in range(self._num_wheels)]    # [m]
        self._wheel_est_vel = [0.0 for x in range(self._num_wheels)]    # [m/s]

    def reset(self, time):
        ''' Reset the odometry

        Parameters
            time    current time
        '''        
        self._timestamp = time

    def update_6(self, wheel_servo_pos, time):
        ''' Update the odometry with the latest wheel servo positions

        Parameters
            wheel_servo_pos     angular position of the 6 wheel servos [rad]
            time                current time
        '''
        for i in range(self._num_wheels):
            # Get the current wheel joint (linear) positions [m]
            self._wheel_cur_pos[i] = wheel_servo_pos[i] * self._wheel_radius

            # Estimate the velocity of the wheels using old and current positions
            self._wheel_est_vel[i] = self._wheel_cur_pos[i] - self._wheel_old_pos[i]

            # Update old position with current
            self._wheel_old_pos[i] = self._wheel_cur_pos[i]

        # @TODO - remove hardcoding and use a lookup instead
        MID_RIGHT = 1
        MID_LEFT  = 4

        # Compute linear and angular velocities of the mobile base (base_link frame)  
        lin_vel = (self._wheel_est_vel[MID_RIGHT] + self._wheel_est_vel[MID_LEFT]) * 0.5
        ang_vel = (self._wheel_est_vel[MID_RIGHT] - self._wheel_est_vel[MID_LEFT]) / self._mid_wheel_lat_separation

        # Integrate the velocities to get the linear and angular positions
        self._integrate_velocities(lin_vel, ang_vel)

        # Cannot estimate the speed for small time intervals
        dt = (time - self._timestamp).to_sec()
        if dt < 0.0001:
            return False

        # Estimate speeds using a rolling mean / mode to filter them
        self._timestamp = time

        # Add to velocity filters
        self._lin_vel_filter.update(lin_vel/dt)
        self._ang_vel_filter.update(ang_vel/dt)

        return True

    def update_2(self, wheel_servo_pos, time):
        ''' Update the odometry with the latest mid wheel servo positions

        Parameters
            wheel_servo_pos     angular position of the 2 mid wheel servos [rad]
            time                current time
        '''
        for i in range(2):
            # Get the current wheel joint (linear) positions [m]
            self._wheel_cur_pos[i] = wheel_servo_pos[i] * self._wheel_radius

            # Estimate the velocity of the wheels using old and current positions
            self._wheel_est_vel[i] = self._wheel_cur_pos[i] - self._wheel_old_pos[i]

            # Update old position with current
            self._wheel_old_pos[i] = self._wheel_cur_pos[i]

        LEFT  = Servo.LEFT
        RIGHT = Servo.RIGHT

        # Compute linear and angular velocities of the mobile base (base_link frame)  
        lin_vel = (self._wheel_est_vel[RIGHT] + self._wheel_est_vel[LEFT]) * 0.5
        ang_vel = (self._wheel_est_vel[RIGHT] - self._wheel_est_vel[LEFT]) / self._mid_wheel_lat_separation

        # Integrate the velocities to get the linear and angular positions
        self._integrate_velocities(lin_vel, ang_vel)

        # Cannot estimate the speed for small time intervals
        dt = (time - self._timestamp).to_sec()
        if dt < 0.0001:
            return False

        # Estimate speeds using a rolling mean / mode to filter them
        self._timestamp = time

        # Add to velocity filters
        self._lin_vel_filter.update(lin_vel/dt)
        self._ang_vel_filter.update(ang_vel/dt)

        return True

    def get_heading(self):
        ''' Get the heading [rad]
        '''
        return self._heading

    def get_x(self):
        ''' Get the x position [m]
        '''
        return self._x

    def get_y(self):
        ''' Get the y position [m]
        '''
        return self._y

    def get_lin_vel(self):
        ''' Get the linear velocity of the body [m/s]
        '''
        return self._lin_vel_filter.get_mean()

    def get_ang_vel(self):
        ''' Get the angular velocity of the body [rad/s]
        '''
        return self._ang_vel_filter.get_mean()

    def set_wheel_params(self, wheel_radius, mid_wheel_lat_separation):
        ''' Set the wheel and steering geometry
        '''
        self._wheel_radius = wheel_radius
        self._mid_wheel_lat_separation = mid_wheel_lat_separation

    def _integrate_velocities(self, lin_vel, ang_vel):
        if math.fabs(ang_vel) < 1e-6:
            self._integrate_runge_kutta2(lin_vel, ang_vel)
        else:
            #  Exact integration (should solve problems when angular is zero):
            heading_old = self._heading
            r = lin_vel/ang_vel
            self._heading = self._heading + ang_vel
            self._x = self._x + r * (math.sin(self._heading) - math.sin(heading_old))
            self._y = self._y - r * (math.cos(self._heading) - math.cos(heading_old))

    def _integrate_runge_kutta2(self, lin_vel, ang_vel):
        direction = self._heading + ang_vel * 0.5

        # Runge-Kutta 2nd order integration:
        self._x = self._x + lin_vel * math.cos(direction)
        self._y = self._y + lin_vel * math.sin(direction)
        self._heading = self._heading + ang_vel

class BaseController(object):
    ''' Mobile base controller for 6-wheel powered Ackerman steering.

    A 6-wheel Ackerman steering mobile base controller where each wheel
    is driven by a servo and there are 4-steering servos for each
    of the corner wheels.

    The LX-16A servos may be position controlled through an
    angle of 240 deg. This range is not enough to allow in-place
    steering, so we specify an angle (REVERSE_SERVO_ANGLE) at which
    the servos are reversed, so for example an angle of +130 deg is
    reversed to an angle of 130 - 180 = -50 deg.

    REVERSE_SERVO_ANGLE should be set somewhere around 90 deg.
    '''

    # Velocity limits for the rover
    LINEAR_VEL_MAX =  0.37
    ANGULAR_VEL_MAX = 1.45

    # Servo limits - LX-16A has max angular velocity of approx 1 revolution per second
    SERVO_ANG_VEL_MAX = 2 * math.pi
    SERVO_SPEED_MAX = 1000

    # Steering angle limits
    REVERSE_SERVO_ANGLE = 90.0      # The angle at which we reverse the servo by 180 deg.
    SERVO_ANGLE_MAX = 120.0         # Maximum (abs) angle at which the servo can be set.
    SERVO_POS_MIN = 0.0             # Minimum servo position (servo units).
    SERVO_POS_MAX = 1000.0          # Maximum servo position (servo units).

    # 6 wheels, 4 steering.
    NUM_WHEELS = 6
    NUM_STEERS = 4

    def __init__(self):
        rospy.loginfo('Initialising mobile base controller...')

        # Wheel geometry on a flat surface - defaults
        self._wheel_radius                = 0.060
        self._mid_wheel_lat_separation    = 0.052
        self._front_wheel_lat_separation  = 0.047
        self._front_wheel_lon_separation  = 0.028
        self._back_wheel_lat_separation   = 0.047
        self._back_wheel_lon_separation   = 0.025

        if rospy.has_param('~wheel_radius'):
            self._wheel_radius= rospy.get_param('~wheel_radius')
        if rospy.has_param('~mid_wheel_lat_separation'):
            self._mid_wheel_lat_separation = rospy.get_param('~mid_wheel_lat_separation')
        if rospy.has_param('~front_wheel_lat_separation'):
            self._front_wheel_lat_separation = rospy.get_param('~front_wheel_lat_separation') 
        if rospy.has_param('~front_wheel_lon_separation'):
            self._front_wheel_lon_separation = rospy.get_param('~front_wheel_lon_separation')
        if rospy.has_param('~back_wheel_lat_separation'):
            self._back_wheel_lat_separation = rospy.get_param('~back_wheel_lat_separation')
        if rospy.has_param('~back_wheel_lon_separation'):
            self._back_wheel_lon_separation = rospy.get_param('~back_wheel_lon_separation')
        
        rospy.loginfo('wheel_radius: {:.2f}'.format(self._wheel_radius))
        rospy.loginfo('mid_wheel_lat_separation: {:.2f}'.format(self._mid_wheel_lat_separation))
        rospy.loginfo('front_wheel_lat_separation: {:.2f}'.format(self._front_wheel_lat_separation))
        rospy.loginfo('front_wheel_lon_separation: {:.2f}'.format(self._front_wheel_lon_separation))
        rospy.loginfo('back_wheel_lat_separation: {:.2f}'.format(self._back_wheel_lat_separation))
        rospy.loginfo('back_wheel_lon_separation: {:.2f}'.format(self._back_wheel_lon_separation))

        def calc_position(lon_label, lat_label):
            ''' Calculate servo positions using the wheel geometry parameters
            '''
            if lon_label == Servo.FRONT:
                if lat_label == Servo.LEFT:
                    return [self._front_wheel_lon_separation, self._front_wheel_lat_separation/2.0]
                if lat_label == Servo.RIGHT:
                    return [self._front_wheel_lon_separation, -self._front_wheel_lat_separation/2.0]
            if lon_label == Servo.MID:
                if lat_label == Servo.LEFT:
                    return [0.0, self._mid_wheel_lat_separation/2.0]
                if lat_label == Servo.RIGHT:
                    return [0.0, -self._mid_wheel_lat_separation/2.0]
            if lon_label == Servo.BACK:
                if lat_label == Servo.LEFT:
                    return [-self._back_wheel_lon_separation, self._back_wheel_lat_separation/2.0]
                if lat_label == Servo.RIGHT:
                    return [-self._back_wheel_lon_separation, -self._back_wheel_lat_separation/2.0]

            return [0.0, 0.0]

        # Servo parameters - required
        wheel_servo_params = rospy.get_param('~wheel_servos')
        if len(wheel_servo_params) != BaseController.NUM_WHEELS:
            rospy.logerr("Parameter 'wheel_servos' must be an array length 6, got: {}"
                .format(len(wheel_servo_params)))
            return
        rospy.logdebug('wheel servo params: {}'.format(wheel_servo_params))

        steer_servo_params = rospy.get_param('~steer_servos')
        if len(steer_servo_params) != BaseController.NUM_STEERS:
            rospy.logerr("parameter 'steer_servos' must be an array length 4, got: {}"
                .format(len(steer_servo_params)))
            return
        rospy.logdebug('steer servo params: {}'.format(steer_servo_params))

        self._wheel_servos = []
        for params in wheel_servo_params:
            id = params['id']
            lon_label = Servo.to_lon_label(params['lon_label'])
            lat_label = Servo.to_lat_label(params['lat_label'])
            orientation = 1 if lat_label == Servo.LEFT else -1
            servo = Servo(id, lon_label, lat_label, orientation)
            servo.position = calc_position(lon_label, lat_label)
            self._wheel_servos.append(servo)
            rospy.logdebug('servo: id: {}, lon_label: {}, lat_label: {}, orientation: {}, offset: {}, position: {}'
                .format(servo.id, servo.lon_label, servo.lat_label, servo.orientation, servo.offset, servo.position))

        self._steer_servos = []
        for params in steer_servo_params:
            id = params['id']
            lon_label = Servo.to_lon_label(params['lon_label'])
            lat_label = Servo.to_lat_label(params['lat_label'])
            orientation = -1
            servo = Servo(id, lon_label, lat_label, orientation)            
            servo.offset = params['offset']
            servo.position = calc_position(lon_label, lat_label)
            self._steer_servos.append(servo)
            rospy.logdebug('servo: id: {}, lon_label: {}, lat_label: {}, orientation: {}, offset: {}, position: {}'
                .format(servo.id, servo.lon_label, servo.lat_label, servo.orientation, servo.offset, servo.position))

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

        # Subscriber
        self._cmd_vel_msg = Twist()
        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Tuning / calibration
        rospy.loginfo('Setting steer servo offsets...')
        self.set_steer_servo_offsets()

        # Odometry
        rospy.loginfo('Initialise odometry...')
        self._odometry = AckermannOdometry()
        self._odometry.reset(rospy.get_rostime())
        self._odometry.set_wheel_params(
            self._wheel_radius,
            self._mid_wheel_lat_separation)

        self._odom_msg = Odometry()
        self._odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self._init_odometry()

        # Encoder filters
        self._classifier_window = rospy.get_param('~classifier_window', 10)

        if not rospy.has_param('~classifier_filename'):
            rospy.logerr('Missing parameter: classifier_filename. Exiting...')
        self._classifier_filename = rospy.get_param('~classifier_filename')

        self._wheel_servo_duty = [0 for i in range(BaseController.NUM_WHEELS)]
        self._encoder_filters = [
            LX16AEncoderFilter(self._classifier_filename, self._classifier_window)
            for i in range(BaseController.NUM_WHEELS)
        ]

        for i in range(BaseController.NUM_WHEELS):
            # Invert the encoder filters on the right side
            servo = self._wheel_servos[i]
            if servo.lat_label == Servo.RIGHT:
                self._encoder_filters[i].set_invert(True)
            
        self._reset_encoders()

        # Transform
        self._odom_broadcaster = TransformBroadcaster()

        # Robot status
        self._curio_state_msg = CurioState()
        self._curio_state_pub = rospy.Publisher('status', CurioState, queue_size=10)
        self._wheel_servo_state = [LX16AState() for x in range(BaseController.NUM_WHEELS)]
        self._steer_servo_state = [LX16AState() for x in range(BaseController.NUM_STEERS)]

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
        wheel_lin_vel = []
        steer_angle = []
        if omega_p == 0:
            # Body frame has no angular velocity - set wheel velocity directly
            wheel_vel_max = math.fabs(lin_vel)
            for servo in self._wheel_servos:
                wheel_lin_vel.append(lin_vel)

            for servo in self._steer_servos:
                steer_angle.append(0.0)

        else:
            for servo in self._wheel_servos:
                # Wheel position
                id = servo.id
                x = servo.position[0]
                y = servo.position[1]

                # Wheel turn radius
                r = math.sqrt(x*x + (r_p - y)*(r_p - y))

                # Wheel velocity
                sgn = -1 if (r_p - y) < 0 else 1
                vel = sgn * r * omega_p 
                wheel_vel_max = max(wheel_vel_max, math.fabs(vel))
                wheel_lin_vel.append(vel)
                # rospy.logdebug("id: {}, r: {:.2f}, wheel_lin_vel: {:.2f}".format(id, r, vel))

            for servo in self._steer_servos:
                # Wheel position
                id = servo.id
                x = servo.position[0]
                y = servo.position[1]

                # Wheel angle
                angle = math.atan2(x, (r_p - y))
                steer_angle.append(angle)
                # rospy.logdebug("id: {}, angle: {:.2f}".format(id, degree(angle)))

        # Apply speed limiter - preserving turning radius
        if wheel_vel_max > BaseController.LINEAR_VEL_MAX:
            speed_limiter_sf = BaseController.LINEAR_VEL_MAX / wheel_vel_max
            for i in range(len(wheel_lin_vel)):
                wheel_lin_vel[i] = wheel_lin_vel[i] * speed_limiter_sf

        # Update steer servos
        # @TODO link the time of the move to the angle which the servos turn through
        rospy.logdebug('Updating steer servos')
        for i in range(len(self._steer_servos)):
            servo = self._steer_servos[i]

            # Input angles are in radians
            angle_deg = degree(steer_angle[i])

            # Transition from turning radius outside the base footprint to inside
            # (i.e in-place turning) 
            if angle_deg > BaseController.REVERSE_SERVO_ANGLE:
                angle_deg = angle_deg - 180

            if angle_deg < -BaseController.REVERSE_SERVO_ANGLE:
                angle_deg = 180 + angle_deg

            # Map steering angle degrees [-120, 120] to servo position [0, 1000]
            servo_pos = int(map(angle_deg * servo.orientation,
                -BaseController.SERVO_ANGLE_MAX, BaseController.SERVO_ANGLE_MAX,
                BaseController.SERVO_POS_MIN, BaseController.SERVO_POS_MAX))

            rospy.logdebug('id: {}, angle: {:.2f}, servo_pos: {}'.format(servo.id, angle_deg, servo_pos))
            self._servo_driver.servo_mode_write(servo.id)
            self._servo_driver.move_time_write(servo.id, servo_pos, 50)

        # Update wheel servos
        rospy.logdebug('Updating wheel servos')
        for i in range(len(self._wheel_servos)):
            servo = self._wheel_servos[i]

            # Wheel angular velocity
            wheel_ang_vel = wheel_lin_vel[i] / self._wheel_radius

            # Map speed to servo duty [-1000, 1000]
            duty = int(map(wheel_ang_vel * servo.orientation,
                -BaseController.SERVO_ANG_VEL_MAX, BaseController.SERVO_ANG_VEL_MAX, 
                -BaseController.SERVO_SPEED_MAX, BaseController.SERVO_SPEED_MAX))

            # Set servo speed
            rospy.logdebug('id: {}, wheel_ang_vel: {:.2f}, servo_vel: {}'
                .format(servo.id, wheel_ang_vel, duty))
            self._servo_driver.motor_mode_write(servo.id, duty)

            # Update duty array (needed for servo position classifier)
            self._wheel_servo_duty[i] = duty 

    def set_steer_servo_offsets(self):
        ''' Set angle offsets for the steering servos.
        
        The offsets are specified in the node parameters are are
        adjusted to ensure each corner wheel is centred when the robot
        is commanded to move with no turn.  
        '''
        # Set the steering servo offsets to centre the corner wheels
        for servo in self._steer_servos:
            rospy.loginfo('id: {}, offset: {}'.format(servo.id, servo.offset))
            self._servo_driver.angle_offset_adjust(servo.id, servo.offset)
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

    def update(self, event):
        ''' Callback for the control loop.
        
        This to be called at the control loop frequency by the node's main function,
        usually managed by a rospy.Timer.

        Parameters
            event   A rospy.Timer event. See http://wiki.ros.org/rospy/Overview/Time for details.
        '''
        # Get the current real time (just before this function was called)
        time = event.current_real

        # Read and publish
        self._update_odometry(time)
        # @TODO: move status update to a separate update loop with a lower update rate
        # self._update_status(time)
        self._publish_odometry(time)
        self._publish_tf(time)
        self._publish_status(time)

        # PID control would go here...

        # Write commands
        self.move(self._cmd_vel_msg.linear.x, self._cmd_vel_msg.angular.z)

    def shutdown(self):
        ''' Called by the node shutdown hook on exit.
        '''
        # Stop all servos - @TODO add e-stop with latch.
        self.stop()

    ################################################################################ 
    # Odometry related

    def _reset_encoders(self):
        ''' Reset the encoders
        '''
        # Reset encoder filters
        for i in range(BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            filter = self._encoder_filters[i]
            pos = self._servo_driver.pos_read(servo.id)            
            filter.reset(pos)

    # @TODO: Errors when running at low control loop update rate (e.g. < 15Hz).
    # 
    # The error is because the encoder filter is not updated fast enough to
    # have two measurements close enough in time either side of a discontinuity 
    # in the servo position to see a jump of 1000-1400 counts. Instead the
    # encoder suffers from aliasing. As result the odometry will work at
    # low velocities and then suddenly fail as it is increased.
    # 
    def _update_all_wheel_servo_positions(self, time):
        ''' Get the servo positions in radians for all wheels.
        '''
        servo_positions = [0 for i in range(BaseController.NUM_WHEELS)]
        msg = 'time: {}, '.format(time)
        for i in range (BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            filter = self._encoder_filters[i]
            state = self._wheel_servo_state[i]            

            # Calculate the encoder count
            duty = self._wheel_servo_duty[i]            
            pos = self._servo_driver.pos_read(servo.id)            

            filter.update(time, duty, pos)
            count = filter.get_count()
            theta = filter.get_angular_position()
 
            servo_positions[i] = theta

            # Update state
            state.duty = duty
            state.pos = pos
            state.count = count
            state.count_is_inverted = 0 if filter.get_invert() > 0 else 1

            # Append to debug message
            msg = msg + "{}: {}, ".format(servo.id, count)

        rospy.loginfo(msg)

        return servo_positions

    def _update_mid_wheel_servo_positions(self, time):
        ''' Update the servo positions in radians for the left and right mid wheels.
        '''

        # @TODO: resolve hardcoded index
        left_pos  = self._update_wheel_servo_position(time, 1)
        right_pos = self._update_wheel_servo_position(time, 4)

        servo_positions = [0 for i in range(2)]
        servo_positions[Servo.LEFT]  = left_pos
        servo_positions[Servo.RIGHT] = right_pos

        rospy.logdebug("time: {}, left: {}, right: {}".format(time, left_pos, right_pos))

        return servo_positions

    def _update_wheel_servo_position(self, time, i):
        ''' Update the servo positions in radians for the i-th wheel.
        '''
        servo = self._wheel_servos[i]
        filter = self._encoder_filters[i]
        state = self._wheel_servo_state[i]            

        # Calculate the encoder count
        duty = self._wheel_servo_duty[i]            
        pos = self._servo_driver.pos_read(servo.id)            

        filter.update(time, duty, pos)
        count = filter.get_count()
        theta = filter.get_angular_position()

        # Update state
        state.duty = duty
        state.pos = pos
        state.count = count
        state.count_is_inverted = 0 if filter.get_invert() > 0 else 1
        return theta

    def _init_odometry(self):
        ''' Initialise the odometry
        '''
        odom_frame_id = 'odom'
        base_frame_id = 'base_link'
        pose_cov_diag = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01] 
        twist_cov_diag = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01] 

        self._odom_msg.header.frame_id = odom_frame_id
        self._odom_msg.child_frame_id  = base_frame_id

        self._odom_msg.pose.pose.position.y = 0.0
        self._odom_msg.pose.covariance = [
            pose_cov_diag[0], 0., 0., 0., 0., 0.,
            0., pose_cov_diag[1], 0., 0., 0., 0.,
            0., 0., pose_cov_diag[2], 0., 0., 0.,
            0., 0., 0., pose_cov_diag[3], 0., 0.,
            0., 0., 0., 0., pose_cov_diag[4], 0.,
            0., 0., 0., 0., 0., pose_cov_diag[5]
        ]

        self._odom_msg.twist.twist.linear.y  = 0.0
        self._odom_msg.twist.twist.linear.z  = 0.0
        self._odom_msg.twist.twist.angular.x = 0.0
        self._odom_msg.twist.twist.angular.y = 0.0
        self._odom_msg.twist.covariance = [
            twist_cov_diag[0], 0., 0., 0., 0., 0.,
            0., twist_cov_diag[1], 0., 0., 0., 0.,
            0., 0., twist_cov_diag[2], 0., 0., 0.,
            0., 0., 0., twist_cov_diag[3], 0., 0.,
            0., 0., 0., 0., twist_cov_diag[4], 0.,
            0., 0., 0., 0., 0., twist_cov_diag[5]
        ]

    def _publish_odometry(self, time):
        ''' Populate the nav_msgs.Odometry message and publish.
        '''
        # rospy.loginfo('x: {:.2f}, y: {:.2f}, heading: {:.2f}, lin_vel: {:.2f}, ang_vel: {:.2f}'
        #     .format(
        #         self._odometry.get_x(),
        #         self._odometry.get_y(),
        #         self._odometry.get_heading(),
        #         self._odometry.get_lin_vel(),
        #         self._odometry.get_ang_vel()))

        quat = quaternion_from_euler(0.0, 0.0, self._odometry.get_heading())

        self._odom_msg.header.stamp = time
        self._odom_msg.pose.pose.position.x   = self._odometry.get_x()
        self._odom_msg.pose.pose.position.y   = self._odometry.get_y()
        self._odom_msg.pose.pose.orientation.x = quat[0]
        self._odom_msg.pose.pose.orientation.y = quat[1]
        self._odom_msg.pose.pose.orientation.z = quat[2]
        self._odom_msg.pose.pose.orientation.w = quat[3]
        self._odom_msg.twist.twist.linear.x    = self._odometry.get_lin_vel()
        self._odom_msg.twist.twist.angular.z   = self._odometry.get_ang_vel()

        self._odom_pub.publish(self._odom_msg)

    def _update_odometry(self, time):
        ''' Update odometry

        This is the same calculation as used in the odometry
        for the ackermann_drive_controller.
        '''
        # Get the angular position of the all wheel servos [rad] and update odometry
        # wheel_servo_pos = self._update_all_wheel_servo_positions(time)
        # self._odometry.update_6(wheel_servo_pos, time)

        # Get the angular position of the mid wheel servos [rad] and update odometry
        wheel_servo_pos = self._update_mid_wheel_servo_positions(time)
        self._odometry.update_2(wheel_servo_pos, time)

    def _publish_tf(self, time):
        ''' Publish the transform from 'odom' to 'base_link'
        '''
        # Broadcast the transform from 'odom' to 'base_link'
        self._odom_broadcaster.sendTransform(
            (self._odometry.get_x(), self._odometry.get_y(), 0.0),
            quaternion_from_euler(0.0, 0.0, self._odometry.get_heading()),
            time,
            'base_link',
            'odom')

    def _update_status(self, time):
        ''' Update the rover's status
        '''
        for i in range (BaseController.NUM_WHEELS):
            servo = self._wheel_servos[i]
            state = self._wheel_servo_state[i]
            state.servo_id = servo.id
            state.temperature = self._servo_driver.temp_read(servo.id)
            state.voltage = self._servo_driver.vin_read(servo.id)
            state.angle_offset = self._servo_driver.angle_offset_read(servo.id)
            state.mode = LX16AState.LX16A_MODE_MOTOR

        for i in range (BaseController.NUM_STEERS):
            servo = self._steer_servos[i]
            state = self._steer_servo_state[i]
            state.servo_id = servo.id
            state.temperature = self._servo_driver.temp_read(servo.id)
            state.voltage = self._servo_driver.vin_read(servo.id)
            state.angle_offset = self._servo_driver.angle_offset_read(servo.id)
            state.mode = LX16AState.LX16A_MODE_SERVO

    def _publish_status(self, time):
        ''' Publish the rover's status
        '''
        # Header
        self._curio_state_msg.header.stamp = time
        self. _curio_state_msg.header.frame_id = 'base_link'

        # LX16A state
        self._curio_state_msg.wheel_servo_state = self._wheel_servo_state
        self._curio_state_msg.steer_servo_state = self._steer_servo_state

        # Publish rover state
        self._curio_state_pub.publish(self._curio_state_msg)
