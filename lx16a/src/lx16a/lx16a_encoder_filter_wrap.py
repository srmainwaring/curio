from lx16a import LX16AEncoderFilter as LX16AEncoderFilterBase
from lx16a.lx16a_encoder_filter import LX16AEncoderFilter
import rospy

# Implement the interface defined by C++ extension LX16EncoderFilterBase
class LX16AEncoderFilterWrap(LX16AEncoderFilterBase):
    def __init__(self, classifier_filename, regressor_filename=None, window=10):
        LX16AEncoderFilterBase.__init__(self)
        self._classifier_filename = classifier_filename
        self._regressor_filename = regressor_filename
        self._window = window
        self._encoder_filters = {}

    def init(self):
        pass

    def add(self, servo_id):
        filter =  LX16AEncoderFilter(
            self._classifier_filename,
            self._regressor_filename,
            self._window)
        self._encoder_filters[servo_id] = filter

    # Note: the Python overload expects a lx16a.Time
    def update(self, servo_id, time, duty, position):
        ros_time = rospy.Time(time.sec, time.nsec)
        filter = self._encoder_filters[servo_id]
        filter.update(ros_time, duty, position)

    def get_revolutions(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_revolutions()

    def get_count(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_count()

    def get_duty(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_duty()

    def get_angular_position(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_angular_position()

    # Note: the Python overload returns a tuple (position, is_valid)
    def get_servo_position(self, servo_id, map_position=True):
        filter = self._encoder_filters[servo_id]
        position, is_valid = filter.get_servo_pos(map_position)
        return position, is_valid

    def get_invert(self, servo_id):
        filter = self._encoder_filters[servo_id]
        return filter.get_invert()

    def set_invert(self, servo_id, is_inverted):
        filter = self._encoder_filters[servo_id]
        filter.set_invert(is_inverted)

    def reset(self, servo_id, position):
        filter = self._encoder_filters[servo_id]
        filter.reset(position)

    def add_v(self, servo_ids):
        for id in servo_ids:
            self.add(servo_id)

    # Note: the Python overload expects a lx16a.Time and returns a list
    def update_v(self, servo_ids, time, duties, positions):
        ros_time = rospy.Time(time.sec, time.nsec)
        i = 0
        angular_positions = [0 for x in servo_ids]
        for id in servo_ids:
            filter = self._encoder_filters[servo_id]
            filter.update_v(ros_time, duties[i], positions[i])
            angular_positions[i] = filter.get_angular_position()
            i = i + 1
        return angular_positions
        
