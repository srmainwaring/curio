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

''' IMU calibration server

Test the use of ROS actionlib to call IMU calibration routines.
'''

import rospy
import actionlib
from curio_base.msg import CalibrateImuAction
from curio_base.msg import CalibrateImuFeedback
from curio_base.msg import CalibrateImuResult

class CalibrateImuServer(object):

    def __init__(self):
        self.action_name_ = 'calibrate_imu'
        self.feedback_ = CalibrateImuFeedback()
        self.result_ = CalibrateImuResult()

        self.server_ = actionlib.SimpleActionServer(
            self.action_name_,
            CalibrateImuAction,
            execute_cb=self.execute,
            auto_start=False
        )
        self.server_.start()

    def execute(self, goal):
        '''
        '''
        rospy.loginfo('{}: Executing: calibration_type: {}'
            .format(self.action_name_, goal.calibration_type))

        rate = rospy.Rate(0.5)
        success = True

        self.feedback_.calibration_status = 0

        # Mock calibration
        for i in  range(5):
            # Check that prempt has not been requested by the client
            if self.server_.is_preempt_requested():
                rospy.loginfo('{}: Preempted'.format(self.action_name_))
                self.server_.set_preempted()
                success = False
                break
            self.feedback_.calibration_status = 0
            self.server_.publish_feedback(self.feedback_)
            rate.sleep()

        if success:
            self.result_.calibration_status = 1
            rospy.loginfo('{}: Succeeded'.format(self.action_name_))
            self.server_.set_succeeded(self.result_)

if __name__ == '__main__':
    rospy.init_node('imu_calibration_server')
    rospy.loginfo('Starting Imu calibration server')
    server = CalibrateImuServer()
    rospy.spin()
