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

''' IMU calibration client

Test the use of ROS actionlib to call IMU calibration routines.
'''

import rospy
import actionlib
from curio_base.msg import CalibrateImuAction, CalibrateImuGoal 
from actionlib_msgs.msg import GoalStatus 

def goal_state_to_string(state):
    ''' http://docs.ros.org/melodic/api/actionlib_msgs/html/msg/GoalStatus.html
    '''
    if state == GoalStatus.PENDING:
        return "Pending"
    elif state == GoalStatus.ACTIVE:
        return 'Active'
    elif state == GoalStatus.PREEMPTED:
        return 'Preempted'
    elif state == GoalStatus.SUCCEEDED:
        return 'Succeeded'
    elif state == GoalStatus.ABORTED:
        return 'Aborted'
    elif state == GoalStatus.REJECTED:
        return 'Rejected'
    elif state == GoalStatus.PREEMPTING:
        return 'Preempting'
    elif state == GoalStatus.RECALLING:
        return 'Recalling'
    elif state == GoalStatus.RECALLED:
        return 'Recalled'
    elif state == GoalStatus.LOST:
        return 'Lost'
    else:
        return 'Unknown'

def action_done(state, result):
    rospy.loginfo('Finished: {}'.format(goal_state_to_string(state)))
    rospy.loginfo('Result:   {}'.format(result))

def action_active():
    rospy.loginfo('Goal is active')

def action_feedback(feedback):
    rospy.loginfo('Feedback: {}'.format(feedback.calibration_status))

def imu_calibration_client():
    ''' Creates a SimpleActionClient for calibrating an IMU
    '''
    client = actionlib.SimpleActionClient('calibrate_imu', CalibrateImuAction)

    rospy.loginfo('Waiting for action server to start...')
    client.wait_for_server()

    # Create and send the goal
    rospy.loginfo('Action server started, sending goal.')
    goal = CalibrateImuGoal()
    goal.calibration_type = 1
    client.send_goal(goal,
        done_cb=action_done,
        active_cb=action_active,
        feedback_cb=action_feedback)

if __name__ == '__main__':
    try:
        rospy.init_node('imu_calibration_client')
        rospy.loginfo('Starting Imu calibration client')
        imu_calibration_client()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Program interrupted before completion')
