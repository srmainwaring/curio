//
//  Software License Agreement (BSD-3-Clause)
//   
//  Copyright (c) 2020 Rhys Mainwaring
//  All rights reserved
//   
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1.  Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//  2.  Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//  3.  Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//

#include "curio_base/lx16a_encoder_filter_client.h"

#include "curio_base/EncoderFilterAdd.h"
#include "curio_base/EncoderFilterGetAngularPosition.h"
#include "curio_base/EncoderFilterGetCount.h"
#include "curio_base/EncoderFilterGetDuty.h"
#include "curio_base/EncoderFilterGetInvert.h"
#include "curio_base/EncoderFilterGetRevolutions.h"
#include "curio_base/EncoderFilterGetServoPos.h"
#include "curio_base/EncoderFilterReset.h"
#include "curio_base/EncoderFilterSetInvert.h"
#include "curio_base/EncoderFilterUpdate.h"

#include <ros/ros.h>

namespace curio_base
{
    LX16AEncoderFilterClient::~LX16AEncoderFilterClient()
    {
    }

    LX16AEncoderFilterClient::LX16AEncoderFilterClient(
        ros::NodeHandle &nh,
        int8_t servo_id,
        const std::string &classifier_filename,
        const std::string &regressor_filename,
        int16_t window) :
        nh_(nh),
        servo_id_(servo_id),
        classifier_filename_(classifier_filename),
        regressor_filename_(regressor_filename),
        window_(window)
    {
    }

    void LX16AEncoderFilterClient::init()
    {
        // Service clients
        filter_add_ = nh_.serviceClient<curio_base::EncoderFilterAdd>("lx16a/encoder_filter/add");
        filter_get_angular_position_ = nh_.serviceClient<curio_base::EncoderFilterGetAngularPosition>("lx16a/encoder_filter/get_angular_position"); 
        filter_get_count_ = nh_.serviceClient<curio_base::EncoderFilterGetCount>("lx16a/encoder_filter/get_count"); 
        filter_get_duty_ = nh_.serviceClient<curio_base::EncoderFilterGetDuty>("lx16a/encoder_filter/get_duty"); 
        filter_get_invert_ = nh_.serviceClient<curio_base::EncoderFilterGetInvert>("lx16a/encoder_filter/get_invert"); 
        filter_get_revolutions_ = nh_.serviceClient<curio_base::EncoderFilterGetRevolutions>("lx16a/encoder_filter/get_revolutions"); 
        filter_get_servo_pos_ = nh_.serviceClient<curio_base::EncoderFilterGetServoPos>("lx16a/encoder_filter/get_servo_pos"); 
        filter_reset_ = nh_.serviceClient<curio_base::EncoderFilterReset>("lx16a/encoder_filter/reset"); 
        filter_set_invert_ = nh_.serviceClient<curio_base::EncoderFilterSetInvert>("lx16a/encoder_filter/set_invert"); 
        filter_update_ = nh_.serviceClient<curio_base::EncoderFilterUpdate>("lx16a/encoder_filter/update"); 

        // Initialise
        curio_base::EncoderFilterAdd srv;
        srv.request.servo_id = servo_id_;
        srv.request.classifier_filename = classifier_filename_;
        srv.request.regressor_filename = regressor_filename_;
        srv.request.window = window_;
        if (filter_add_.call(srv))
        {
            ROS_INFO_STREAM("Initialised encoder filter: " << srv.response.status);
        }
        else
        {
            ROS_ERROR("Failed to initialise encoder filter");
        }
    }

    void LX16AEncoderFilterClient::update(const ros::Time &ros_time, int16_t duty, int16_t position)
    {
        auto&& service = filter_update_;

        curio_base::EncoderFilterUpdate srv;
        srv.request.servo_id = servo_id_;
        srv.request.time = ros_time;
        srv.request.duty = duty;
        srv.request.position = position;
        if (service.call(srv))
        {
            ROS_DEBUG_STREAM("Updated encoder filter: " << srv.response.status);
        }
        else
        {
            ROS_ERROR("Failed to update encoder filter");
        }
    }

    int16_t LX16AEncoderFilterClient::getRevolutions() const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_revolutions_);

        curio_base::EncoderFilterGetRevolutions srv;
        srv.request.servo_id = servo_id_;
        if (service.call(srv))
        {
            return srv.response.revolutions;
        }
        else
        {
            ROS_ERROR("Failed to get revolutions from encoder filter");
            return 0;
        }
    }

    int16_t LX16AEncoderFilterClient::getCount() const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_count_);

        curio_base::EncoderFilterGetCount srv;
        srv.request.servo_id = servo_id_;
        if (service.call(srv))
        {
            return srv.response.count;
        }
        else
        {
            ROS_ERROR("Failed to get count from encoder filter");
            return 0;
        }
    }

    int16_t LX16AEncoderFilterClient::getDuty() const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_duty_);

        curio_base::EncoderFilterGetDuty srv;
        srv.request.servo_id = servo_id_;
        if (service.call(srv))
        {
            return srv.response.duty;
        }
        else
        {
            ROS_ERROR("Failed to get duty from encoder filter");
            return 0;
        }
    }

    double LX16AEncoderFilterClient::getAngularPosition() const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_angular_position_);

        curio_base::EncoderFilterGetAngularPosition srv;
        srv.request.servo_id = servo_id_;
        if (service.call(srv))
        {
            return srv.response.position;
        }
        else
        {
            ROS_ERROR("Failed to get angular position from encoder filter");
            return 0.0;
        }
    }

    void LX16AEncoderFilterClient::getServoPosition(int16_t &position, bool &is_valid, bool map_position) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_servo_pos_);

        curio_base::EncoderFilterGetServoPos srv;
        srv.request.servo_id = servo_id_;
        if (service.call(srv))
        {
            position = srv.response.position;
            is_valid = srv.response.is_valid;
        }
        else
        {
            ROS_ERROR("Failed to get servo position from encoder filter");
        }
    }

    int16_t LX16AEncoderFilterClient::getInvert() const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_invert_);

        curio_base::EncoderFilterGetInvert srv;
        srv.request.servo_id = servo_id_;
        if (service.call(srv))
        {
            return srv.response.invert;
        }
        else
        {
            ROS_ERROR("Failed to get invert from encoder filter");
            return 1;
        }
    }

    void LX16AEncoderFilterClient::setInvert(bool is_inverted)
    {
        auto&& service = filter_set_invert_;

        curio_base::EncoderFilterSetInvert srv;
        srv.request.servo_id = servo_id_;
        srv.request.is_inverted = is_inverted;
        if (service.call(srv))
        {
            ROS_INFO_STREAM("Set invert on encoder filter: " << srv.response.status);
        }
        else
        {
            ROS_ERROR("Failed to set invert on encoder filter");
        }
    }

    void LX16AEncoderFilterClient::reset(int16_t position)
    {
        auto&& service = filter_reset_;

        curio_base::EncoderFilterUpdate srv;
        srv.request.servo_id = servo_id_;
        srv.request.position = position;
        if (service.call(srv))
        {
            ROS_INFO_STREAM("Reset encoder filter: " << srv.response.status);
        }
        else
        {
            ROS_ERROR("Failed to reset encoder filter");
        }
    }

} // namespace curio_base

