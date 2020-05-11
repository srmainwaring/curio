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

#include "lx16a/lx16a_encoder_filter_client.h"

#include "lx16a_msgs/EncoderFilterAdd.h"
#include "lx16a_msgs/EncoderFilterGetAngularPosition.h"
#include "lx16a_msgs/EncoderFilterGetCount.h"
#include "lx16a_msgs/EncoderFilterGetDuty.h"
#include "lx16a_msgs/EncoderFilterGetInvert.h"
#include "lx16a_msgs/EncoderFilterGetRevolutions.h"
#include "lx16a_msgs/EncoderFilterGetServoPos.h"
#include "lx16a_msgs/EncoderFilterReset.h"
#include "lx16a_msgs/EncoderFilterSetInvert.h"
#include "lx16a_msgs/EncoderFilterUpdate.h"

#include "lx16a_msgs/EncoderFilterAddV.h"
#include "lx16a_msgs/EncoderFilterUpdateV.h"

#include <ros/ros.h>

namespace lx16a
{
    LX16AEncoderFilterClient::~LX16AEncoderFilterClient()
    {
    }

    LX16AEncoderFilterClient::LX16AEncoderFilterClient(
        ros::NodeHandle &nh,
        const std::string &classifier_filename,
        const std::string &regressor_filename,
        int16_t window) :
        nh_(nh),
        classifier_filename_(classifier_filename),
        regressor_filename_(regressor_filename),
        window_(window)
    {
    }

    void LX16AEncoderFilterClient::init()
    {
        // Service clients
        filter_add_ = nh_.serviceClient<lx16a_msgs::EncoderFilterAdd>("lx16a/encoder_filter/add");
        filter_get_angular_position_ = nh_.serviceClient<lx16a_msgs::EncoderFilterGetAngularPosition>("lx16a/encoder_filter/get_angular_position"); 
        filter_get_count_ = nh_.serviceClient<lx16a_msgs::EncoderFilterGetCount>("lx16a/encoder_filter/get_count"); 
        filter_get_duty_ = nh_.serviceClient<lx16a_msgs::EncoderFilterGetDuty>("lx16a/encoder_filter/get_duty"); 
        filter_get_invert_ = nh_.serviceClient<lx16a_msgs::EncoderFilterGetInvert>("lx16a/encoder_filter/get_invert"); 
        filter_get_revolutions_ = nh_.serviceClient<lx16a_msgs::EncoderFilterGetRevolutions>("lx16a/encoder_filter/get_revolutions"); 
        filter_get_servo_pos_ = nh_.serviceClient<lx16a_msgs::EncoderFilterGetServoPos>("lx16a/encoder_filter/get_servo_pos"); 
        filter_reset_ = nh_.serviceClient<lx16a_msgs::EncoderFilterReset>("lx16a/encoder_filter/reset"); 
        filter_set_invert_ = nh_.serviceClient<lx16a_msgs::EncoderFilterSetInvert>("lx16a/encoder_filter/set_invert"); 
        filter_update_ = nh_.serviceClient<lx16a_msgs::EncoderFilterUpdate>("lx16a/encoder_filter/update"); 

        // Vectorised service clients
        filter_add_v_ = nh_.serviceClient<lx16a_msgs::EncoderFilterAddV>("lx16a/encoder_filter/add_v");
        filter_update_v_ = nh_.serviceClient<lx16a_msgs::EncoderFilterUpdateV>("lx16a/encoder_filter/update_v"); 

        // Block until the service is ready...
        ros::Duration timeout_(2); // [s]
        bool is_ready = filter_add_.waitForExistence(timeout_);
        if (!is_ready)
        {
            ROS_ERROR("Failed to initialise encoder filter - service not available");
        }
    }

    void LX16AEncoderFilterClient::add(uint8_t servo_id)
    {
        auto&& service = filter_add_;

        lx16a_msgs::EncoderFilterAdd srv;
        srv.request.servo_id = servo_id;
        srv.request.classifier_filename = classifier_filename_;
        srv.request.regressor_filename = regressor_filename_;
        srv.request.window = window_;
        if (service.call(srv))
        {
            ROS_INFO_STREAM("Added encoder filter: " << (srv.response.status ? "ERROR" : "OK"));
        }
        else
        {
            ROS_ERROR_STREAM("Failed to add encoder filter for servo: "
                << static_cast<int>(servo_id));
        }
    }

    void LX16AEncoderFilterClient::update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position)
    {
        auto&& service = filter_update_;

        lx16a_msgs::EncoderFilterUpdate srv;
        srv.request.servo_id = servo_id;
        srv.request.time = ros_time;
        srv.request.duty = duty;
        srv.request.position = position;
        if (service.call(srv))
        {
            ROS_DEBUG_STREAM("Updated encoder filter: " << (srv.response.status ? "ERROR" : "OK"));
        }
        else
        {
            ROS_ERROR_STREAM("Failed to update encoder filter for servo: "
                << static_cast<int>(servo_id));
        }
    }

    int16_t LX16AEncoderFilterClient::getRevolutions(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_revolutions_);

        lx16a_msgs::EncoderFilterGetRevolutions srv;
        srv.request.servo_id = servo_id;
        if (service.call(srv))
        {
            return srv.response.revolutions;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to get revolutions from encoder filter for servo: "
                << static_cast<int>(servo_id));
            return 0;
        }
    }

    int16_t LX16AEncoderFilterClient::getCount(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_count_);

        lx16a_msgs::EncoderFilterGetCount srv;
        srv.request.servo_id = servo_id;
        if (service.call(srv))
        {
            return srv.response.count;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to get count from encoder filter for servo: "
                << static_cast<int>(servo_id));
            return 0;
        }
    }

    int16_t LX16AEncoderFilterClient::getDuty(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_duty_);

        lx16a_msgs::EncoderFilterGetDuty srv;
        srv.request.servo_id = servo_id;
        if (service.call(srv))
        {
            return srv.response.duty;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to get duty from encoder filter for servo: "
                << static_cast<int>(servo_id));
            return 0;
        }
    }

    double LX16AEncoderFilterClient::getAngularPosition(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_angular_position_);

        lx16a_msgs::EncoderFilterGetAngularPosition srv;
        srv.request.servo_id = servo_id;
        if (service.call(srv))
        {
            return srv.response.position;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to get angular position from encoder filter for servo: "
                << static_cast<int>(servo_id));
            return 0.0;
        }
    }

    void LX16AEncoderFilterClient::getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_servo_pos_);

        lx16a_msgs::EncoderFilterGetServoPos srv;
        srv.request.servo_id = servo_id;
        if (service.call(srv))
        {
            position = srv.response.position;
            is_valid = srv.response.is_valid;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to get servo position from encoder filter for servo: "
                << static_cast<int>(servo_id));
        }
    }

    int16_t LX16AEncoderFilterClient::getInvert(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_invert_);

        lx16a_msgs::EncoderFilterGetInvert srv;
        srv.request.servo_id = servo_id;
        if (service.call(srv))
        {
            return srv.response.invert;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to get invert from encoder filter for servo: "
                << static_cast<int>(servo_id));
            return 1;
        }
    }

    void LX16AEncoderFilterClient::setInvert(uint8_t servo_id, bool is_inverted)
    {
        auto&& service = filter_set_invert_;

        lx16a_msgs::EncoderFilterSetInvert srv;
        srv.request.servo_id = servo_id;
        srv.request.is_inverted = is_inverted;
        if (service.call(srv))
        {
            ROS_INFO_STREAM("Set invert on encoder filter: " << (srv.response.status ? "ERROR" : "OK"));
        }
        else
        {
            ROS_ERROR_STREAM("Failed to set invert on encoder filter for servo: "
                << static_cast<int>(servo_id));
        }
    }

    void LX16AEncoderFilterClient::reset(uint8_t servo_id, int16_t position)
    {
        auto&& service = filter_reset_;

        lx16a_msgs::EncoderFilterUpdate srv;
        srv.request.servo_id = servo_id;
        srv.request.position = position;
        if (service.call(srv))
        {
            ROS_INFO_STREAM("Reset encoder filter: " << (srv.response.status ? "ERROR" : "OK"));
        }
        else
        {
            ROS_ERROR_STREAM("Failed to reset encoder filter for servo: "
                << static_cast<int>(servo_id));
        }
    }

    void LX16AEncoderFilterClient::add_v(const std::vector<uint8_t> &servo_ids)
    {   
        auto&& service = filter_add_v_;

        std::vector<int8_t> int8_servo_ids(servo_ids.size());
        std::copy(servo_ids.begin(), servo_ids.end(), int8_servo_ids.begin());

        lx16a_msgs::EncoderFilterAddV srv;
        srv.request.servo_ids = int8_servo_ids;
        srv.request.classifier_filename = classifier_filename_;
        srv.request.regressor_filename = regressor_filename_;
        srv.request.window = window_;
        if (service.call(srv))
        {
            ROS_INFO_STREAM("Added encoder filter: " << (srv.response.status ? "ERROR" : "OK"));
        }
        else
        {
            ROS_ERROR_STREAM("Failed to add encoder filter for servos");
        }
    }

    void LX16AEncoderFilterClient::update_v(
        const std::vector<uint8_t> &servo_ids,
        const ros::Time &ros_time,
        const std::vector<int16_t> &duties,
        const std::vector<int16_t> &positions,
        std::vector<double> &angular_positions)
    {
        auto&& service = filter_update_v_;

        std::vector<int8_t> int8_servo_ids(servo_ids.size());
        std::copy(servo_ids.begin(), servo_ids.end(), int8_servo_ids.begin());

        lx16a_msgs::EncoderFilterUpdateV srv;
        srv.request.servo_ids = int8_servo_ids;
        srv.request.time = ros_time;
        srv.request.duties = duties;
        srv.request.positions = positions;
        if (service.call(srv))
        {
            ROS_DEBUG_STREAM("Updated encoder filter");
            std::copy(
                srv.response.angular_positions.begin(),
                srv.response.angular_positions.end(),
                angular_positions.begin());

        }
        else
        {
            ROS_ERROR_STREAM("Failed to update encoder filter");
        }
    }


} // namespace lx16a

