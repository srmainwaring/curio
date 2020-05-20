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
#include "lx16a/lx16a_exception.h"

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

#include <chrono>
#include <future>
#include <thread>

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

        // Wait until the service is ready...
        ros::Duration timeout_(10); // [s]
        bool is_ready = filter_add_v_.waitForExistence(timeout_);
        if (!is_ready)
        {
            throw LX16AException("Encoder service not available");
        }
    }

    void LX16AEncoderFilterClient::add(uint8_t servo_id)
    {
        lx16a_msgs::EncoderFilterAdd srv;
        srv.request.servo_id = servo_id;
        srv.request.classifier_filename = classifier_filename_;
        srv.request.regressor_filename = regressor_filename_;
        srv.request.window = window_;

        auto timeout = std::chrono::seconds(10);
        auto start_time = std::chrono::steady_clock::now();
        auto call_srv = [&]() {  return filter_add_.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder update: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder add: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            auto end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end_time - start_time;
            ROS_INFO_STREAM("Added encoder " << static_cast<int>(servo_id)
                << ": took " << elapsed_seconds.count() << "s");
        }
        else
        {
            throw LX16AException("Encoder add: service call failed");
        }
    }

    void LX16AEncoderFilterClient::update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position)
    {
        lx16a_msgs::EncoderFilterUpdate srv;
        srv.request.servo_id = servo_id;
        srv.request.time = ros_time;
        srv.request.duty = duty;
        srv.request.position = position;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return filter_update_.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder update failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder update failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (!fut.get())
        {
            throw LX16AException("Encoder update failed: invalid service call");
        }
    }

    int16_t LX16AEncoderFilterClient::getRevolutions(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_revolutions_);

        lx16a_msgs::EncoderFilterGetRevolutions srv;
        srv.request.servo_id = servo_id;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return service.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder revolutions failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder revolutions failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            return srv.response.revolutions;
        }
        else
        {
            throw LX16AException("Encoder revolutions failed: invalid service call");
        }
    }

    int16_t LX16AEncoderFilterClient::getCount(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_count_);

        lx16a_msgs::EncoderFilterGetCount srv;
        srv.request.servo_id = servo_id;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return service.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder count failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder count failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            return srv.response.count;
        }
        else
        {
            throw LX16AException("Encoder count failed: invalid service call");
        }
    }

    int16_t LX16AEncoderFilterClient::getDuty(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_duty_);

        lx16a_msgs::EncoderFilterGetDuty srv;
        srv.request.servo_id = servo_id;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return service.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder duty failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder duty failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            return srv.response.duty;
        }
        else
        {
            throw LX16AException("Encoder duty failed: invalid service call");
        }
    }

    double LX16AEncoderFilterClient::getAngularPosition(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_angular_position_);

        lx16a_msgs::EncoderFilterGetAngularPosition srv;
        srv.request.servo_id = servo_id;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return service.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder angular position failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder angular position failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            return srv.response.position;
        }
        else
        {
            throw LX16AException("Encoder angular position failed: invalid service call");
        }
    }

    void LX16AEncoderFilterClient::getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_servo_pos_);

        lx16a_msgs::EncoderFilterGetServoPos srv;
        srv.request.servo_id = servo_id;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return service.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder servo position failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder servo position failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            position = srv.response.position;
            is_valid = srv.response.is_valid;
        }
        else
        {
            throw LX16AException("Encoder servo position failed: invalid service call");
        }
    }

    int16_t LX16AEncoderFilterClient::getInvert(uint8_t servo_id) const
    {
        auto&& service = const_cast<ros::ServiceClient&>(filter_get_invert_);

        lx16a_msgs::EncoderFilterGetInvert srv;
        srv.request.servo_id = servo_id;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return service.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder invert failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder invert failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            return srv.response.invert;
        }
        else
        {
            throw LX16AException("Encoder invert failed: invalid service call");
        }
    }

    void LX16AEncoderFilterClient::setInvert(uint8_t servo_id, bool is_inverted)
    {
        lx16a_msgs::EncoderFilterSetInvert srv;
        srv.request.servo_id = servo_id;
        srv.request.is_inverted = is_inverted;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return filter_set_invert_.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder set invert failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder set invert failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (!fut.get())
        {
            throw LX16AException("Encoder set invert failed: invalid service call");
        }
    }

    void LX16AEncoderFilterClient::reset(uint8_t servo_id, int16_t position)
    {
        lx16a_msgs::EncoderFilterUpdate srv;
        srv.request.servo_id = servo_id;
        srv.request.position = position;

        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto call_srv = [&]() {  return filter_reset_.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder reset failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder reset failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (!fut.get())
        {
            throw LX16AException("Encoder reset failed: invalid service call");
        }
    }

    void LX16AEncoderFilterClient::add_v(const std::vector<uint8_t> &servo_ids)
    {   
        std::vector<int8_t> int8_servo_ids(servo_ids.size());
        std::copy(servo_ids.begin(), servo_ids.end(), int8_servo_ids.begin());

        lx16a_msgs::EncoderFilterAddV srv;
        srv.request.servo_ids = int8_servo_ids;
        srv.request.classifier_filename = classifier_filename_;
        srv.request.regressor_filename = regressor_filename_;
        srv.request.window = window_;
        
        auto timeout = std::chrono::seconds(10);
        auto start_time = std::chrono::steady_clock::now();
        auto call_srv = [&]() {  return filter_add_v_.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder add failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder add failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            auto end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end_time - start_time;
            ROS_INFO_STREAM("Adding encoders took " << elapsed_seconds.count() << "s");
        }
        else
        {
            throw LX16AException("Encoder add failed: invalid service call");
        }
    }

    void LX16AEncoderFilterClient::update_v(
        const std::vector<uint8_t> &servo_ids,
        const ros::Time &ros_time,
        const std::vector<int16_t> &duties,
        const std::vector<int16_t> &positions,
        std::vector<double> &angular_positions)
    {
        std::vector<int8_t> int8_servo_ids(servo_ids.size());
        std::copy(servo_ids.begin(), servo_ids.end(), int8_servo_ids.begin());

        lx16a_msgs::EncoderFilterUpdateV srv;
        srv.request.servo_ids = int8_servo_ids;
        srv.request.time = ros_time;
        srv.request.duties = duties;
        srv.request.positions = positions;

        // Asynchronos call_srv returns true if the call succeeds, false otherwise.        
        auto timeout = std::chrono::nanoseconds(timeout_.toNSec());
        auto start_time = std::chrono::steady_clock::now();
        auto call_srv = [&]() {  return filter_update_v_.call(srv); };
        std::future<bool> fut = std::async(std::launch::async, call_srv);
        std::future_status status;
        do
        {
            if (!fut.valid())
            {
                throw LX16AException("Encoder update failed: future_errc::no_state");
            }
            status = fut.wait_for(timeout);
            if (status == std::future_status::timeout)
            {
                throw LX16AException("Encoder update failed: timeout");
            }
        } while (status != std::future_status::ready);

        if (fut.get())
        {
            auto end_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = end_time - start_time;
            ROS_DEBUG_STREAM("Encoder update took " << elapsed_seconds.count() << "s");
            std::copy(
                srv.response.angular_positions.begin(),
                srv.response.angular_positions.end(),
                angular_positions.begin());                
        }
        else
        {
            throw LX16AException("Encoder update failed: invalid service call");
        }
    }

} // namespace lx16a

