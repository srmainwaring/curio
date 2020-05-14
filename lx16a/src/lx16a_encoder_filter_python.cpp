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

#include "lx16a/lx16a_encoder_filter_python.h"

#include <pybind11/pybind11.h>
#include <ros/ros.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace lx16a
{
    LX16AEncoderFilterPython::~LX16AEncoderFilterPython()
    {
    }

    LX16AEncoderFilterPython::LX16AEncoderFilterPython(
        const std::string &classifier_filename,
        const std::string &regressor_filename,
        int16_t window) :
        classifier_filename_(classifier_filename),
        regressor_filename_(regressor_filename),
        window_(window)
    {
    }

    void LX16AEncoderFilterPython::init()
    {
        // Import derived class and cast from Python to C++.
        py_module_ = py::module::import("lx16a.lx16a_encoder_filter_wrap").attr("LX16AEncoderFilterWrap");
        py_encoder_filter_ = py_module_(
            "classifier_filename"_a = classifier_filename_,
            "regressor_filename"_a = regressor_filename_,
            "window"_a = window_
        );
        encoder_filter_ = py_encoder_filter_.cast<LX16AEncoderFilter*>();
        encoder_filter_->init();
    }

    void LX16AEncoderFilterPython::add(uint8_t servo_id)
    {   
        encoder_filter_->add(11);
    }

    void LX16AEncoderFilterPython::update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position)
    {
        encoder_filter_->update(servo_id, ros_time, duty, position);
    }

    int16_t LX16AEncoderFilterPython::getRevolutions(uint8_t servo_id) const
    {
        return encoder_filter_->getRevolutions(servo_id);
    }

    int16_t LX16AEncoderFilterPython::getCount(uint8_t servo_id) const
    {
        return encoder_filter_->getCount(servo_id);
    }

    int16_t LX16AEncoderFilterPython::getDuty(uint8_t servo_id) const
    {
        return encoder_filter_->getDuty(servo_id);
    }

    double LX16AEncoderFilterPython::getAngularPosition(uint8_t servo_id) const
    {
        return encoder_filter_->getAngularPosition(servo_id);
    }

    void LX16AEncoderFilterPython::getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position) const
    {
        encoder_filter_->getServoPosition(servo_id, position, is_valid, map_position);
    }

    int16_t LX16AEncoderFilterPython::getInvert(uint8_t servo_id) const
    {
        return encoder_filter_->getInvert(servo_id);
    }

    void LX16AEncoderFilterPython::setInvert(uint8_t servo_id, bool is_inverted)
    {
        encoder_filter_->setInvert(servo_id, is_inverted);
    }

    void LX16AEncoderFilterPython::reset(uint8_t servo_id, int16_t position)
    {
        encoder_filter_->reset(servo_id, position);
    }

    void LX16AEncoderFilterPython::add_v(const std::vector<uint8_t> &servo_ids)
    {   
        encoder_filter_->add_v(servo_ids);
    }

    void LX16AEncoderFilterPython::update_v(
        const std::vector<uint8_t> &servo_ids,
        const ros::Time &ros_time,
        const std::vector<int16_t> &duties,
        const std::vector<int16_t> &positions,
        std::vector<double> &angular_positions)
    {
        encoder_filter_->update_v(servo_ids, ros_time, duties, positions, angular_positions);
    }


} // namespace lx16a

