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

/// \brief Port of lx16a_encoder_filter.py to C++.

#ifndef LX16A_ENCODER_FILTER_ONNX_H_
#define LX16A_ENCODER_FILTER_ONNX_H_

#include "lx16a/lx16a_encoder_filter.h"

#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <string>

namespace lx16a
{
    class LX16AEncoderFilterOnnxImpl;

    /// \copydoc LX16AEncoderFilter.
    class LX16AEncoderFilterOnnx : public LX16AEncoderFilter
    {
    public:
        /// \copydoc LX16AEncoderFilter::~LX16AEncoderFilter()
        virtual ~LX16AEncoderFilterOnnx();

        /// \brief Constructor
        ///
        /// \param classifier_filename A std::string containing the file
        /// name of the scikit-learn decision tree classifier
        /// \param regressor_filename A std::string containing the file
        /// name of the scikit-learn decision tree regressor,
        /// has (default None)
        /// \param window An integer size of the sample window used in
        /// the classifier, has default 10)
        ///
        LX16AEncoderFilterOnnx(
            const std::string &classifier_filename,
            const std::string &regressor_filename = "",
            int16_t window = 10);

        /// \copydoc LX16AEncoderFilter::init()
        void init() override;

        /// \copydoc LX16AEncoderFilter::add()
        void add(uint8_t servo_id) override;

        /// \copydoc LX16AEncoderFilter::update()
        void update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position) override;

        /// \copydoc LX16AEncoderFilter::getRevolutions()
        int16_t getRevolutions(uint8_t servo_id) const override;

        /// \copydoc LX16AEncoderFilter::getCount()
        int16_t getCount(uint8_t servo_id) const override;

        /// \copydoc LX16AEncoderFilter::getDuty()
        int16_t getDuty(uint8_t servo_id) const override;

        /// \copydoc LX16AEncoderFilter::getAngularPosition()
        double getAngularPosition(uint8_t servo_id) const override;

        /// \copydoc LX16AEncoderFilter::getServoPosition()
        void getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position=true) const override;

        /// \copydoc LX16AEncoderFilter::getInvert()
        int16_t getInvert(uint8_t servo_id) const override;

        /// \copydoc LX16AEncoderFilter::setInvert()
        void setInvert(uint8_t servo_id, bool is_inverted) override;

        /// \copydoc LX16AEncoderFilter::reset()
        void reset(uint8_t servo_id, int16_t position) override;

        /// Vectorised interface

        /// \copydoc LX16AEncoderFilter::add_v()
        void add_v(const std::vector<uint8_t> &servo_ids) override;

        /// \copydoc LX16AEncoderFilter::update_v()
        void update_v(
            const std::vector<uint8_t> &servo_ids,
            const ros::Time &ros_time,
            const std::vector<int16_t> &duties,
            const std::vector<int16_t> &positions,
            std::vector<double> &angular_positions) override;
    };

} // namespace lx16a

#endif // LX16A_ENCODER_FILTER_ONNX_H_
