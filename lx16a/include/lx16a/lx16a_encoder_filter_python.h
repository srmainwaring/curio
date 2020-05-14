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

#ifndef LX16A_ENCODER_FILTER_PYTHON_H_
#define LX16A_ENCODER_FILTER_PYTHON_H_

#include "lx16a/lx16a_encoder_filter.h"

#include <pybind11/pybind11.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>

namespace lx16a
{
    /// \brief An encoder filter for the LX-16A servo.
    class LX16AEncoderFilterPython : public LX16AEncoderFilter
    {
    public:
        /// \brief Destructor
        virtual ~LX16AEncoderFilterPython();

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
        LX16AEncoderFilterPython(
            const std::string &classifier_filename,
            const std::string &regressor_filename = "",
            int16_t window = 10);

        /// \brief Initialise the encoder filter.
        void init() override;

        /// \brief Add a servo.
        void add(uint8_t servo_id) override;

        /// \brief Update the encoder filter.
        ///
        /// Update the encoder and estimate whether or not the new servo
        /// position is in the valid range.
        ///
        /// The feature vector X contains 3 * window entries:
        ///     dt[window]     the change in ros_time between servo
        ///                    position readings
        ///     duty[window]   the commanded duty to the LX-16A
        ///     pos[window]    the measured position on the LX-16A
        ///        
        /// \param ros_time A ros::Time containing the time from ros::Time::now()
        /// \param duty An integer servo duty
        /// \param position An integer servo position        
        ///
        void update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position) override;

        /// \brief Get the number of revoutions since reset.
        ///
        /// \return An integer number of revolutions since the count was reset.
        ///
        int16_t getRevolutions(uint8_t servo_id) const override;

        /// \brief Get the current encoder count since reset (filtered).
        ///
        /// Note that the encoder count is offset from the servo position
        /// so that the count is zero when the encoder filter is reset.
        ///
        /// \return An integer, the current encoder count.
        ///
        int16_t getCount(uint8_t servo_id) const override;

        /// \brief Get the current encoder duty.
        ///
        /// \return An integer, the current encoder duty.
        ///
        int16_t getDuty(uint8_t servo_id) const override;

        /// Get the angular position of the encoder (filtered)
        ///
        /// \return A double, the angular position of the encoder [rad].
        ///
        double getAngularPosition(uint8_t servo_id) const override;

        /// \brief Get the current (un-filtered) servo position and an
        /// estimate whether it is valid.
        ///
        /// \param position An integer reference that is set to the current position. 
        /// \param is_valid A bool that is set to true for valid, false otherwise.
        /// \param map_position A bool: if true map the position to the
        /// range [0, 1500],  has (default True)
        ///
        void getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position=true) const override;

        /// \brief Get the invert state: whether the encoder count is inverted.
        ///
        /// \return An integer: -1 if the count is inverted, 1 otherwise. 
        ///
        int16_t getInvert(uint8_t servo_id) const override;

        /// \brief Invert the direction of the encoder count.
        ///
        /// \param is_inverted A bool: true if the encoder count is reversed.
        ///
        void setInvert(uint8_t servo_id, bool is_inverted) override;

        /// Reset the encoder counters to zero.
        ///
        /// \param position An integer (assumed valid) position of the
        /// servo when the encoder is reset.  
        /// 
        void reset(uint8_t servo_id, int16_t position) override;

        /// Vectorised interface

        /// \brief Add servos.
        void add_v(const std::vector<uint8_t> &servo_ids) override;

        /// \brief Update the encoder filter.
        void update_v(
            const std::vector<uint8_t> &servo_ids,
            const ros::Time &ros_time,
            const std::vector<int16_t> &duties,
            const std::vector<int16_t> &positions,
            std::vector<double> &angular_positions) override;

    private:
        std::string classifier_filename_;       // classifier filename
        std::string regressor_filename_;        // regressor filename
        int16_t window_ = 10;                   // size of the history window

        // Store the Python module and class instance objects as well
        // as a pointer to the C++ type. If the Python objects go out of
        // scope the pointer is undefined.   
        pybind11::object py_module_;
        pybind11::object py_encoder_filter_;
        LX16AEncoderFilter *encoder_filter_ = nullptr;
    };

} // namespace lx16a

#endif // LX16A_ENCODER_FILTER_PYTHON_H_
