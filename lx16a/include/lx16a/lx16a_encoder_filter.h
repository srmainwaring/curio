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

/// \brief Interface of lx16a_encoder_filter.py to C++.

#ifndef LX16A_ENCODER_FILTER_H_
#define LX16A_ENCODER_FILTER_H_

#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <string>

namespace lx16a
{
    /// \brief An encoder filter interface for the LX-16A servo.
    ///
    /// This class loads a `scikit-learn` decision tree classifier
    /// which is used to predict whether or not a position
    /// obtained from a LX-16A servo lies within its valid measurement
    /// region which covers a range of about 330 deg.
    ///
    /// The LX-16A has 1500 counts per revolution giving an
    /// angular resolution of 0.24 deg.
    ///
    /// The class maintains a count of the number of full revolutions
    /// made by the servo (positive and negative) and uses this and
    /// the servo postion to determine the overall encoder count.
    ///
    /// There is also an optional facilty to estimate the servo position
    /// in the invalid region using a decision tree regressor. This is
    /// enabled by supplying the constructor with a filename for the
    /// regressor model. 
    ///
    class LX16AEncoderFilter
    {
    public:
        /// \brief Destructor
        virtual ~LX16AEncoderFilter();

        /// \brief Initialise the encoder filter.
        virtual void init() = 0;

        /// \brief Add a servo.
        virtual void add(uint8_t servo_id) = 0;

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
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param ros_time A ros::Time containing the time from ros::Time::now()
        /// \param duty An integer servo duty
        /// \param position An integer servo position        
        ///
        virtual void update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position) = 0;

        /// \brief Get the number of revoutions since reset.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \return An integer number of revolutions since the count was reset.
        ///
        virtual int16_t getRevolutions(uint8_t servo_id) const = 0;

        /// \brief Get the current encoder count since reset (filtered).
        ///
        /// Note that the encoder count is offset from the servo position
        /// so that the count is zero when the encoder filter is reset.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \return An integer, the current encoder count.
        ///
        virtual int16_t getCount(uint8_t servo_id) const = 0;

        /// \brief Get the current encoder duty.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \return An integer, the current encoder duty.
        ///
        virtual int16_t getDuty(uint8_t servo_id) const = 0;

        /// Get the angular position of the encoder (filtered)
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \return A double, the angular position of the encoder [rad].
        ///
        virtual double getAngularPosition(uint8_t servo_id) const = 0;

        /// \brief Get the current (un-filtered) servo position and an
        /// estimate whether it is valid.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param position An integer reference that is set to the current position. 
        /// \param is_valid A bool that is set to true for valid, false otherwise.
        /// \param map_position A bool: if true map the position to the
        /// range [0, 1500],  has (default True)
        ///
        virtual void getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position=true) const = 0;

        /// \brief Get the invert state: whether the encoder count is inverted.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \return An integer: -1 if the count is inverted, 1 otherwise. 
        ///
        virtual int16_t getInvert(uint8_t servo_id) const = 0;

        /// \brief Invert the direction of the encoder count.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param is_inverted A bool: true if the encoder count is reversed.
        ///
        virtual void setInvert(uint8_t servo_id, bool is_inverted) = 0;

        /// Reset the encoder counters to zero.
        ///
        /// \param servo_id An integer for servo serial identifier, in [0, 253]
        /// \param position An integer (assumed valid) position of the
        /// servo when the encoder is reset.  
        /// 
        virtual void reset(uint8_t servo_id, int16_t position) = 0;
    };

} // namespace lx16a

#endif // LX16A_ENCODER_FILTER_H_
