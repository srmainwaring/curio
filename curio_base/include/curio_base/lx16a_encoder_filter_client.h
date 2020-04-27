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

#ifndef CURIO_BASE_LX16A_ENCODER_FILTER_CLIENT_H_
#define CURIO_BASE_LX16A_ENCODER_FILTER_CLIENT_H_

#include "curio_base/lx16a_encoder_filter.h"

#include <ros/ros.h>

#include <cstdint>
#include <string>

namespace curio_base
{
    /// \brief An encoder filter for the LX-16A servo.
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
    class LX16AEncoderFilterClient : public LX16AEncoderFilter
    {
    public:
        /// \brief Destructor
        virtual ~LX16AEncoderFilterClient();

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
        LX16AEncoderFilterClient(
            ros::NodeHandle &nh,
            int8_t servo_id,
            const std::string &classifier_filename,
            const std::string &regressor_filename = "",
            int16_t window = 10);

        /// \brief Initialise the encoder filter 
        void init() override;

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
        void update(const ros::Time &ros_time, int16_t duty, int16_t position) override;

        /// \brief Get the number of revoutions since reset.
        ///
        /// \return An integer number of revolutions since the count was reset.
        ///
        int16_t getRevolutions() const override;

        /// \brief Get the current encoder count since reset (filtered).
        ///
        /// Note that the encoder count is offset from the servo position
        /// so that the count is zero when the encoder filter is reset.
        ///
        /// \return An integer, the current encoder count.
        ///
        int16_t getCount() const override;

        /// \brief Get the current encoder duty.
        ///
        /// \return An integer, the current encoder duty.
        ///
        int16_t getDuty() const override;

        /// Get the angular position of the encoder (filtered)
        ///
        /// \return A double, the angular position of the encoder [rad].
        ///
        double getAngularPosition() const override;

        /// \brief Get the current (un-filtered) servo position and an
        /// estimate whether it is valid.
        ///
        /// \param position An integer reference that is set to the current position. 
        /// \param is_valid A bool that is set to true for valid, false otherwise.
        /// \param map_position A bool: if true map the position to the
        /// range [0, 1500],  has (default True)
        ///
        void getServoPosition(int16_t &position, bool &is_valid, bool map_position=true) const override;

        /// \brief Get the invert state: whether the encoder count is inverted.
        ///
        /// \return An integer: -1 if the count is inverted, 1 otherwise. 
        ///
        int16_t getInvert() const override;

        /// \brief Invert the direction of the encoder count.
        ///
        /// \param is_inverted A bool: true if the encoder count is reversed.
        ///
        void setInvert(bool is_inverted) override;

        /// Reset the encoder counters to zero.
        ///
        /// \param position An integer (assumed valid) position of the
        /// servo when the encoder is reset.  
        /// 
        void reset(int16_t position) override;

    private:
        ros::NodeHandle  nh_;
        int8_t servo_id_;                       // servo_id for this filter
        std::string classifier_filename_;       // classifier filename
        std::string regressor_filename_;        // regressor filename
        int16_t window_ = 10;                   // size of the history window

        // Service clients (proxies)
        ros::ServiceClient filter_add_;
        ros::ServiceClient filter_get_angular_position_;
        ros::ServiceClient filter_get_count_;
        ros::ServiceClient filter_get_duty_;
        ros::ServiceClient filter_get_invert_;
        ros::ServiceClient filter_get_revolutions_;
        ros::ServiceClient filter_get_servo_pos_;
        ros::ServiceClient filter_reset_;
        ros::ServiceClient filter_set_invert_;
        ros::ServiceClient filter_update_;
    };

} // namespace curio_base

#endif // CURIO_BASE_LX16A_ENCODER_FILTER_CLIENT_H_
