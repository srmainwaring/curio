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

#ifndef CURIO_BASE_LX16A_ENCODER_FILTER_H_
#define CURIO_BASE_LX16A_ENCODER_FILTER_H_

#include <ros/ros.h>

#include <cmath>
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
    class LX16AEncoderFilter
    {
    public:
        /// \brief Destructor
        ~LX16AEncoderFilter();

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
        LX16AEncoderFilter(
            const std::string &classifier_filename,
            const std::string &regressor_filename = "",
            int16_t window = 10);

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
        void update(const ros::Time &ros_time, int16_t duty, int16_t position);

        /// \brief Get the number of revoutions since reset.
        ///
        /// \return An integer number of revolutions since the count was reset.
        ///
        int16_t getRevolutions() const;

        /// \brief Get the current encoder count since reset (filtered).
        ///
        /// Note that the encoder count is offset from the servo position
        /// so that the count is zero when the encoder filter is reset.
        ///
        /// \return An integer, the current encoder count.
        ///
        int16_t getCount() const;

        /// \brief Get the current encoder duty.
        ///
        /// \return An integer, the current encoder duty.
        ///
        int16_t getDuty() const;

        /// Get the angular position of the encoder (filtered)
        ///
        /// \return A double, the angular position of the encoder [rad].
        ///
        double getAngularPosition() const;

        /// \brief Get the current (un-filtered) servo position and an
        /// estimate whether it is valid.
        ///
        /// \param position An integer reference that is set to the current position. 
        /// \param is_valid A bool that is set to true for valid, false otherwise.
        /// \param map_position A bool: if true map the position to the
        /// range [0, 1500],  has (default True)
        ///
        void getServoPosition(int16_t &position, bool &is_valid, bool map_position=true) const;

        /// \brief Get the invert state: whether the encoder count is inverted.
        ///
        /// \return An integer: -1 if the count is inverted, 1 otherwise. 
        ///
        int16_t getInvert() const;

        /// \brief Invert the direction of the encoder count.
        ///
        /// \param is_inverted A bool: true if the encoder count is reversed.
        ///
        void setInvert(bool is_inverted);

        /// Reset the encoder counters to zero.
        ///
        /// \param position An integer (assumed valid) position of the
        /// servo when the encoder is reset.  
        /// 
        void reset(int16_t position);

    private:
        const int16_t ENCODER_MIN    = 0;       // minimum servo position reading
        const int16_t ENCODER_MAX    = 1500;    // maximum servo position reading
        const int16_t ENCODER_LOWER  = 1190;    // lower bound of the invalid range
        const int16_t ENCODER_UPPER  = 1310;    // upper bound of the invalid range
        const int16_t ENCODER_STEP   = 1000;    // threshold for determining the encoder has
                                                // completed a revolution

        // Initialise ring buffers that store the encoder history.
        int16_t window_ = 10;
        int16_t index_  = 0;                    // index for the ring buffers
        std::vector<ros::Time> ros_time_;       // [rospy.Time() for x in range(window)]
        std::vector<int16_t> duty_;             // [0.0 for x in range(window)]
        std::vector<int16_t> position_;         // [0.0 for x in range(window)]
        std::vector<int16_t> X_;                // [0.0 for x in range(3 * window)]
        std::string classifier_filename_;       // classifier filename
        std::string regressor_filename_;        // regressor filename
        //classifier_     = None                // scikit-learn classifier
        //regressor_      = None                // scikit-learn regressor
        int16_t count_offset_   = 0;            // set to ensure count=0 when reset
        int16_t revolutions_    = 0;            // number of revolutions since reset
        int16_t prev_valid_position_ = 0;       // the previous valid position
        int16_t invert_         = 1;            // 1 or -1 depending on the desired
                                                // direction for increasing count

        /// \brief Load classifier
        void loadClassifier();

        /// \brief Load regressor
        void loadRegressor();
    };

} // namespace curio_base

#endif // CURIO_BASE_LX16A_ENCODER_FILTER_H_