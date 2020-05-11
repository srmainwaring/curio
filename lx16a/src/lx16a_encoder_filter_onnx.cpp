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

#include "lx16a/lx16a_encoder_filter_onnx.h"
#include <cmath>

namespace lx16a
{
////////////////////////////////////////////////////////////////////////
// LX16AEncoderFilterOnnxImpl

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
    class LX16AEncoderFilterOnnxImpl
    {
    public:
        /// \brief Destructor
        ~LX16AEncoderFilterOnnxImpl()
        {
        }

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
        LX16AEncoderFilterOnnxImpl(
            const std::string &classifier_filename,
            const std::string &regressor_filename,
            int16_t window) :
            window_(window),
            ros_time_(window, ros::Time()),
            duty_(window, 0), 
            position_(window, 0),
            X_(window, 0),
            classifier_filename_(classifier_filename),
            regressor_filename_(regressor_filename)
        {
        }

        /// \brief Initialise the encoder filter 
        void init(uint8_t servo_id)
        {
            // Load the ML classifier pipeline
            loadClassifier();

            // Load the ML regressor pipeline
            loadRegressor();
        }

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
        void update(const ros::Time &ros_time, int16_t duty, int16_t position)
        {
            // Update the ring buffers
            index_ = (index_ + 1) % window_;
            ros_time_[index_] = ros_time;
            duty_[index_] = duty;
            position_[index_] = position;
                    
            // times
            for (int16_t i=0; i<window_; ++i)
            {
                int16_t idx = (index_ - i) % window_;
                double dt = (ros_time_[idx] - ros_time_[index_]).toSec();
                X_[i] = dt;
            }

            // duty 
            for (int16_t i=0; i<window_; ++i)
            {
                int16_t idx = (index_ - i);
                int16_t duty_i = duty_[idx];
                X_[window_ + i] = duty_i;
            }

            // positions
            for (int16_t i=0; i<window_; ++i)
            {
                int16_t idx = (index_ - i);
                int16_t pos_i = position_[idx];
                X_[2 * window_ + i] = pos_i;
            }

            // @TODO: this section would benefit from restructuring,
            //        once the regression logic has been finalised.
            //   
            // Apply the filter and update the encoder counters
            int16_t pos = position_[index_] % ENCODER_MAX;
            bool is_valid = true; // = self._classifier.predict([self._X])[0]
            if (is_valid)
            {
                // If the absolute change in the servo position is 
                // greater than ENCODER_STEP then we increment / decrement
                // the revolution counter.
                int16_t delta = pos - prev_valid_position_;
                if (delta > ENCODER_STEP)
                {
                    revolutions_ -= 1;
                }
                if (delta < -ENCODER_STEP)
                {
                    revolutions_ +=  1;
                }

                // Update the previous valid position
                prev_valid_position_ = pos;
            }
            else if(true /*regressor_ != nullptr*/)
            {
                // Not valid - so we'll try to use the regressor to predict
                // a value for the encoder count.
                int16_t pos_est = position; //static_cast<int16_t>(
                    //regressor_.predict([self.X_])[0]) % ENCODER_MAX);

                // @DEBUG_INFO
                int16_t count_est = pos_est + ENCODER_MAX * revolutions_; 
                ROS_DEBUG_STREAM("count_est: {}" << count_est);

                // @TODO: the acceptance criteria may need further tuning.
                // The classifier will sometimes report false positives.
                // To limit the impact of using a regressed value in this
                // case we check that the previous position is 'close' to
                // one of the boundaries. 
                int16_t dist1 = std::abs(ENCODER_LOWER - prev_valid_position_);
                int16_t dist2 = std::abs(ENCODER_UPPER - prev_valid_position_);
                int16_t dist  = std::min(dist1, dist2);
                int16_t DIST_MAX = (ENCODER_UPPER - ENCODER_LOWER)/2 + 5;

                // Accept the estimated position if in the range where the
                // encoder does not report valid values: [1190, 1310]
                if (dist < DIST_MAX 
                    && pos_est >= ENCODER_LOWER
                    && pos_est <= ENCODER_UPPER)
                {
                    // If the absolute change in the servo position is 
                    // greater than ENCODER_STEP then we increment
                    // (or decrement) the revolution counter.
                    int16_t delta = pos_est - prev_valid_position_;
                    if (delta > ENCODER_STEP)
                    {
                        revolutions_ -= 1;
                    }
                    if (delta < -ENCODER_STEP)
                    {
                        revolutions_ += 1;
                    }

                    // Update the previous valid position
                    prev_valid_position_ = pos_est;
                }
            }
        }

        /// \brief Get the number of revoutions since reset.
        ///
        /// \return An integer number of revolutions since the count was reset.
        ///
        int16_t getRevolutions() const
        {
            return revolutions_;
        }

        /// \brief Get the current encoder count since reset (filtered).
        ///
        /// Note that the encoder count is offset from the servo position
        /// so that the count is zero when the encoder filter is reset.
        ///
        /// \return An integer, the current encoder count.
        ///
        int16_t getCount() const
        {
            int16_t count = prev_valid_position_ + ENCODER_MAX * revolutions_; 
            return invert_ * (count - count_offset_); 
        }

        /// \brief Get the current encoder duty.
        ///
        /// \return An integer, the current encoder duty.
        ///
        int16_t getDuty() const
        {
            int16_t duty = duty_[index_]; 
            return duty;
        }

        /// Get the angular position of the encoder (filtered)
        ///
        /// \return A double, the angular position of the encoder [rad].
        ///
        double getAngularPosition() const
        {
            return 2.0 * M_PI * getCount() / ENCODER_MAX;
        }

        /// \brief Get the current (un-filtered) servo position and an
        /// estimate whether it is valid.
        ///
        /// \param position An integer reference that is set to the current position. 
        /// \param is_valid A bool that is set to true for valid, false otherwise.
        /// \param map_position A bool: if true map the position to the
        /// range [0, 1500],  has (default True)
        ///
        void getServoPosition(int16_t &position, bool &is_valid, bool map_position=true) const
        {
            is_valid = true; //classifier_.predict([X_])[0];
            if (map_position)
            {
                position = position_[index_] % ENCODER_MAX;
            }
            else
            {
                position = position_[index_];
            }
        }

        /// \brief Get the invert state: whether the encoder count is inverted.
        ///
        /// \return An integer: -1 if the count is inverted, 1 otherwise. 
        ///
        int16_t getInvert() const
        {
            return invert_; 
        }

        /// \brief Invert the direction of the encoder count.
        ///
        /// \param is_inverted A bool: true if the encoder count is reversed.
        ///
        void setInvert(bool is_inverted)
        {
            invert_ = is_inverted ? -1 : 1;   
        }

        /// Reset the encoder counters to zero.
        ///
        /// \param position An integer (assumed valid) position of the
        /// servo when the encoder is reset.  
        /// 
        void reset(int16_t position)
        {
            ros::Time now = ros::Time::now();
            for (int16_t i=0; i<window_; ++i)
            {
                ros::Time t = now - ros::Duration((window_ - i)/50.0);
                update(t, 0, position);
            }
            
            // Calculate the offset to zero the counter
            int16_t curr_position = 0;
            bool is_valid = false;
            getServoPosition(curr_position, is_valid);
            if (is_valid)
            {
                count_offset_ = curr_position;
            }

            // Initialise remaining variables
            revolutions_    = 0;
            prev_valid_position_ = curr_position;
        }

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
        void loadClassifier()
        {
        }

        void loadRegressor()
        {
        }
    };

////////////////////////////////////////////////////////////////////////
// LX16AEncoderFilterOnnx

    LX16AEncoderFilterOnnx::~LX16AEncoderFilterOnnx()
    {
    }

    LX16AEncoderFilterOnnx::LX16AEncoderFilterOnnx(
        const std::string &classifier_filename,
        const std::string &regressor_filename,
        int16_t window)
    {
    }

    void LX16AEncoderFilterOnnx::init()
    {
    }

    void LX16AEncoderFilterOnnx::add(uint8_t servo_id)
    {
    }

    void LX16AEncoderFilterOnnx::update(uint8_t servo_id, const ros::Time &ros_time, int16_t duty, int16_t position)
    {
    }

    int16_t LX16AEncoderFilterOnnx::getRevolutions(uint8_t servo_id) const
    {
        return 0;
    }

    int16_t LX16AEncoderFilterOnnx::getCount(uint8_t servo_id) const
    {
        return 0;
    }

    int16_t LX16AEncoderFilterOnnx::getDuty(uint8_t servo_id) const
    {
        return 0;
    }

    double LX16AEncoderFilterOnnx::getAngularPosition(uint8_t servo_id) const
    {
        return 0.0;
    }

    void LX16AEncoderFilterOnnx::getServoPosition(uint8_t servo_id, int16_t &position, bool &is_valid, bool map_position) const
    {        
    }

    int16_t LX16AEncoderFilterOnnx::getInvert(uint8_t servo_id) const
    {
        return 1;
    }

    void LX16AEncoderFilterOnnx::setInvert(uint8_t servo_id, bool is_inverted)
    {
    }

    void LX16AEncoderFilterOnnx::reset(uint8_t servo_id,  int16_t position)
    {
    }

    void LX16AEncoderFilterOnnx::add_v(const std::vector<uint8_t> &servo_ids)
    {   
    }

    void LX16AEncoderFilterOnnx::update_v(
        const std::vector<uint8_t> &servo_ids,
        const ros::Time &ros_time,
        const std::vector<int16_t> &duties,
        const std::vector<int16_t> &positions,
        std::vector<double> &angular_positions)
    {   
    }

} // namespace lx16a

