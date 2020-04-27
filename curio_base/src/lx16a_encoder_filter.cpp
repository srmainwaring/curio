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

#include "curio_base/lx16a_encoder_filter.h"
#include <cmath>

namespace curio_base
{
    LX16AEncoderFilter::~LX16AEncoderFilter()
    {
    }

    LX16AEncoderFilter::LX16AEncoderFilter(
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
        // Load the ML classifier pipeline
        loadClassifier();

        // Load the ML regressor pipeline
        loadRegressor();
    }

    void LX16AEncoderFilter::update(const ros::Time &ros_time, int16_t duty, int16_t position)
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

    int16_t LX16AEncoderFilter::getRevolutions() const
    {
        return revolutions_;
    }

    int16_t LX16AEncoderFilter::getCount() const
    {
        int16_t count = prev_valid_position_ + ENCODER_MAX * revolutions_; 
        return invert_ * (count - count_offset_); 
    }

    int16_t LX16AEncoderFilter::getDuty() const
    {
        int16_t duty = duty_[index_]; 
        return duty;
    }

    double LX16AEncoderFilter::getAngularPosition() const
    {
        return 2.0 * M_PI * getCount() / ENCODER_MAX;
    }

    void LX16AEncoderFilter::getServoPosition(int16_t &position, bool &is_valid, bool map_position) const
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

    int16_t LX16AEncoderFilter::getInvert() const
    {
        return invert_; 
    }

    void LX16AEncoderFilter::setInvert(bool is_inverted)
    {
        invert_ = is_inverted ? -1 : 1;   
    }

    void LX16AEncoderFilter::reset(int16_t position)
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

    void LX16AEncoderFilter::loadClassifier()
    {

    }

    void LX16AEncoderFilter::loadRegressor()
    {

    }

} // namespace curio_base

