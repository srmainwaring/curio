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

#ifndef CURIO_BASE_ROVER_BASE_HAL_H_
#define CURIO_BASE_ROVER_BASE_HAL_H_

#include <ros/ros.h>
#include <exception>

namespace curio_base
{
    /// \brief Exception for the rover hardware abstraction layer.
    class RoverBaseHALException : public std::exception
    {
        // Disable copy constructors
        RoverBaseHALException& operator=(const RoverBaseHALException&);
        std::string e_what_;
    public:
        RoverBaseHALException (const char *description);
        RoverBaseHALException (const RoverBaseHALException& other);
        virtual ~RoverBaseHALException() throw();
        virtual const char* what () const throw ();
    };

    /// \brief Hardware Abstraction Layer for the rover base.
    class RoverBaseHAL
    {
    public:
        /// \brief Destructor.
        virtual ~RoverBaseHAL();

        /// \brief Get the number of wheels.
        virtual size_t getNumWheels() const = 0;

        /// \brief Get the number of steers.
        virtual size_t getNumSteers() const = 0;

        /// \brief Get the angular position of the i-th wheel [rad].
        virtual double getWheelPosition(const ros::Time &time, int i) const = 0;

        /// \brief Get the angular velocity of the i-th wheel [rad/s].
        virtual double getWheelVelocity(const ros::Time &time, int i) const = 0;

        /// \brief Set the angular velocity of the i-th wheel [rad/s].
        virtual void setWheelVelocity(const ros::Time &time, int i, double velocity) = 0;

        /// \brief Get the steering angle of the i-th steer [rad].
        virtual double getSteerAngle(const ros::Time &time, int i) const = 0;

        /// \brief Set the angle of the i-th steer [rad].
        virtual void setSteerAngle(const ros::Time &time, int i, double angle) = 0;

        // Vectorised interface.

        /// \brief Get the wheel angular positions [rad].
        virtual void getWheelPositions(const ros::Time &time, std::vector<double>& positions) const = 0;

        /// \brief Get the wheel angular velocities [rad/s].
        virtual void getWheelVelocities(const ros::Time &time, std::vector<double>& velocities) const = 0;

        /// \brief Set the wheel angular velocities [rad/s].
        virtual void setWheelVelocities(const ros::Time &time, const std::vector<double>& velocities) = 0;

        /// \brief Get the steering angles [rad].
        virtual void getSteerAngles(const ros::Time &time, std::vector<double>& positions) const = 0;

        /// \brief Set the steering angles [rad].
        virtual void setSteerAngles(const ros::Time &time, const std::vector<double>& positions) = 0;

    };

} // namespace curio_base

#endif // CURIO_BASE_ROVER_BASE_HAL_H_
