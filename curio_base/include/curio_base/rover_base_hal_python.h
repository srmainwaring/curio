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

#ifndef CURIO_BASE_ROVER_BASE_HAL_PYTHON_H_
#define CURIO_BASE_ROVER_BASE_HAL_PYTHON_H_

#include "curio_base/rover_base_hal.h"

#include <pybind11/pybind11.h>
#include <ros/ros.h>
#include <vector>

namespace curio_base
{
    /// \brief Mock implementation of the rover base hardware abstraction layer for testing.
    class RoverBaseHALPython : public RoverBaseHAL
    {
    public:
        /// \copydoc RoverBaseHAL::~RoverBaseHAL
        ~RoverBaseHALPython() override;

        /// \brief Constructor.
        RoverBaseHALPython(ros::NodeHandle &nh, ros::NodeHandle &private_nh);

        /// \copydoc RoverBaseHAL::getNumWheels()
        size_t getNumWheels() const override;

        /// \copydoc RoverBaseHAL::getNumSteers()
        size_t getNumSteers() const override;

        /// \copydoc RoverBaseHAL::getWheelPosition()
        double getWheelPosition(const ros::Time &time, int i) const override;

        /// \copydoc RoverBaseHAL::getWheelVelocity()
        double getWheelVelocity(const ros::Time &time, int i) const override;

        /// \copydoc RoverBaseHAL::setWheelVelocity()
        void setWheelVelocity(const ros::Time &time, int i, double velocity) override;

        /// \copydoc RoverBaseHAL::getSteerAngle()
        double getSteerAngle(const ros::Time &time, int i) const override;

        /// \copydoc RoverBaseHAL::setSteerAngle()
        void setSteerAngle(const ros::Time &time, int i, double angle) override;

        /// \copydoc RoverBaseHAL::getWheelPositions()
        void getWheelPositions(const ros::Time &time, std::vector<double>& positions) const override;

        /// \copydoc RoverBaseHAL::getWheelVelocities()
        void getWheelVelocities(const ros::Time &time, std::vector<double>& velocities) const override;

        /// \copydoc RoverBaseHAL::setWheelVelocities()
        void setWheelVelocities(const ros::Time &time, const std::vector<double>& velocities) override;

        /// \copydoc RoverBaseHAL::getSteerAngles()
        void getSteerAngles(const ros::Time &time, std::vector<double>& positions) const override;

        /// \copydoc RoverBaseHAL::setSteerAngles()
        void setSteerAngles(const ros::Time &time, const std::vector<double>& positions) override;

    private:
        // Node handles
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // Store the Python module and class instance objects as well
        // as a pointer to the C++ type. If the Python objects go out of
        // scope the pointer is undefined.   
        pybind11::object py_module_;
        pybind11::object py_rover_base_hal_;
        RoverBaseHAL *rover_base_hal_ = nullptr;
    };

} // namespace curio_base

#endif // CURIO_BASE_ROVER_BASE_HAL_PYTHON_H_
