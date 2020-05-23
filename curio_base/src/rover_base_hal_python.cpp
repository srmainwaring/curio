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

#include "curio_base/rover_base_hal_python.h"

#include <pybind11/pybind11.h>
#include <ros/ros.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace curio_base
{
    RoverBaseHALPython::~RoverBaseHALPython()
    {        
    }

    RoverBaseHALPython::RoverBaseHALPython(ros::NodeHandle &nh, ros::NodeHandle &private_nh) :
        nh_(nh),
        private_nh_(private_nh)
    {
        // Load params
        std::string hal_module_, hal_class_;
        ros::param::param<std::string>("~python_hal_module", hal_module_, "curio_base.ext");
        ros::param::param<std::string>("~python_hal_class", hal_class_, "MockRoverBaseHAL");

        // Import derived class and cast from Python to C++.
        ROS_INFO_STREAM("Attempting to load Python RoverBaseHAL module: " << hal_module_ << "." << hal_class_);
        py_module_ = py::module::import(hal_module_.c_str()).attr(hal_class_.c_str());
        py_rover_base_hal_ = py_module_();
        rover_base_hal_ = py_rover_base_hal_.cast<RoverBaseHAL*>();
    }

    size_t RoverBaseHALPython::getNumWheels() const
    {
        return rover_base_hal_->getNumWheels();
    }

    size_t RoverBaseHALPython::getNumSteers() const
    {
        return rover_base_hal_->getNumSteers();
    }

    double RoverBaseHALPython::getWheelPosition(const ros::Time &time, int i) const
    {
        return rover_base_hal_->getWheelPosition(time, i);
    }

    double RoverBaseHALPython::getWheelVelocity(const ros::Time &time, int i) const
    {
        return rover_base_hal_->getWheelVelocity(time, i);
    }

    void RoverBaseHALPython::setWheelVelocity(const ros::Time &time, int i, double velocity)
    {
        rover_base_hal_->setWheelVelocity(time, i, velocity);
    }

    double RoverBaseHALPython::getSteerAngle(const ros::Time &time, int i) const
    {
        return rover_base_hal_->getSteerAngle(time, i);
    }

    void RoverBaseHALPython::setSteerAngle(const ros::Time &time, int i, double angle)
    {
        rover_base_hal_->setSteerAngle(time, i, angle);
    }

    void RoverBaseHALPython::getWheelPositions(const ros::Time &time, std::vector<double>& positions) const
    {
        rover_base_hal_->getWheelPositions(time, positions);
    }

    void RoverBaseHALPython::getWheelVelocities(const ros::Time &time, std::vector<double>& velocities) const
    {
        rover_base_hal_->getWheelVelocities(time, velocities);
    }

    void RoverBaseHALPython::setWheelVelocities(const ros::Time &time, const std::vector<double>& velocities)
    {
        rover_base_hal_->setWheelVelocities(time, velocities);
    }

    void RoverBaseHALPython::getSteerAngles(const ros::Time &time, std::vector<double>& positions) const
    {
        rover_base_hal_->getSteerAngles(time, positions);
    }

    void RoverBaseHALPython::setSteerAngles(const ros::Time &time, const std::vector<double>& positions)
    {
        rover_base_hal_->setSteerAngles(time, positions);
    }

} // namespace curio_base
