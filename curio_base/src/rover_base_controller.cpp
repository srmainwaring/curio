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

#include "curio_base/rover_base_hal.h"
#include "curio_base/rover_base_hal_lx16a.h"
#include "curio_base/rover_base_hardware.h"
#include <lx16a/lx16a_pybind11_embed.h>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <chrono>
#include <functional>
#include <thread>

namespace py = pybind11;

typedef std::chrono::steady_clock time_source;

// Control loop (not realtime - timing scheduled by AsyncSpinner)
void controlLoop(curio_base::RoverBaseHardware &hardware,
    controller_manager::ControllerManager &controller_manager,
    time_source::time_point &last_time)
{
    // Acquire the Python GIL
    py::gil_scoped_acquire gil{};

    // Calculate monotonic time difference
    time_source::time_point this_time = time_source::now();
    std::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    // Process control loop
    hardware.read(ros::Time::now(), elapsed);
    controller_manager.update(ros::Time::now(), elapsed);
    hardware.write(ros::Time::now(), elapsed);
}

// Entry point
int main(int argc, char *argv[])
{
    // Initialise node
    ros::init(argc, argv, "rover_base_controller");
    ros::NodeHandle nh, private_nh("~");

    // Initialise the Python interpreter
    py::scoped_interpreter guard{};
    lx16a::addCmdArgsToSys(argc, argv);

    // Read parameters
    double control_frequency;
    private_nh.param<double>("control_frequency", control_frequency, 10.0);

    // Initialise robot hardware and controller manager
    curio_base::RoverBaseHardware rover_base_hardware(nh, private_nh);
    controller_manager::ControllerManager controller_manager(&rover_base_hardware, nh);

    // Custom callback queue (non-threadsafe?)
    ros::CallbackQueue callback_queue;

    time_source::time_point last_time = time_source::now();
    ros::TimerOptions control_timer(
        ros::Duration(1.0 / control_frequency),
        std::bind(controlLoop, std::ref(rover_base_hardware),
            std::ref(controller_manager), std::ref(last_time)),
        &callback_queue);
    ros::Timer control_loop = nh.createTimer(control_timer);

    // Process ROS callbacks (controller_manager)
    {
        // Release the Python GIL from this thread
        py::gil_scoped_release gil_release {};

        // Async spinner - must only use be called with 1 thread.
        ros::AsyncSpinner spinner(1, &callback_queue);
        spinner.start();
        ros::waitForShutdown();
    }

    return 0;
}