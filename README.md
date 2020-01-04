# ROS packages for Curio - a Sawppy Rover

This is a collection of ROS software packages to control a version of
[Roger Chen's Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover).

These packages are intended to help get builders of Roger's Sawppy up and running on ROS
while staying faithful to the original design using LX-16A servo motors. That presents some
challenges with getting reliable odometry which are still being worked through. On the other hand
teleoperation and simulation work well.

## Overview

There are a number of ROS packages to control the rover, visualise it in rviz,
and simulate it in Gazebo.

- `ackermann_drive_controller` a 6 wheel, 4 steering controller consistent with the `ros_control` framework.
- `curio_base` hardware drivers and a ROS motor controller node subscribing to `geometry_msgs/Twist` on `/cmd_vel`.
- `curio_bringup` a set of launch files for bringing up the rover nodes
- `curio_control` configuration and launch files using the `ros_control` framework
- `curio_description` a URDF / xacro model for the robot using STL files from the Sawppy CAD model
- `curio_gazebo` configuration and launch files for spawning the rover in Gazebo with ROS control
- `curio_teleop` a telep node for interpreting PWM signals from a RC unit and publishing to `/cmd_vel`  
- `curio_viz` configuration and launch files for loading the robot model into rviz.

For more detail see the sections below.

## Dependencies

You will need a working installation of ROS and Gazebo to use these packages.

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p curio_ws/src
cd curio_ws
catkin init
```

### Clone and build the packages

Clone the `curio_msgs` and `curio` packages into `curio_ws/src`:

```bash
cd src
git clone https://github.com/srmainwaring/curio_msgs.git
git clone https://github.com/srmainwaring/curio.git
```

Build the packages from the `curio_ws` directory:

```bash
catkin build
```

## Usage - Rover

### macOS

- OSX 10.11.6
- ROS Melodic Morenia
- Gazebo version 9.6.0

### Linux

- Ubuntu 18.04.03 LTS (Bionic Beaver)
- ROS Melodic Morenia
- Gazebo version 9.0.0

### Raspberry Pi 4

- Raspbian GNU/Linux 10 (buster)
- ROS Melodic Morenia (source build)




### `curio_base`

This package is used to control the rover. It contains a hardware interface
for the LX-16A servos and a motor controller node that subscribes to ROS
`geometry_msgs/Twist` messages published to the topic `/cmd_vel`. The Sawppy Rover is
capable of in-place turns, and so meets one of the the requirements of a mobile base
for the ROS navigation stack.

### `curio_teleop`

This package is used to map PWM messages from a radio control unit into 

## Usage - Visualisation

### `curio_viz`

## Usage - Simulation

### `curio_gazebo`

The Gazebo simulation may be run standalone and uses the `ros_control` framework for drive and
steering control using the `ackermann_drive_controller` package.

## Other packages

The plan is to integrate a C++ version of the LX-16A driver with the `ros_control` framework so that
both hardware and simulation use the same `ackermann_drive_controller` and configration, which should
also make it more straighforward to adapt to different device drivers.

### `ackermann_drive_controller`

### `curio_bringup`

### `curio_control`

### `curio_description`


## License

This software is licensed under the BSD-3-Clause license found in the LICENSE file
in the root directory of this source tree.

## Acknowledgments
