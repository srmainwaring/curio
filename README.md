# ROS packages for Curio - a Sawppy Rover

This is a collection of ROS software packages to control a version of
[Roger Chen's Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover).

These packages are intended to help get builders of Roger's Sawppy up and running on ROS
while staying faithful to the original design using LX-16A servo motors. This presents some
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
They have been built and tested on the following platforms / distributions:

### macOS

- OSX 10.11.6
- ROS Melodic Morenia
- Gazebo version 9.6.0

### Linux

- Ubuntu 18.04.03 LTS (Bionic Beaver)
- ROS Melodic Morenia
- Gazebo version 9.0.0

### Raspberry Pi 4 (rover only - no simulation)

- Raspbian GNU/Linux 10 (buster)
- ROS Melodic Morenia (source build)

### Python

The ROS packages rely on the Python
[`pyserial`](https://pyserial.readthedocs.io/en/latest/pyserial.html) package
to communicate with the hardware devices, so make sure this is installed:

```bash
python -m pip install pyserial
```

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

Source the build:

```bash
source devel/setup.bash
```


## Usage - Rover

### `curio_base`

This package is used to control the rover. It contains a hardware interface
for the LX-16A servos and a motor controller node that subscribes to ROS
`geometry_msgs/Twist` messages published to the topic `/cmd_vel`.

#### Set up the hardware

The first step is to check that the motor controller is working correctly.
Make sure your rover is supported with it's wheels off the ground with all joints
able to rotate / turn freely.

Connect the LewanSoul Servo Bus board to your computer with a USB cable and power
the servo board up. You can check the USB serial device names using:

```bash
$ python -m serial.tools.list_ports -v
/dev/ttyUSB0
    desc: USB2.0-Serial
    hwid: USB VID:PID=1A86:7523 LOCATION=1-2
1 ports found
```

You'll get different device names according to the operating system and number of USB ports in use.

### Launch the motor controller node

Start `roscore`:

```bash
roscore
```

In another terminal launch the motor controller:

```bash
roslaunch curio_base motor_controller.launch port:=/dev/ttyUSB0
```

where you should substitute the correct port for the LewanSoul Servo Bus board.

On linux you may get an error such as:

```bash
serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyUSB0: [Errno 13] permission denied: '/dev/ttyUSB0'
```

to resolve this you need to set access permissions on the device with:

```bash
sudo chmod 777 /dev/ttyUSB0
```

then try re-launching the motor controller node.

### Test the motor controller

Check the motor controller is subscribing to `/cmd_vel`:

```bash
$ rosnode info /curio_motor_controller 
--------------------------------------------------------------------------------
Node [/curio_motor_controller]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /cmd_vel [unknown type]

Services:
 * /curio_motor_controller/get_loggers
 * /curio_motor_controller/set_logger_level


contacting node http://<your machine ip here>/ ...
Pid: 67587
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
```

Start `rqt_robot_steering` and send commands:

```bash
rosrun rqt_robot_steering rqt_robot_steering
```

You should see the robot steering widget.
Setting the linear velocity should cause the wheels to move forwards / backwards.
Setting the angular velocity should cause the corner steering joints to rotate
and the wheels turn.

### `curio_teleop`

This package is used to control the robot using a radio control setup.

## Usage - Visualisation

### `curio_viz`

This package contains launch and `rviz` configuration files.

To view the rover in `rviz` and manually control the joints run:

```bash
roslaunch curio_view view_model.launch
```

## Usage - Simulation

### `curio_gazebo`

This package contains launch files for spawning the robot model into a Gazebo simulation:

```bash
roslaunch curio_gazebo curio_empty_world.launch
```

The robot should appear in an empty world, with a `rqt_robot_steering`
widget in a separate window.

You can simultaneously view the rover in `rviz` with:

```bash
roslauch curio_viz view_robot.launch
```

The robot will appear in the `/odom` frame and the joints will respond as the rover
is moved in the simulation.

## Other packages

### `ackermann_drive_controller`

### `curio_bringup`

### `curio_control`

### `curio_description`

The `curio_description` package contains a URDF / xacro model of the rover with
`<gazebo>` extensions for simulating the rover in Gazebo.

Note that the rocker-bogie differential cannot be fully modelled in URDF because it
induces closed loop kinematic chains. The SDF representation of the model does capture
the full kinematics so you will see additional links and joints in Gazebo that are not
present in `rviz`. These are the two turnbuckle linkages with ball joints that connect
the rocker assemblies to the differential beam.

## License

This software is licensed under the BSD-3-Clause license found in the LICENSE file
in the root directory of this source tree.

## Acknowledgments

Many thanks to Roger Chen for publishing detailed plans and build instructions for his Sawppy Rover:

- [Hackaday](https://hackaday.io/project/158208-sawppy-the-rover)
- [GitHub](https://github.com/Roger-random/Sawppy_Rover)
- [OnShape CAD file](https://cad.onshape.com/documents/43678ef564a43281c83e1aef/w/392bbf8745395bc24367a35c/e/9bd6bbb7aba50a97523d14f2)
- [Build Blog](https://newscrewdriver.com/category/projects/sawppy-the-rover/)

The JPL Open Source Rover project provided the inspiration for the Sawppy Rover and has helpful
documentation on the Ackermann steering calculations:

- [Open Source Rover Home](https://opensourcerover.jpl.nasa.gov/#!/home)
- [Github](https://github.com/nasa-jpl/open-source-rover)
- [Michael Cox, Eric Junkins, Olivia Lofaro. "Open Source Rover: Software Controls"](https://github.com/nasa-jpl/open-source-rover/blob/master/Software/Software%20Controls.pdf)

There is a vast body of information to be found about using ROS. In addition to the ROS
and Gazebo tutorials, I have found the open source packages and manual for the
Turtlebot 3 by Robotis and the Clearpath Husky very useful when trying to understand
how to organsise and structure packages for a rover:

- [Robotis e-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [Husky](https://github.com/husky)
