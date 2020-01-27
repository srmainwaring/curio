# ROS packages for Curio - a Sawppy Rover

This is a collection of ROS software packages to control a version of
[Roger Chen's Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover).

These packages are intended to help get builders of Roger's Sawppy up and running on ROS
while staying faithful to the original design using LX-16A servo motors. This presents some
challenges with getting reliable odometry which we address with an encoder filter
that identifies when the encoder position is outside its valid range.

## Overview

There are a number of ROS packages to control the rover, visualise it in rviz,
and simulate it in [Gazebo](http://gazebosim.org/).

- `ackermann_drive_controller` a 6 wheel, 4 steering controller consistent with the [`ros_control`](http://wiki.ros.org/ros_control) framework.
- `curio_base` hardware drivers and a ROS motor controller node subscribing to [`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) on `/cmd_vel`.
- `curio_bringup` a set of launch files for bringing up the rover nodes
- `curio_control` configuration and launch files using the [`ros_control`](http://wiki.ros.org/ros_control) framework
- `curio_description` a URDF / xacro model for the robot using STL files from the Sawppy CAD model
- `curio_gazebo` configuration and launch files for spawning the rover in [Gazebo with ROS control](http://gazebosim.org/tutorials/?tut=ros_control)
- `curio_teleop` a telep node for interpreting PWM signals from a RC unit and publishing to `/cmd_vel`  
- `curio_viz` configuration and launch files for loading the robot model into [`rviz`](http://wiki.ros.org/rviz).

For more detail see the sections below.

## Dependencies

You will need a working installation of ROS and [Gazebo](http://gazebosim.org/) to use these packages.
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

These packages use ROS and Python 2.7. In addition to the Python packages
required for a ROS desktop-full installation you will need the following:

For the serial communication with USB connected hardware devices:

- [`pyserial`](https://pypi.org/project/pyserial/)

For the machine learning classifier used in the encoder filter:

- [`joblib`](https://pypi.org/project/joblib/)
- [`numpy`](https://pypi.org/project/numpy/)
- [`pandas`](https://pypi.org/project/pandas/)
- [`scikit-learn`](https://pypi.org/project/scikit-learn/)
- [`scipy`](https://pypi.org/project/scipy/)


### C++

For C++ serial communication we use:

- [`serial`](https://github.com/wjwwood/serial).

To install:

```bash
cd ~/curio_ws/src
git clone https://github.com/wjwwood/serial.git
cd ~/curio_ws
catkin build
```

## Installation

### Create and configure a workspace

Source your ROS installation:

```bash
source /opt/ros/melodic/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p ~/curio_ws/src
cd ~/curio_ws
catkin init
```

### Clone and build the packages

Clone the [`curio_msgs`](https://github.com/srmainwaring/curio_msgs.git) and
[`curio`](https://github.com/srmainwaring/curio.git) packages into `curio_ws/src`:

```bash
cd ~/curio_ws/src
git clone https://github.com/srmainwaring/curio_msgs.git
git clone https://github.com/srmainwaring/curio.git
```

Build the packages from the `curio_ws` directory:

```bash
cd ~/curio_ws
catkin build
```

Source the build:

```bash
source devel/setup.bash
```

## Calibration

The encoder filter uses a decision tree classifier and regressor from `scikit-learn`.
The persistence approach supported natively in `scikit-learn` relies on
Python pickle, which may not be compatible across Python and package versions.
For this reason we have included the training data set and training routines
in the `curio_base` package so that you can create instances of the models
customised to your environment.

The labelled data and its corresponding dataset, and default instances
of a classifier and regressor are located in `curio_base/data`:

```bash
curio_base/data/lx16a_dataset.zip
curio_base/data/lx16a_labelled_data.zip
curio_base/data/lx16a_tree_classifier.joblib
curio_base/data/lx16a_tree_regressor.joblib
```

It is strongly recommended for security and compatibility that you create
a new instance of the classifier and regressor:

```bash
roslaunch curio_base train_classifier.launch
```

```bash
roslaunch curio_base train_regressor.launch
```

which will overwrite the default classifier and regresssor instances
included with the distribution. This will need to be run on the rover
(and the desktop computer if you want to run the base controller
from there while testing).

## Usage - Rover

### `curio_base`

This package is used to control the rover. It contains a hardware interface
for the LX-16A servos and a motor controller node that subscribes to ROS
[`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
messages published to the topic `/cmd_vel`.

#### Configuration

The motor controller is configured using the file `curio_base/config/motor_controller.yaml`.
The most important parameters to check are the serial id's assigned to each servo, and to ensure
that these are correct for your rover. The default configuration uses the following assignment:

| Joint Name        |  Servo ID |
| :---| :---: |
| Front Left Wheel  | 11  |
| Mid Left Wheel    | 12  |
| Back Left Wheel   | 13  |
| Front Right Wheel | 21  |
| Mid Right Wheel   | 22  |
| Back Right Wheel  | 23  |
| Front Left Steer  | 111 |
| Mid Left Steer    | 121 |
| Mid Right Steer   | 221 |
| Back Right Steer  | 231 |

#### Set up the hardware

The first step is to check that the motor controller is working correctly.
Make sure your rover is supported with it's wheels off the ground with all joints
able to rotate / turn freely.

Connect the LewanSoul BusLinker TTL/USB debug board to your computer with a USB cable and power
it up. You can check the USB serial device names using:

```bash
$ python -m serial.tools.list_ports -v
/dev/ttyUSB0
    desc: USB2.0-Serial
    hwid: USB VID:PID=1A86:7523 LOCATION=1-2
1 ports found
```

You'll get different device names according to the operating system and number of USB ports in use.

### Launch the motor controller node

Start [`roscore`](http://wiki.ros.org/roscore):

```bash
roscore
```

In another terminal launch the motor controller:

```bash
roslaunch curio_base motor_controller.launch port:=/dev/ttyUSB0
```

where you should substitute the correct port for the LewanSoul BusLinker board.

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

Start [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering) and send commands:

```bash
rosrun rqt_robot_steering rqt_robot_steering
```

You should see the robot steering widget.
Setting the linear velocity should cause the wheels to move forwards / backwards.
Setting the angular velocity should cause the corner steering joints to rotate
and the wheels turn.

### `curio_teleop`

This package is used to control the robot using a radio control setup.
It depends on the [`rosserial`](http://wiki.ros.org/rosserial/) libraries
and assumes that the `curio_firmware` package
has been used to program the Arduino controlling the RC receiver.

## Usage - Visualisation

### `curio_viz`

This package contains launch and [`rviz`](http://wiki.ros.org/rviz) configuration files.

To view the rover in [`rviz`](http://wiki.ros.org/rviz) and manually control the joints run:

```bash
roslaunch curio_view view_model.launch
```

![Rviz View Model](https://github.com/srmainwaring/curio/wiki/images/curio_viz_view_model_rviz.jpg)
<!-- ![Curio View Model Joints](https://github.com/srmainwaring/curio/wiki/images/curio_viz_view_model_joint_state_publisher_gui.jpg) -->

## Usage - Simulation

### `curio_gazebo`

This package contains launch files for spawning the robot model into a
[Gazebo](http://gazebosim.org/) simulation.

The first places the rover in an empty world:

```bash
roslaunch curio_gazebo curio_empty_world.launch
```

The robot should appear on an empty ground plane in [Gazebo](http://gazebosim.org/) with a
[`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering)
widget in a separate window.

![Gazebo Empty World](https://github.com/srmainwaring/curio/wiki/images/curio_gazebo_curio_empty_world_gazebo.jpg)

You can simultaneously view the rover in [`rviz`](http://wiki.ros.org/rviz) with:

```bash
roslaunch curio_viz view_robot.launch
```

![Rviz View Robot](https://github.com/srmainwaring/curio/wiki/images/curio_gazebo_curio_empty_world_rviz.jpg)

The robot is visualised in the [`/odom`](https://www.ros.org/reps/rep-0105.html)
frame and the joints will respond as the rover is moved in the simulation.

The second launch file:

```bash
roslaunch curio_gazebo curio_mars_world.launch
```

places the rover in a Mars terrain model sourced from
[curiosity_mars_rover](https://bitbucket.org/theconstructcore/curiosity_mars_rover/src/master/).

![Gazebo Mars World](https://github.com/srmainwaring/curio/wiki/images/curio_gazebo_mars_terrain.jpg)

## Other packages

### `ackermann_drive_controller`

This package contains a controller plugin for a six wheel rover with four wheel Ackermann steering
that can be used in the [`ros_control`](http://wiki.ros.org/ros_control) framework. It was
adapted from the [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller?distro=melodic)
from the [`ros_controllers`](http://wiki.ros.org/ros_controllers?distro=melodic) library.

It is used in `curio_gazebo` to control the simulated rovers steering.

### `curio_bringup`

This package contains launch files for bringing up the entire robot. Typically they
configure and coordinate calling launch files from other packages in this distribution.

To bringup the robot in a single command:

```bash
roslaunch curio_bringup curio_robot.launch motor_port:=/dev/ttyUSB0 teleop_port:=/dev/ttyACM0
```

### `curio_control`

This package contains configuration and launch files for joint controllers such as
the `ackermann_drive_controller`. Not currently in use: `curio_base` will depend
on this package once support for the [`ros_control`](http://wiki.ros.org/ros_control) framework has been added to
the robot hardware abstraction layer.  

### `curio_description`

The `curio_description` package contains a URDF / xacro model of the rover with
`<gazebo>` extensions.

Note that the rocker-bogie differential cannot be fully modelled in URDF because it
induces closed loop kinematic chains. The SDF representation of the model does capture
the full kinematics so you will see additional links and joints in [Gazebo](http://gazebosim.org/)
that are not present in [`rviz`](http://wiki.ros.org/rviz). These are the two turnbuckle linkages
with ball joints that connect the rocker assemblies to the differential beam.

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

Miguel Angel Rodriguez and TheConstruct team for the `curiosity_path`
[Gazebo](http://gazebosim.org/) terrain model:

- [curiosity_mars_rover on BitBucket](https://bitbucket.org/theconstructcore/curiosity_mars_rover/src/master/)
- [NASA 3D models](https://nasa3d.arc.nasa.gov/detail/curiosity-path).

The turnbuckle CAD file used in the rover description was authored by Carlos Rey and retrieved from GrabCAD:

- [Tensor/Turnbuckle](https://grabcad.com/library/tensor-turnbuckle-1)

S. Chitta, E. Marder-Eppstein, W. Meeussen, V. Pradeep, A. Rodríguez Tsouroukdissian, J. Bohren, D. Coleman, B. Magyar, G. Raiola, M. Lüdtke and E. Fernandez Perdomo
**"ros_control: A generic and simple control framework for ROS"**,
The Journal of Open Source Software, 2017. ([PDF](http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf)). The `ackermann_drive_steering` controller borrows heavily
from the
[`diff_drive_controller`](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller)
and the entire framework made the Gazebo integration much simpler.

There is a vast body of information that describes how to use ROS. In addition to the ROS
and [Gazebo](http://gazebosim.org/) tutorials, I have found the open source packages and manual for the
Turtlebot 3 by Robotis and the Clearpath Husky very useful when trying to understand
how to organsise and structure packages for a rover:

- [Robotis e-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [Husky](https://github.com/husky)
