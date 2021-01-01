# ROS packages for Curio - a Sawppy Rover

This is a collection of ROS software packages to control a version of
[Roger Chen's Sawppy Rover](https://github.com/Roger-random/Sawppy_Rover).

These packages are intended to help get builders of Roger's Sawppy up
and running on ROS while staying faithful to the original design using LX-16A
servo motors. This presents some challenges with getting reliable odometry
which we address with an encoder filter that identifies when the encoder
position is outside its valid range.

## Overview

There are a number of ROS packages to control the rover, visualise it in rviz,
and simulate it in [Gazebo](http://gazebosim.org/).

- `ackermann_drive_controller` a 6 wheel, 4 steering controller consistent
with the [`ros_control`](http://wiki.ros.org/ros_control) framework.
- `curio_base` hardware drivers and a ROS base controller node subscribing to
[`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
on `/cmd_vel`.
- `curio_bringup` a set of launch files for bringing up the rover nodes
- `curio_control` configuration and launch files using the
[`ros_control`](http://wiki.ros.org/ros_control) framework
- `curio_description` a URDF / xacro model for the robot using STL files
from the Sawppy CAD model
- `curio_gazebo` configuration and launch files for spawning the rover in
[Gazebo with ROS control](http://gazebosim.org/tutorials/?tut=ros_control)
- `curio_navigation` configuration and launch files for the
[ROS navigation stack](http://wiki.ros.org/navigation).
- `curio_teleop` a telep node for interpreting PWM signals from a RC unit
and publishing to `/cmd_vel`  
- `curio_viz` configuration and launch files for loading the robot model into
[`rviz`](http://wiki.ros.org/rviz).

For more detail see the sections below.

## Dependencies

You will need a working installation of ROS and [Gazebo](http://gazebosim.org/)
to use these packages. They have been built and tested on the following
platforms / distributions:

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

### ROS

We use [`rosserial_arduino`](http://wiki.ros.org/rosserial_arduino)
to communicate with the Arduino controller. It is included in the
[`rosserial`](http://wiki.ros.org/rosserial) stack and available
from GitHub:

- [https://github.com/ros-drivers/rosserial](https://github.com/ros-drivers/rosserial)

If you wish to use the navigation stack you will need a laser scanner
and a ROS driver for it. We use the [`rplidar`](http://wiki.ros.org/rplidar) driver:

- [https://github.com/Slamtec/rplidar_ros](https://github.com/Slamtec/rplidar_ros)

As usual, clone the source into your catkin source folder `~/curio/src`
and build with `catkin build`.

### Curio

Firmware and custom messages for the rover are maintained in separate
repositories:

- [`curio_firmware`](https://github.com/srmainwaring/curio_firmware.git)
Arduino firmware for the rover
- [`curio_msgs`](https://github.com/srmainwaring/curio_msgs.git)
ROS messages for the rover.

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

Clone the
[`curio`](https://github.com/srmainwaring/curio.git),
[`curio_firmware`](https://github.com/srmainwaring/curio_firmware.git)
and
[`curio_msgs`](https://github.com/srmainwaring/curio_msgs.git)
packages into `~/curio_ws/src`:

```bash
cd ~/curio_ws/src
git clone https://github.com/srmainwaring/curio.git
git clone https://github.com/srmainwaring/curio_firmware.git
git clone https://github.com/srmainwaring/curio_msgs.git
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

## Hardware

Curio was constructed using off the shelf consumer electronics
that require little or no modification (i.e. no custom boards):

- Arduino Mega 2560
- Raspberry Pi 4 4GB with 32GB SD card
- LewanSoul BusLinker LX-16A Bus Servo  v2.4
- Graupner mz-12 RC transmitter and Falcon 12 receiver
- 4-49V DC to 1.5-35V DC voltage regulator
- 7.2V NiMh 5000mAh battery
- RPLidar A1 laser scanner

Follow the wiring instructions in the
[Curio firmware Readme](https://github.com/srmainwaring/curio_firmware.git)
to set up the connections between the Arduino, BusLinker board and
the radio control receiver.

In the following we will assume that the rover has been configured
to use the Arduino as the servo controller and radio control decoder.
Note: this is a change from the previous version where the Python module
communicated with the servo bus directly. The reason for the change is
that the USB serial connection to the BusLinker board cannot provide
updates on all servo positions fast enough for the encoder filter
to work accurately.

## Calibration

### Encoder calibration

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
roslaunch curio_base train_regressor.launch
```

which will overwrite the default classifier and regresssor instances
included with the distribution. You will need to run this on the rover
(and the desktop computer if you want to run the base controller
from it while testing).

### Odometry calibration

The parameters for the base controller are in the configuration
file `curio_base/config/base_controller.yaml`.

The parameters:

- `wheel_radius_multiplier`
- `mid_wheel_lat_separation_multiplier`

are used to tune the odometry. A value of 1.0 means no adjustment.

Using the given settings for the rovers wheel geometry we found
the odometry linear measurement to be satisfactory. The angular
measurement was underestimated by about a 1/4 revolution
in every 8. To correct for this the wheel separation multiplier
is set to 31/32 = 0.96875. 

The reason for the adjustment can be explained as follows:
when the rovers middle wheels are under load the rocker and bogie
arms flex slighty, resulting in the wheels contact point with the
gound moving from the centre of the wheel towards the inner edge.
This reduces the effective lateral separation of the wheels and
results in the rover turning in-place faster than it would
otherwise.

## Usage - Rover

### `curio_base`

This package is used to control the rover. It contains a hardware interface
for the LX-16A servos and a base controller node that subscribes to ROS
[`geometry_msgs/Twist`](https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
messages published to the topic `/cmd_vel`.

#### *Configuration*

The base and arduino controllers are configured using the file
`curio_base/config/base_controller.yaml`.
The most important parameters to check are the serial id's assigned to
each servo, and to ensure that these are correct for your rover.
The default configuration uses the following assignment:

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

#### *Test the hardware*

The first step is to check that the arduino controller is working correctly.
Place the rover on blocks and ensure all joints are able to turn freely.

You can check the USB serial device names using:

```bash
$ python -m serial.tools.list_ports -v
/dev/ttyACM0
    desc: ttyACM0
    hwid: USB VID:PID=2341:0042 SER=75830333238351806212 LOCATION=1-1.4:1.0
/dev/ttyAMA0
    desc: ttyAMA0
    hwid: fe201000.serial
2 ports found
```

You'll get different device names according to the operating system
and number of USB ports in use. In this example on a Raspberry Pi
we see the Arduino is on `/dev/ttyACM0`.

#### *Test the arduino controller node*

Start [`roscore`](http://wiki.ros.org/roscore):

```bash
roscore
```

In a separate terminal launch the arduino controller:

```bash
roslaunch curio_base arduino_controller.launch port:=/dev/ttyACM0
```

where you should substitute the correct port for the Arduino.

On linux you may get an error such as:

```bash
serial.serialutil.SerialException: [Errno 13] could not open port /dev/ttyACM0: [Errno 13] permission denied: '/dev/ttyACM0'
```

to resolve this you need to add yourself to the dialout group:

```bash
sudo usermod -a -G dialout $USER
```

then try re-launching the arduino controller node.

If the node is running correctly you should see:

```bash
$ rosnode info /curio_arduino_controller
------------------------------------------------------------
Node [/curio_arduino_controller]
Publications:
 * /diagnostics [diagnostic_msgs/DiagnosticArray]
 * /radio/channels [curio_msgs/Channels]
 * /rosout [rosgraph_msgs/Log]
 * /servo/positions [curio_msgs/CurioServoPositions]
 * /servo/states [curio_msgs/CurioServoStates]

Subscriptions:
 * /servo/commands [unknown type]

Services:
 * /curio_arduino_controller/get_loggers
 * /curio_arduino_controller/set_logger_level


contacting node http://curio:40719/ ...
Pid: 2964
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
```

#### *Test the base controller node*

Launch the base controller:

```bash
roslaunch curio_base base_controller.launch
```

Inspecting the node you should see:

```bash
$ rosnode info /curio_base_controller
------------------------------------------------------------
Node [/curio_base_controller]
Publications:
 * /odom [nav_msgs/Odometry]
 * /rosout [rosgraph_msgs/Log]
 * /servo/commands [curio_msgs/CurioServoCommands]
 * /servo/encoders [curio_msgs/CurioServoEncoders]
 * /tf [tf2_msgs/TFMessage]

Subscriptions:
 * /cmd_vel [unknown type]
 * /servo/positions [curio_msgs/CurioServoPositions]

Services:
 * /curio_base_controller/get_loggers
 * /curio_base_controller/set_logger_level


contacting node http://curio:43269/ ...
Pid: 3249
Connections:
 * topic: /servo/commands
    * to: /curio_arduino_controller
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /servo/positions
    * to: /curio_arduino_controller (http://curio:40719/)
    * direction: inbound
    * transport: TCPROS
```

#### *Test the Ackermann steering*

Start [`rqt_robot_steering`](http://wiki.ros.org/rqt_robot_steering)
and send commands:

```bash
rosrun rqt_robot_steering rqt_robot_steering
```

You should see the robot steering widget.
Adjusting the linear velocity should cause the wheels to move forwards / backwards.
Adjusting the angular velocity should cause the corner steering joints to rotate
and the wheels turn.

#### *Test the base failsafe node [optional]*

When using an Arduino to communicate with the servo bus board there
are situations when the controller can leave the servos in a runaway
state if you use the Arduino IDE and bootloader to upload sketches:

- Arduino hardware reset
- Arduino watchdog timer reset
- rosserial error: 'Lost sync with device, restarting...'

The Arduino may fail to run the curio firmware after the reset
(i.e. setup()) is not called). This happens when rosserial continues
to publish data to the Arduino via USB serial after the reset
(i.e. rosserial_arduino has registered one or more subscribers).
rosserial does not stop transmitting data while it attempts
to resync, and this seems to be confusing the Arduino so that it
never moves on from its bootloader sequence.

As a result the servos are left powered on and running at
whatever duty was last set in the Arduino control loop.

The failsafe node takes advantage of the fact that communication via
the serial header on the Lewansoul BusLinker board appears to have
priority over the USB-to-TTL circuit. As a result we are able to run
a background process that attempts to stop the servos which is ignored
under usual operating conditions. However as soon as the Arduino is
reset and stops transmitting and receiving to the servo board, the
failsafe becomes effective and stops the servos.

To use the failsafe node connect the micro USB to the
Lewansoul BusLinker board *in addition* to the connection between
the Arduino and the serial header:

```bash
roslaunch curio_base base_failsafe.launch port:=/dev/ttyUSB0
```

where you will need to substitute the appropriate port for your
configuration.

To test the failsafe follow the instructions above for testing the
Ackermann steering and set the rover to move forward at half speed.
When you press the reset button on the Arduino the drive servos
should stop immediately. If you are monitoring the topic
`servos/positions` you will notice the messages stop. The RX LED
on the Arduino will continue to flash, and the `arduino_controller` node
will report that it has lost sync and is attempting to restart. You will
need to restart the `arduino_controller` node to re-stablish the
connection.

If you upload the curio firmware without a bootloader using ICSP then
the firmware should run immediately after reset, the initialisation
sequence should stop the servos without requiring the failsafe,
and rosserial should resync automatically.

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
roslaunch curio_viz view_model.launch
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


## Usage - Navigation

### `curio_navigation`

This package contains configuration and launch files for running the
[ROS navigation](http://wiki.ros.org/navigation) stack on the rover.

Currently we provide basic navigation support using the `move_base`
and `amcl` packages. For full details and tutorials please see the
[ROS navigation tutorials](http://wiki.ros.org/navigation/Tutorials).

The launch file `curio_navigation/launch/move_base_no_map.launch` and
associated configuration files are provided so you can observe the
[cost map](http://wiki.ros.org/costmap_2d) being created
in [`rviz`](http://wiki.ros.org/rviz) while driving the rover around.
See the tutorial
["How to Build a Map Using Logged Data"](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData)
for more detail.

### To create a map of the environment

- If `roscore` isn't aready running, run roscore

        $ roscore

- Open a new terminal and start the simulation:

        $ roslaunch curio_gazebo curio_shapes_world.launch

- Open a new terminal and start `rviz`:

        $ roslaunch curio_viz view_robot.launch

- Open a new terminal and start gmapping:

        $ rosrun gmapping slam_gmapping scan:=sensors/laser

- In the gazebo window, hit the "play" triangle

- In the `rqt_robot_steering` window, drive the robot around till you are happy with the map displayed in rviz.

- In a new terminal, save the developed map:

        $ rosrun map_server map_saver -f <map_name>

### To navigate using an existing map of the environment

- If `roscore` isn't aready running, run roscore

        $ roscore

- Open a new terminal and start the simulation:

        $ roslaunch curio_navigation curio_navigation.launch map_file:=<map_name>.yaml

- In rviz, check to see that the AMCL 'Particle Cloud' surrounds the robot.  If not, select '2D Pose Estimate' from the toolbar and click and drag on the map to give the pose estimator an initial estimate. The current sensor information is overlaid on the map in red to give you (and AMCL) a reference.

![Rviz View map](https://github.com/srmainwaring/curio/wiki/images/curio_navigation_rviz.png)

- In rviz, select `2D Nav Goal` from the toolbar, then click and drag on the map to write that goal to `move_base_simple/goal`.  The rover should move to that location.

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
roslaunch curio_bringup curio_robot.launch arduino_port:=/dev/ttyACM0 laser_scan_port:=/dev/ttyUSB0
```

It will start the base controller, arduino controller, and the laser scanner.

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

## Build Status

### Develop Job Status

|    | Melodic |
|--- |--- |
| curio_msgs | [![Build Status](https://travis-ci.com/srmainwaring/curio_msgs.svg?branch=develop)](https://travis-ci.com/srmainwaring/curio_msgs) |
| curio | [![Build Status](https://travis-ci.com/srmainwaring/curio.svg?branch=develop)](https://travis-ci.com/srmainwaring/curio) |


### Release Job Status

|    | Melodic |
|--- |--- |
| curio_msgs | [![Build Status](https://travis-ci.com/srmainwaring/curio_msgs.svg?branch=master)](https://travis-ci.com/srmainwaring/curio_msgs) |
| curio | [![Build Status](https://travis-ci.com/srmainwaring/curio.svg?branch=master)](https://travis-ci.com/srmainwaring/curio) |

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
