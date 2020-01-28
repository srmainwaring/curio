# LX-16A Learn

ML estimator to predict whether the position data from a LX-16A servo is in its valid range or not.

## Instructions

### Generate basic data

Run `roscore`:

```bash
roscore
```

Run rotary encoder node

```bash
roslaunch curio_base rotary_encoder.launch
```

Run the servo logger node:

```bash
rosrun curio_base lx16a_encoder_logger.py
```

Start `rqt` and use the topic publisher to the linear velocity component of `/cmd_vel`.

### Generate data from a simulation

#### Record a sample of `/cmd_vel`

Run `roscore`

```bash
roscore
```

Run the Gazebo simulation node:

```bash
roslaunch curio_gazebo curio_mars_world.launch 
```

Run the `teleop_rc` node:

```bash
roslaunch curio_teleop teleop_rc.launch port:=/dev/cu.usbmodemFD5121
```

Relay `/cmd_vel` to `/ackermann_drive_controller/cmd_vel` for the controller:

```bash
rosrun topic_tools relay /cmd_vel /ackermann_drive_controller/cmd_vel
```

Start recording with `rosbag`:

```bash
rosbag record -o curio_mars cmd_vel
```

Drive the rover about the simulation using the RC unit. 

#### Play a sample of `/cmd_vel`

Run `roscore`:

```bash
roscore
```

Run the rotary encoder node:

```bash
roslaunch curio_base rotary_encoder.launch
```

Run the servo logger node:

```bash
rosrun curio_base lx16a_encoder_logger.py
```

Play the `rosbag` using simulation time:

```bash
rosparam set use_sim_time true
rosplay --clock curio_mars_2020-01-11-16-49-30.bag
```

The calibration node will record the encoder count associated with the servo position
for the commanded duty.
