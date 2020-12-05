In order to start up the simulation:
```
roslaunch mybot_gazebo mybot_world.launch
roslaunch roslaunch curio_viz view_model.launch
```

Send a base controller command and ensure that the robot moves in both Gazebo and rviz:
```
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
```

To make a map:
Terminal A: 
A$: roslaunch curio_gazebo curio_shapes_world.launch 
will start up gazebo and rqt_robot_steering
Terminal B:
B$: ± roslaunch curio_viz view_robot.launch 
Terminal B:
C$: ± rosrun gmapping slam_gmapping scan:=sensors/laser
Gazebo:
 Don't forget to hit 'play' before you move the robot around.

Driver the robot around, making a map as seen in rviz.  When you are happy with the map:
 rosrun map_server map_saver -f <map_name>



Trying to mirror https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation

roslaunch curio_navigation curio_navigation.launch map_file:=$HOME/curio_shapes_map.yaml
