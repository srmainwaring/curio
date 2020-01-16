cmake_minimum_required(VERSION 2.8.3)
project(curio_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    curio_description
    curio_control
    curio_msgs
    geometry_msgs
    joint_state_publisher
    nav_msgs
    robot_state_publisher
    rospy
    # serial
    std_msgs
    tf
)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Declare catkin configuration

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
        curio_description
        curio_control
        curio_msgs
        geometry_msgs
        joint_state_publisher
        nav_msgs
        robot_state_publisher
        rospy
        # serial
        std_msgs
        tf
)

################################################################################
# Build

# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
# )

################################################################################
# Install

catkin_install_python(PROGRAMS
    nodes/curio_base_controller
    scripts/lx16a_cmd_vel_random.py
    scripts/lx16a_cmd_vel_sinusoid.py
    scripts/lx16a_cmd_vel_stepped.py
    scripts/lx16a_driver_test.py
    scripts/lx16a_encoder_filter_test.py
    scripts/lx16a_encoder_logger.py
    scripts/lx16a_odometry_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
