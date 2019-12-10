cmake_minimum_required(VERSION 2.8.3)
project(sawppy_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    rospy
    # serial
    # std_msgs
)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Declare catkin configuration

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES 
    CATKIN_DEPENDS
        rospy
        # serial
        # std_msgs
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


################################################################################
# Install

catkin_install_python(PROGRAMS
    scripts/sawppy_servo_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
