cmake_minimum_required(VERSION 2.8.3)
project(curio_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    rospy
    # serial
    geometry_msgs
    std_msgs
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
        rospy
        # serial
        geometry_msgs
        std_msgs
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
    nodes/curio_motor_controller
    scripts/lx16a_driver_test.py
    scripts/lx16a_encoder_calibration.py
    scripts/lx16a_make_training_data.py
    scripts/lx16a_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
