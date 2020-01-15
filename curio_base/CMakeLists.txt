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
    nodes/curio_base_controller
    scripts/lx16a_cmd_vel_random.py
    scripts/lx16a_cmd_vel_sinusoid.py
    scripts/lx16a_cmd_vel_stepped.py
    scripts/lx16a_driver_test.py
    scripts/lx16a_encoder_filter.py
    scripts/lx16a_encoder_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
