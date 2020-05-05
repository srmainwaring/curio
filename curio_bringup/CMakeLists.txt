cmake_minimum_required(VERSION 2.8.3)
project(curio_bringup)

# Find catkin macros and libraries
find_package(catkin REQUIRED
)

# Catkin specific configuration
catkin_package(
    CATKIN_DEPENDS
    curio_base
    curio_description
    curio_teleop
)

# Install
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
