cmake_minimum_required(VERSION 2.8.3)
project(ackermann_drive_controller)


# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages
find_package(catkin REQUIRED COMPONENTS
    controller_interface
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    pluginlib
    realtime_tools
    roscpp
    std_msgs
    tf
    urdf
)

generate_dynamic_reconfigure_options(config/AckermannDriveController.cfg)

################################################################################
# Declare catkin configuration
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ackermann_drive_controller
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(ackermann_drive_controller
    src/ackermann_drive_controller.cpp
    src/odometry.cpp
    src/speed_limiter.cpp
)
target_link_libraries(ackermann_drive_controller ${catkin_LIBRARIES})
add_dependencies(ackermann_drive_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

################################################################################
# Install

install(TARGETS ackermann_drive_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES ackermann_drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
