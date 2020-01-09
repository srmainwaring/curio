cmake_minimum_required(VERSION 2.8.3)
project(curio_teleop)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages
find_package(catkin REQUIRED COMPONENTS
    curio_msgs
    geometry_msgs
    roscpp
    roslaunch
    std_msgs
)

################################################################################
# Declare catkin configuration

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        curio_msgs
        geometry_msgs
        roscpp
        std_msgs
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(curio_teleop_rc
    src/curio_teleop_rc.cpp
)
target_link_libraries(curio_teleop_rc ${catkin_LIBRARIES})

################################################################################
# Install

install(TARGETS curio_teleop_rc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

