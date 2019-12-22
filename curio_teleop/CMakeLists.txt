cmake_minimum_required(VERSION 2.8.3)
project(curio_teleop)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roslaunch
    roscpp
    std_msgs
    curio_msgs
)

################################################################################
# Declare catkin configuration

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES wild1_base
    CATKIN_DEPENDS
        geometry_msgs
        roscpp
        std_msgs
        curio_msgs
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(curio_teleop_rc_node
    src/curio_teleop_rc_node.cpp
)
target_link_libraries(curio_teleop_rc_node ${catkin_LIBRARIES})

################################################################################
# Install

install(TARGETS curio_teleop_rc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

