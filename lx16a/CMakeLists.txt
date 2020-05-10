cmake_minimum_required(VERSION 2.8.3)
project(lx16a)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    lx16a_msgs
    roscpp
    rospy
    serial
    std_msgs
)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Declare catkin configuration

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        lx16a
    CATKIN_DEPENDS
        lx16a_msgs
        message_runtime
        roscpp
        rospy
        serial
        std_msgs
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Library
add_library(lx16a
    src/lx16a_encoder_filter_client.cpp
    src/lx16a_encoder_filter_onnx.cpp
    src/lx16a_encoder_filter.cpp
    src/lx16a_driver.cpp
)
target_link_libraries(lx16a ${catkin_LIBRARIES})

# Examples
add_executable(lx16a_position_publisher
    src/examples/lx16a_position_publisher.cpp
)
target_link_libraries(lx16a_position_publisher lx16a ${catkin_LIBRARIES})

add_executable(lx16a_driver_test
    src/examples/lx16a_driver_test.cpp
)
target_link_libraries(lx16a_driver_test lx16a ${catkin_LIBRARIES})

add_executable(lx16a_encoder_filter_test
    src/examples/lx16a_encoder_filter_test.cpp
)
target_link_libraries(lx16a_encoder_filter_test lx16a ${catkin_LIBRARIES})

################################################################################
# Install

install(TARGETS
    lx16a
    lx16a_position_publisher
    lx16a_driver_test
    lx16a_encoder_filter_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/lx16a/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
    nodes/lx16a_encoder_filter_server
    scripts/lx16a_cmd_vel_random.py
    scripts/lx16a_cmd_vel_sinusoid.py
    scripts/lx16a_cmd_vel_stepped.py
    scripts/lx16a_driver_test.py
    scripts/lx16a_encoder_filter_test.py
    scripts/lx16a_encoder_logger.py
    scripts/lx16a_failsafe_test.py
    scripts/lx16a_odometry_test.py
    scripts/lx16a_train_classifier.py
    scripts/lx16a_train_regressor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
