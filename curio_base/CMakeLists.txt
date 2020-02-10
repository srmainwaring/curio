cmake_minimum_required(VERSION 2.8.3)
project(curio_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    curio_description
    curio_control
    curio_msgs
    geometry_msgs
    joint_state_publisher
    nav_msgs
    message_generation
    robot_localization
    robot_state_publisher
    roscpp
    rospy
    serial
    sensor_msgs
    std_msgs
    tf
)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Generate messages, services, actions

add_action_files(DIRECTORY action FILES
    CalibrateImu.action
)

# add_message_files(DIRECTORY msg FILES
# )

# add_service_files(DIRECTORY srv FILES
# )

generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

################################################################################
# Declare catkin configuration

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        curio_base
    CATKIN_DEPENDS
        actionlib_msgs
        curio_description
        curio_control
        curio_msgs
        geometry_msgs
        joint_state_publisher
        message_runtime
        nav_msgs
        robot_localization
        robot_state_publisher
        roscpp
        rospy
        serial
        sensor_msgs
        std_msgs
        tf
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(curio_base
    src/lx16a_driver.cpp
)
target_link_libraries(curio_base ${catkin_LIBRARIES})

add_executable(lx16a_position_publisher
    src/examples/lx16a_position_publisher.cpp
)
target_link_libraries(lx16a_position_publisher curio_base ${catkin_LIBRARIES})

################################################################################
# Install

install(TARGETS
    curio_base
    lx16a_position_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/curio_base/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
    nodes/curio_base_controller
    nodes/curio_base_failsafe
    nodes/curio_base_sensors
    scripts/lx16a_cmd_vel_random.py
    scripts/lx16a_cmd_vel_sinusoid.py
    scripts/lx16a_cmd_vel_stepped.py
    scripts/lx16a_driver_test.py
    scripts/lx16a_encoder_filter_test.py
    scripts/lx16a_encoder_logger.py
    scripts/lx16a_failsafe_test.py
    scripts/lx16a_mean_filter_test.py
    scripts/lx16a_odometry_test.py
    scripts/lx16a_train_classifier.py
    scripts/lx16a_train_regressor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
