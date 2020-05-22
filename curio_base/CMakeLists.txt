cmake_minimum_required(VERSION 2.8.3)
project(curio_base)

################################################################################
# Preprocessor options
option(CURIO_WITH_PYTHON_HAL "Use the Python HAL" OFF)

if(CURIO_WITH_PYTHON_HAL)
    add_definitions(-DCURIO_USE_PYTHON_HAL=1)
else()
    add_definitions(-DCURIO_USE_PYTHON_HAL=0)
endif()

################################################################################
# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    controller_manager
    curio_description
    curio_control
    curio_msgs
    geometry_msgs
    hardware_interface
    joint_state_publisher
    lx16a
    lx16a_msgs
    message_generation
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

find_package(pybind11 REQUIRED)

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
        lx16a
        lx16a_msgs
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

# Library
add_library(curio_base
    src/rover_base_hal.cpp
    src/rover_base_hal_lx16a.cpp
    src/rover_base_hal_mock.cpp
    src/rover_base_hal_python.cpp
    src/rover_base_hardware.cpp
)
target_link_libraries(curio_base pybind11::embed ${catkin_LIBRARIES})
add_dependencies(curio_base ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Python bindings
pybind11_add_module(_curio_base
    src/pybind11/py_curio_base.cpp
    src/pybind11/py_rover_base_hal.cpp
    src/pybind11/py_time.cpp
)
target_link_libraries(_curio_base PRIVATE curio_base ${catkin_LIBRARIES})
set_target_properties(_curio_base PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

# Nodes
add_executable(rover_base_controller
    src/rover_base_controller.cpp
)
target_link_libraries(rover_base_controller curio_base ${catkin_LIBRARIES})

# Examples
add_executable(ros_serial_test
    src/examples/ros_serial_test.cpp
)
target_link_libraries(ros_serial_test curio_base ${catkin_LIBRARIES})

add_executable(serial_test
    src/examples/serial_test.cpp
)
target_link_libraries(serial_test curio_base ${catkin_LIBRARIES})

add_executable(test_rover_base_hal_lx16a
    test/test_rover_base_hal_lx16a.cpp
)
target_link_libraries(test_rover_base_hal_lx16a curio_base ${catkin_LIBRARIES})

################################################################################
# Install

install(TARGETS
    curio_base
    serial_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/curio_base/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
    nodes/curio_base_controller
    nodes/curio_base_failsafe
    nodes/curio_base_sensors
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config data launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
