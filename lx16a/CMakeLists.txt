cmake_minimum_required(VERSION 2.8.3)
project(lx16a)

################################################################################
# Preprocessor options
option(LX16A_WITH_DEBUG "Enable debug settings" OFF)

if(LX16A_WITH_DEBUG)
    add_definitions(-DLX16A_DEBUG=1)
else()
    add_definitions(-DLX16A_DEBUG=0)
endif()

################################################################################
# Compile as C++11, supported in ROS Kinetic and newer
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

find_package(pybind11 REQUIRED)
message("pybind11_FOUND: ${pybind11_FOUND}")
message("pybind11_VERSION: ${pybind11_VERSION}")
message("pybind11_INCLUDE_DIRS: ${pybind11_INCLUDE_DIRS}")
message("pybind11_INCLUDE_DIR: ${pybind11_INCLUDE_DIR}")
message("pybind11_DEFINITIONS: ${pybind11_DEFINITIONS}")
message("pybind11_LIBRARIES: ${pybind11_LIBRARIES}")
message("pybind11_LIBRARY: ${pybind11_LIBRARY}")


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
    ${pybind11_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# Library
add_library(lx16a
    src/lx16a_driver.cpp
    src/lx16a_encoder_filter_client.cpp
    src/lx16a_encoder_filter_onnx.cpp
    src/lx16a_encoder_filter_python.cpp
    src/lx16a_encoder_filter.cpp
    src/lx16a_exception.cpp
    src/lx16a_pybind11_embed.cpp
    src/lx16a_time.cpp
)
target_link_libraries(lx16a
    PRIVATE
    pybind11::embed
    ${pybind11_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Python bindings
pybind11_add_module(_lx16a
    src/pybind11/py_lx16a.cpp
    src/pybind11/py_lx16a_driver.cpp
    src/pybind11/py_lx16a_encoder_filter.cpp
    src/pybind11/py_lx16a_time.cpp
)
target_link_libraries(_lx16a PRIVATE lx16a ${pybind11_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(_lx16a PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION})

# Examples
add_executable(lx16a_position_publisher
    src/examples/lx16a_position_publisher.cpp
)
target_link_libraries(lx16a_position_publisher PRIVATE lx16a ${catkin_LIBRARIES})

add_executable(lx16a_driver_test
    src/examples/lx16a_driver_test.cpp
)
target_link_libraries(lx16a_driver_test PRIVATE lx16a ${catkin_LIBRARIES})

add_executable(lx16a_encoder_filter_test
    src/examples/lx16a_encoder_filter_test.cpp
)
target_link_libraries(lx16a_encoder_filter_test
  PRIVATE
  pybind11::embed
  lx16a
  ${pybind11_LIBRARIES}
  ${catkin_LIBRARIES}
)

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

install(DIRECTORY data launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)
    find_package(rosunit REQUIRED)

    # Generate test_config.h
    get_filename_component(TEST_DIR "test" ABSOLUTE)
    get_filename_component(DATA_DIR "data" ABSOLUTE)
    configure_file(test/config.in test/config.h)
    configure_file(test/config_py.in test/config.py)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

    # C++ tests
    catkin_add_gtest(test_encoder_filter test/test_encoder_filter.cpp)
    target_link_libraries(test_encoder_filter pybind11::embed lx16a ${catkin_LIBRARIES})

    # Python tests
    catkin_add_nosetests(test/test_encoder_filter.py)
    catkin_add_nosetests(test/test_ext_encoder_filter.py)
    catkin_add_nosetests(test/test_ext_time.py)

    # Python tests that require hardware configured
    # catkin_add_nosetests(test/test_ext_driver.py)

    # rostest tests
    catkin_add_gtest(test_hardware_encoder test/test_hardware_encoder.cpp)
    target_link_libraries(test_hardware_encoder lx16a ${catkin_LIBRARIES})


endif()
