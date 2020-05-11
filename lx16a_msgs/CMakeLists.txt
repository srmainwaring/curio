cmake_minimum_required(VERSION 2.8.3)

project(lx16a_msgs)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

################################################################################
# Generate messages, services, actions

add_service_files(DIRECTORY srv FILES
    EncoderFilterAdd.srv
    EncoderFilterGetAngularPosition.srv
    EncoderFilterGetCount.srv
    EncoderFilterGetDuty.srv
    EncoderFilterGetInvert.srv
    EncoderFilterGetRevolutions.srv
    EncoderFilterGetServoPos.srv
    EncoderFilterReset.srv
    EncoderFilterSetInvert.srv
    EncoderFilterUpdate.srv
    EncoderFilterAddV.srv
    EncoderFilterUpdateV.srv
)

generate_messages(DEPENDENCIES std_msgs)

################################################################################
# Declare catkin configuration

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
)

################################################################################

