cmake_minimum_required(VERSION 2.8.3)
project(curio_navigation)

find_package(catkin REQUIRED roslaunch)

catkin_package()

# roslaunch_add_file_check(launch)

install(
    DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
