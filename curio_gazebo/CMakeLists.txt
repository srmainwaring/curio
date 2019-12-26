cmake_minimum_required(VERSION 2.8.3)
project(curio_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
