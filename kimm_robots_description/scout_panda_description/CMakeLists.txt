cmake_minimum_required(VERSION 2.8.3)
project(scout_panda_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch)

install(
  DIRECTORY robots meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
