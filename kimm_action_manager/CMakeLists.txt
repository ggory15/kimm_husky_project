cmake_minimum_required(VERSION 3.0.2)
project(kimm_action_manager)
add_compile_options(-std=c++17 -Ofast)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )

OPTION(EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
  tf
  mujoco_ros_msgs
  message_generation
  actionlib_msgs
  actionlib
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
find_package(kimm_hqp_controller REQUIRED)
find_package(eiquadprog REQUIRED)

add_action_files(DIRECTORY action FILES 
  JointPosture.action
  SE3.action
  SE3Array.action
  Gripper.action
  Move.action
  QR.action
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    tf
    roscpp
    mujoco_ros_msgs
    message_runtime 
    std_msgs
  DEPENDS Franka
)

link_directories(${Boost_LIBRARY_DIR})

###########
## Build ##
###########

## Define OS Type
add_definitions(-D_OS_UNIX)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${kimm_hqp_controller_INCLUDE_DIRS}
  ${kimm_trajectory_smoother_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

SET(LIBRARY_NAME kimm_action_manager_base)
SET(${LIBRARY_NAME}_HEADERS
  include/kimm_action_manager/controller/controller.hpp

  include/kimm_action_manager/servers/action_server_base.hpp
  include/kimm_action_manager/servers/joint_posture_action_server.hpp
  include/kimm_action_manager/servers/se3_action_server.hpp
  include/kimm_action_manager/servers/se3_array_action_server.hpp
  include/kimm_action_manager/servers/move_action_server.hpp
  include/kimm_action_manager/servers/gripper_action_server.hpp
  include/kimm_action_manager/servers/qr_action_server.hpp
)

SET(${LIBRARY_NAME}_SOURCES
  src/controller/controller.cpp
  
  src/servers/joint_posture_action_server.cpp
  src/servers/se3_action_server.cpp
  src/servers/se3_array_action_server.cpp
  src/servers/move_action_server.cpp
  src/servers/gripper_action_server.cpp
  src/servers/qr_action_server.cpp
)


ADD_SOURCE_GROUP(${LIBRARY_NAME}_HEADERS)
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)
ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES} ${${LIBRARY_NAME}_HEADERS} )
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} )
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${kimm_hqp_controller_LIBRARIES} ${pinocchio_LIBRARIES} ${kimm_trajectory_smoother_LIBRARIES} eiquadprog::eiquadprog -lpthread)

# exe for simulation
add_executable(${PROJECT_NAME}_simulation src/simulation/simulation.cpp)
target_link_libraries(${PROJECT_NAME}_simulation ${LIBRARY_NAME} ${catkin_LIBRARIES} -lpthread)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_simulation  SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/real/real.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${LIBRARY_NAME}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

