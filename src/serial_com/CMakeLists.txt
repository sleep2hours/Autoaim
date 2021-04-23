cmake_minimum_required(VERSION 2.8.3)
project(serial_com)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
  /opt/MVS/include
  include/serial_com
)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  rospy
  message_generation
  std_msgs
  serial
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  comm.msg
  #parameters.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
#  INCLUDE_DIRS include
#  LIBRARIES serial_com
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/serial_com.cpp
# )

aux_source_directory(src/ PROJ_SRC)

add_executable(stm_com
  stm_com.cc
  ${PROJ_SRC}
)
add_dependencies(stm_com ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stm_com
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(simulator
  simulator.cc
  ${PROJ_SRC}
)
add_dependencies(simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simulator
  ${catkin_LIBRARIES}
)