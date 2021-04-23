cmake_minimum_required(VERSION 2.8.3)
project(armor)

## Torch 需要至少C++14进行编译
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 3 not found.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT Eigen3_FOUND)
   message(FATAL_ERROR "Eigen3 not found.")
endif()

find_package(Pangolin REQUIRED)
if(NOT Pangolin_FOUND)
   message(FATAL_ERROR "Pangolin not found.")
endif()

find_package(Ceres REQUIRED)
if(NOT Ceres_FOUND)
   message(FATAL_ERROR "Ceres not found.")
endif()

set(Torch_DIR "/opt/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
if(NOT Torch_FOUND)
   message(FATAL_ERROR "Torch not found.")
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  #geometry_msgs
  #cv_bridge
  #image_transport
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES armor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/armor.cpp
# )

link_directories(
  /opt/MVS/lib/64
  /opt/MVS/lib/32
  /usr/local/lib
#   ${TORCH_LIBRARY_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../serial_com/include
  /opt/MVS/include
  ./include/
  ./include/armor
)

aux_source_directory(src/distance DIS_SRC)
aux_source_directory(src/aim_deps DEPS_SRC)
aux_source_directory(src/aim_cam CAM_SRC)
aux_source_directory(src/aim_dec DEC_SRC)
aux_source_directory(src/aim_pos POS_SRC)
aux_source_directory(src/aim_pre PRE_SRC)
aux_source_directory(src/aim_rec REC_SRC)
aux_source_directory(src/Sentrydecision SENTRY_SRC)
aux_source_directory(src/vicinity VIC_SRC)
aux_source_directory(src/aim_energy ENE_SRC)

add_executable(Frame Frame.cc
  ${DIS_SRC}
  ${CAM_SRC}
  ${DEC_SRC}
  ${POS_SRC}
  ${PRE_SRC}
  ${REC_SRC}
  ${VIC_SRC}
  ${DEPS_SRC}
  ${SENTRY_SRC}
  ${ENE_SRC}
)

add_dependencies(Frame ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Frame
  MvCameraControl
  libMvCameraControl.so
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  pthread
)
