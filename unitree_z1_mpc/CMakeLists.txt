cmake_minimum_required(VERSION 3.0.2)
project(unitree_z1_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ocs2_core
  ocs2_mpc
  ocs2_ros_interfaces
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unitree_z1_mpc
 CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
AUX_SOURCE_DIRECTORY(src/ src_file)
add_library(src_lib
  ${src_file}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(lib)#UNITREE UDP

add_executable( 
  policy_to_real examples/policy_to_real.cpp
)

target_link_libraries(policy_to_real
  ${catkin_LIBRARIES}
  Z1_SDK_${CMAKE_SYSTEM_PROCESSOR}
  src_lib
  pthread
)