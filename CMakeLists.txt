cmake_minimum_required(VERSION 3.0.2)
project(robust_fast_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O2 -g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  roscpp
  rospy
  costmap_2d
  decomp_util
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robust_fast_navigation
#  CATKIN_DEPENDS geometry_msgs roscpp rospy
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS}
)

add_executable(robust_planner src/planner.cpp src/planner_node.cpp src/JPS.cpp)
target_link_libraries(robust_planner
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(brs_manager src/BRSManager.cpp)
target_link_libraries(brs_manager
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(publish_pf_pose src/publish_pf_pose.cpp)
target_link_libraries(publish_pf_pose
  ${catkin_LIBRARIES}
)
