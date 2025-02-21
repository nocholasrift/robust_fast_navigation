cmake_minimum_required(VERSION 3.0.2)
project(robust_fast_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3 -g)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  rospy
  roscpp
  std_msgs
  costmap_2d
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(GUROBI REQUIRED)
find_package(GiNaC REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS system)

# Using GUROBI_LIBRARY on it's own isn't enough to get things to compile...
# Snippet taken from https://github.com/mit-acl/faster/blob/master/faster/CMakeLists.txt
FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};" )

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
  ${GUROBI_INCLUDE_DIRS}
  ${GINAC_INCLUDE_DIRS}
)

add_executable(planner
    src/JPS.cpp
    src/solver.cpp 
    src/planner_core.cpp
    src/planner_gurobi.cpp 
    src/planner_gurobi_node.cpp 
)
target_link_libraries(planner
    ${catkin_LIBRARIES}
    ${GINAC_LIBRARIES}
    ${GUROBI_LIBRARIES}
    Eigen3::Eigen
)

