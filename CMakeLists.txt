cmake_minimum_required(VERSION 3.16)
project(robust_fast_navigation)

if(NOT DEFINED ROS_VERSION)
  find_package(catkin QUIET)
  if(${catkin_FOUND})
    set(ROS_VERSION 1)
  else()
    find_package(rclcpp QUIET)
    if(${rclcpp_FOUND})
      set(ROS_VERSION 2)
    endif()
  endif()
endif()

add_compile_options(-std=c++17 -O3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(GUROBI REQUIRED)
find_package(GiNaC REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS system)

# Using GUROBI_LIBRARY on it's own isn't enough to get things to compile...
# Snippet taken from https://github.com/mit-acl/faster/blob/master/faster/CMakeLists.txt
FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};" )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
  ${GINAC_INCLUDE_DIRS}
)

list(APPEND PLANNER_SRCS
  src/JPS.cpp
  src/solver.cpp 
  src/rfn_types.cpp
  src/planner_core.cpp
  src/faster_wrapper.cpp
  src/gcopter_wrapper.cpp
)
list(APPEND PLANNER_LIBS
    ${GINAC_LIBRARIES}
    ${GUROBI_LIBRARIES}
    Eigen3::Eigen
)

add_library(planner_lib
  ${PLANNER_SRCS}
)

target_link_libraries(planner_lib
  ${PLANNER_LIBS}
)

if (ROS_VERSION EQUAL 1)
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

    grid_map_ros
    grid_map_sdf
    grid_map_core
    grid_map_costmap_2d
  )

  list(APPEND ROS_PLANNER_SRCS
    src/planner_gurobi.cpp 
    src/planner_gurobi_node.cpp 
  )
  list(APPEND ROS_PLANNER_LIBS
      ${catkin_LIBRARIES}
      planner_lib
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
  find_package(Ceres QUIET)
  if (Ceres_FOUND)
    if (TARGET Ceres::ceres)
      list(APPEND ROS_PLANNER_SRCS
        src/ph_demo.cpp
        src/contour_wrapper.cpp
      )
      list(APPEND ROS_PLANNER_LIBS
          Ceres::ceres
          ipopt
      )
      add_definitions(-DCERES_FOUND)
    else()
      message(STATUS "Ceres found, but not as a target. Using Ceres as a library.")
    endif()
  endif()

  find_package(mrs_msgs QUIET)
  if (mrs_msgs_FOUND)
    add_definitions(-DMRS_MSGS_FOUND)
    list(APPEND PLANNER_LIBS
      ${mrs_msgs_LIBRARIES}
    )
  else()
    message("mrs_msgs NOT FOUND")
  endif()




  catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robust_fast_navigation
#  CATKIN_DEPENDS geometry_msgs roscpp rospy
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
  )

  add_executable(planner
    ${ROS_PLANNER_SRCS}
  )
  target_link_libraries(planner
    ${ROS_PLANNER_LIBS}
  )

  add_executable(plan_through_map
    src/JPS.cpp
    src/solver.cpp
    src/rfn_types.cpp
    src/planner_core.cpp
    src/plan_through_map.cpp
    src/faster_wrapper.cpp
    src/gcopter_wrapper.cpp
  )
  target_link_libraries(plan_through_map
    ${PLANNER_LIBS}
  )

  add_executable(safety_node
      src/safety_node.cpp
  )
  target_link_libraries(safety_node
      ${catkin_LIBRARIES}
      Eigen3::Eigen
  )
endif()
