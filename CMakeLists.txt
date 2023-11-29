cmake_minimum_required(VERSION 3.0.2)
project(robust_fast_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O2 -g)

# Used to tell cmake where FindGUROBI.cmake is
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Using GUROBI_LIBRARY on it's own isn't enough to get things to compile...
# Snippet taken from https://github.com/mit-acl/faster/blob/master/faster/CMakeLists.txt
FILE(GLOB GurobiSOFiles $ENV{GUROBI_HOME}/lib/libgurobi*[0-9]*.so) #files that are start with libgurobi and end with number.so
set(GUROBI_LIBRARIES "$ENV{GUROBI_HOME}/lib/libgurobi_c++.a;${GurobiSOFiles};" )


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
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
find_package(drake REQUIRED)
find_package(GUROBI REQUIRED)
find_package(GiNaC REQUIRED)

find_package(jps3d REQUIRED)
find_package(octomap REQUIRED)

add_message_files(
      DIRECTORY
      FILES
      SolverState.msg
      SolverStateArray.msg
)

add_service_files(
  FILES
  SolveFromState.srv
)

generate_messages(
      DEPENDENCIES 
      nav_msgs
      geometry_msgs 
      trajectory_msgs
)

catkin_package(
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
  ${GUROBI_INCLUDE_DIRS}
  ${GINAC_INCLUDE_DIRS}

  ${JPS3D_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(robust_planner src/planner.cpp src/planner_node.cpp src/JPS.cpp)
target_link_libraries(robust_planner
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  drake::drake
)
add_dependencies(robust_planner ${PROJECT_NAME}_generate_messages)

add_executable(robust_planner_gurobi src/planner_gurobi.cpp src/solver.cpp src/planner_gurobi_node.cpp src/JPS.cpp)
target_link_libraries(robust_planner_gurobi
  ${catkin_LIBRARIES}
  ${GUROBI_LIBRARIES}
  ${GINAC_LIBRARIES}
  ${CLN_LIBRARIES}
  Eigen3::Eigen
  drake::drake
)
add_dependencies(robust_planner_gurobi ${PROJECT_NAME}_generate_messages)

add_executable(brs_manager src/BRSManager.cpp src/JPS.cpp)
target_link_libraries(brs_manager
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  drake::drake
)

add_executable(publish_pf_pose src/publish_pf_pose.cpp)
target_link_libraries(publish_pf_pose
  ${catkin_LIBRARIES}
)

add_executable(primitives src/motion_primitives.cpp)
target_link_libraries(primitives
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_executable(solver_server src/solve_from_state_server.cpp src/solver.cpp src/JPS.cpp)
target_link_libraries(solver_server
  ${catkin_LIBRARIES}
  ${GUROBI_LIBRARIES}
  ${GINAC_LIBRARIES}
  ${CLN_LIBRARIES}
  drake::drake
  Eigen3::Eigen
)
add_dependencies(solver_server ${PROJECT_NAME}_generate_messages)

add_executable(minvo_test src/minvo_basis.cpp)
target_link_libraries(minvo_test
  ${catkin_LIBRARIES}
  ${GINAC_LIBRARIES}
  ${CLN_LIBRARIES}
  Eigen3::Eigen
)

add_executable(jps_test src/JPS_test.cpp src/JPS3D.cpp)
target_link_libraries(jps_test
  ${catkin_LIBRARIES}
  ${JPS3D_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)
