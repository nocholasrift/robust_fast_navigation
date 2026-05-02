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

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(GiNaC REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS system)

list(APPEND PLANNER_SRCS
  src/JPS.cpp
  src/planner_core.cpp
  src/gcopter_wrapper.cpp
)
list(APPEND PLANNER_LIBS
    ${GINAC_LIBRARIES}
    Eigen3::Eigen
)
list(APPEND PLANNER_INCLUDE_DIRS
  include
  ${GINAC_INCLUDE_DIRS}
)


########################
# Gurobi (optional)
########################

find_package(GUROBI QUIET)
if(GUROBI_FOUND)
  message(STATUS "Gurobi found, building with Gurobi support")
  add_definitions(-DGUROBI_FOUND)

  add_library(gurobi_planner 
    src/solver.cpp 
    src/rfn_types.cpp
    src/faster_wrapper.cpp
  )
  target_link_libraries(gurobi_planner PRIVATE
    ${PLANNER_LIBS}
    ${GUROBI_LIBRARIES}
  )
  target_include_directories(gurobi_planner PRIVATE
    ${PLANNER_INCLUDE_DIRS}
    ${GUROBI_INCLUDE_DIRS}
  )

  list(APPEND PLANNER_LIBS gurobi_planner)
  list(APPEND PLANNER_INCLUDE_DIRS ${GUROBI_INCLUDE_DIRS})

  ########################
  # Ceres (optional)
  ########################
  find_package(Ceres QUIET)
  if(Ceres_FOUND)
    if(TARGET Ceres::ceres)
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

else()
  message(STATUS "Gurobi not found, building without Gurobi support")
endif()

########################
# Core planner library
########################
add_library(planner_lib
  ${PLANNER_SRCS}
)
set_target_properties(planner_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(planner_lib
  ${PLANNER_LIBS}
)

target_include_directories(planner_lib PRIVATE
  ${PLANNER_INCLUDE_DIRS}
)

########################
# Python bindings (optional)
########################
option(BUILD_PYTHON_BINDINGS "Build pybind11 Python bindings" ON)

if(BUILD_PYTHON_BINDINGS)
  find_package(Python3 COMPONENTS Development QUIET)
  add_subdirectory(third_party/pybind11)

  if(Python3_Development_FOUND AND TARGET pybind11::module)
    message(STATUS "Pybind11 v${pybind11_VERSION} found, building py_planner Python bindings")
    add_definitions(-DFOUND_PYBIND11)

    pybind11_add_module(py_planner
      src/py_planner.cpp
    )

    target_link_libraries(py_planner
      PRIVATE
      planner_lib
      ${PLANNER_LIBS}
    )

    target_include_directories(py_planner PRIVATE
      ${PLANNER_INCLUDE_DIRS}
      ${Python3_INCLUDE_DIRS}
    )
  else()
    message(STATUS "Pybind11 or Python3 development headers not found, skipping Python bindings")
  endif()
else()
  message(STATUS "BUILD_PYTHON_BINDINGS=OFF, skipping Python bindings")
endif()


########################
# ROS 1
########################
if(ROS_VERSION EQUAL 1)
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

  list(APPEND ROS_PLANNER_SRCS
    planner_ros1/src/planner_gurobi.cpp
    planner_ros1/src/planner_gurobi_node.cpp
  )
  list(APPEND ROS_PLANNER_LIBS
    ${catkin_LIBRARIES}
    planner_lib
  )


  find_package(mrs_msgs QUIET)
  if(mrs_msgs_FOUND)
    add_definitions(-DMRS_MSGS_FOUND)
    list(APPEND ROS_PLANNER_LIBS ${mrs_msgs_LIBRARIES})
  else()
    message(STATUS "mrs_msgs not found")
  endif()

  catkin_package(
    CATKIN_DEPENDS message_runtime
  )

  add_executable(planner
    ${ROS_PLANNER_SRCS}
  )
  target_link_libraries(planner
    ${ROS_PLANNER_LIBS}
  )
  target_include_directories(planner PRIVATE
    ${PLANNER_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    planner_ros1/include
  )
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(trajectory_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(nav2_costmap_2d REQUIRED) # Equivalent to costmap_2d

  # Add your ROS 2 specific source files (e.g., your new ROS 2 wrapper/node)
  list(APPEND ROS2_PLANNER_SRCS
    planner_ros2/src/planner_ros2_node.cpp
    planner_ros2/src/planner_ros2.cpp 
  )

  add_executable(planner_node ${ROS2_PLANNER_SRCS})

  target_link_libraries(planner_node
    planner_lib
    ${PLANNER_LIBS}
  )

  ament_target_dependencies(planner_node
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    visualization_msgs
    tf2
    tf2_ros
    nav2_costmap_2d
  )

  target_include_directories(planner_node PRIVATE
    ${PLANNER_INCLUDE_DIRS}
    planner_ros2/include
  )

  # Standard ROS 2 Install path
  install(TARGETS planner_node
    DESTINATION lib/${PROJECT_NAME}
  )

  # install misc files
  install(DIRECTORY
    planner_ros2/launch
    planner_ros2/params
    DESTINATION share/${PROJECT_NAME}
  )

  # This is mandatory for the root of any ROS 2 package
  ament_package()
endif()
