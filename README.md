Polygonal-gap based motion planner for both aerial and ground vehicles
# Robust Fast Navigation

A high-performance, robust motion planning framework for autonomous navigation in unknown environments. This repository provides an implementation of a fast trajectory optimizer using Gurobi, designed for safe and efficient navigation.

- **Occupancy Grid-Based Planning**: Takes in a `nav_msgs/OccupancyGrid` from the `/map` topic.
- **Odometry Input**: Requires robot odometry before planning can commence.
- **Gurobi Solver**: Leverages Gurobi for efficient trajectory optimization.

## Installation

To install this package, you will first need to install Gurobi and get a WLS Gurobi license file from the [Gurobi Web License Manager](https://license.gurobi.com/manager/licenses). Building of the code can be done via `catkin build`

### Dependencies

Ensure you have the following dependencies installed:

- ROS (tested on ROS Noetic)
- Gurobi optimizer
- Eigen3
- CMake
- Python3
- `roscpp` and `rospy`
- `nav_msgs` for occupancy grid input
- GiNaC

## Running

The code can be run by running the following:

```bash
roslaunch robust_fast_navigation planner_gurobi.launch
```

