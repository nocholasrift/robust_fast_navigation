#pragma once

#include <thread>
#include <memory>
#include <string>
#include <vector>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

// ROS 2 Nav Stack (Costmap2D)
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

// ROS 2 Messages
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// ROS 2 Services
#include "std_srvs/srv/empty.hpp"

#include "robust_fast_navigation/map_util.h"
#include "robust_fast_navigation/planner_core.h"

class PlannerROS : public rclcpp::Node {
public:
  PlannerROS();
  virtual ~PlannerROS();

  void spin();
  void visualizeTraj();
  bool plan(bool is_failsafe = false);

private:
  // ROS 2 Callbacks (using SharedPtr)
  void odomcb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void mapcb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void lasercb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void goalcb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void clickedPointcb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void mpcHorizoncb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  // Timers (ROS 2 uses WallTimer)
  void goalLoop();
  void controlLoop();
  void mapPublisher();
  void publishOccupied();

  trajectory_msgs::msg::JointTrajectory convertTrajToMsg(
    const std::vector<rfn_state_t> &trajectory, 
    double traj_dt, 
    std::string_view frame_str,
    const rclcpp::Time & current_time
  );

  // State Variables
  Eigen::VectorXd _odom;
  Eigen::VectorXd _vel;
  Eigen::VectorXd _goal_vec;

  bool _mpc_backwards;
  bool _use_arclen;
  bool _is_init;
  bool _use_global_costmap;
  bool _started_costmap;
  bool _is_goal_set;
  bool _is_teleop;
  bool _stop_planning;
  bool _is_goal_reset;
  bool _plan_once;
  bool _simplify_jps;
  bool _jps_hysteresis;
  bool _is_costmap_started;
  bool _is_grid_map_started;
  bool _map_received;
  bool _is_barn;
  bool _plan_in_free;
  bool _planned;
  bool _is_occ;
  bool _primitive_started;
  bool _use_minvo;
  bool _force_final_const;
  bool _is_drone;
  bool _use_mpc;

  std::string _frame_str;
  std::string _solver_str;

  trajectory_msgs::msg::JointTrajectory sentTraj;
  trajectory_msgs::msg::JointTrajectory mpcHorizon;

  Planner _planner;
  PlannerStatus _prev_plan_status;
  planner_params_t _planner_params;

  rclcpp::Time start, state_transition_start_t;
  
  // Timers
  rclcpp::TimerBase::SharedPtr controlTimer;
  rclcpp::TimerBase::SharedPtr goalTimer;
  rclcpp::TimerBase::SharedPtr publishTimer;
  rclcpp::TimerBase::SharedPtr safetyTimer;

  // Services (Clients)
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr estop_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr _mpc_backup_client;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr MPCHorizonSub;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr predictionsSub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mpcGoalReachedSub;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridMapPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajVizPub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goalPub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr paddedLaserPub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr jpsPub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr initPointPub;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr corridorPub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr initialPVAJPub;

  // Nav2 Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> _costmap;

  std::unique_ptr<map_util::occupancy_grid_t> _occ_grid;

  std::vector<float> _predictions;
  std::vector<Eigen::Vector2f> _obs;
  std::vector<Eigen::Vector2d> astarPath;
  std::vector<Eigen::Vector2d> _prev_jps_path;

  Eigen::MatrixXd _occ_point;

  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _costmap_executor;
  std::unique_ptr<std::thread> _costmap_thread;

  // Logic params
  double _max_vel, _max_acc, _max_jerk, _max_w, _dt;
  double _const_factor, _lookahead, _traj_dt, _mpc_dt;
  double _prev_jps_cost, _max_dist_horizon, _barn_goal_dist;
  double _curr_horizon, _max_dev, _solver_traj_dt;
  double _factor_init, _factor_final, _factor_increment, _max_solve_time;
  double _inflate_radius, _min_turn_clearance, _trigger_trim_dist, _flying_height;

  int _n_polys, _max_polys, _n_threads, _solver_verbose, _failsafe_count;

  nav_msgs::msg::OccupancyGrid map_msg;
};