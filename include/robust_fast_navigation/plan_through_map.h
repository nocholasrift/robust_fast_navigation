#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <robust_fast_navigation/planner_core.h>
#include <robust_fast_navigation/utils.h>

class PlanNode {
public:
  PlanNode(ros::NodeHandle &nh);

  void spin();
  std::vector<rfn_state_t> plan(const Eigen::MatrixXd &initialPVAJ,
                                double start_t);

  void visualizeTraj(const std::vector<rfn_state_t> &states);

private:
  void odomcb(const nav_msgs::Odometry::ConstPtr &msg);
  void goalcb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void plan_loop(const ros::TimerEvent &);

  std::string _frame_str;
  std::string _solver_str;

  // trajectory generator
  double _max_vel;
  double _max_acc;
  double _max_jerk;
  double _max_w;
  double _dt;
  double _const_factor;
  double _lookahead;
  double _traj_dt;
  double _mpc_dt;
  double _prev_jps_cost;
  double _max_dist_horizon;
  double _barn_goal_dist;
  double _curr_horizon;
  double _max_dev;
  double _solver_traj_dt;
  double _factor_init;
  double _factor_final;
  double _factor_increment;
  double _max_solve_time;

  double _inflate_radius;
  double _min_turn_clearance;
  double _trigger_trim_dist;
  double _flying_height;

  int _n_polys;
  int _max_polys;
  int _n_threads;
  int _solver_verbose;
  int _failsafe_count;

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

  Eigen::VectorXd _goal;
  Eigen::VectorXd _odom;

  Planner _planner;
  PlannerStatus _prev_plan_status;
  planner_params_t _planner_params;

  ros::Subscriber odomSub;
  ros::Subscriber goalSub;

  trajectory_msgs::JointTrajectory sentTraj;

  ros::Publisher trajVizPub;
  ros::Publisher trajPub;
  ros::Publisher trajPubNoReset;
  ros::Publisher meshPub;
  ros::Publisher edgePub;
  ros::Publisher helperMeshPub;
  ros::Publisher helperEdgePub;
  ros::Publisher goalPub;
  ros::Publisher paddedLaserPub;
  ros::Publisher jpsPub;

  ros::Timer _planLoop;

  std::unique_ptr<costmap_2d::Costmap2DROS> _costmap;
  std::unique_ptr<map_util::occupancy_grid_t> _occ_grid;
};
