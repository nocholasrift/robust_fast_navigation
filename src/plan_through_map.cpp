#include "robust_fast_navigation/plan_through_map.h"

#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <iterator>
#include <vector>

PlanNode::PlanNode(ros::NodeHandle &nh) {
  // ROS Params
  nh.param("map_planner/w_max", _max_w, 3.);
  nh.param("map_planner/v_max", _max_vel, 1.0);
  nh.param("map_planner/a_max", _max_acc, 1.2);
  nh.param("map_planner/j_max", _max_jerk, 4.0);
  nh.param("map_planner/traj_dt", _traj_dt, .1);
  nh.param("map_planner/is_barn", _is_barn, false);
  nh.param("map_planner/teleop", _is_teleop, false);
  nh.param("map_planner/is_drone", _is_drone, false);
  nh.param("map_planner/lookahead", _lookahead, .15);
  nh.param("map_planner/max_deviation", _max_dev, 1.);
  nh.param("map_planner/planner_frequency", _dt, .1);
  nh.param("map_planner/plan_once", _plan_once, false);
  nh.param("map_planner/use_arclen", _use_arclen, false);
  nh.param("map_planner/plan_in_free", _plan_in_free, false);
  nh.param("map_planner/simplify_jps", _simplify_jps, false);
  nh.param("map_planner/failsafe_count", _failsafe_count, 2);
  nh.param("map_planner/barn_goal_dist", _barn_goal_dist, 10.);
  nh.param<std::string>("map_planner/frame", _frame_str, "map");
  nh.param("map_planner/jps_hysteresis", _jps_hysteresis, false);
  nh.param("map_planner/max_dist_horizon", _max_dist_horizon, 4.);
  nh.param<std::string>("map_planner/solver", _solver_str, "faster");
  nh.param("map_planner/use_global_costmap", _use_global_costmap, true);
  nh.param("map_planner/trigger_trim_dist", _trigger_trim_dist, -100.);

  nh.param("map_planner/n_polys", _n_polys, 6);
  nh.param("map_planner/max_polys", _max_polys, 4);
  nh.param("map_planner/threads", _n_threads, 0);
  nh.param("map_planner/verbose", _solver_verbose, 0);
  nh.param("map_planner/use_minvo", _use_minvo, false);
  nh.param("map_planner/factor_init", _factor_init, 1.0);
  nh.param("map_planner/factor_final", _factor_final, 10.0);
  nh.param("map_planner/max_solve_time", _max_solve_time, .2);
  nh.param("map_planner/solver_traj_dt", _solver_traj_dt, .05);
  nh.param("map_planner/factor_increment", _factor_increment, 1.0);
  nh.param("map_planner/force_final_const", _force_final_const, true);
  nh.param("map_planner/min_turn_clearance", _min_turn_clearance, 0.1);

  // params
  _planner_params.SOLVER = _solver_str;

  _planner_params.W_MAX = _max_w;
  _planner_params.V_MAX = _max_vel;
  _planner_params.A_MAX = _max_acc;
  _planner_params.J_MAX = _max_jerk;
  _planner_params.DT_FACTOR_INIT = _factor_init;
  _planner_params.DT_FACTOR_FINAL = _factor_final;
  _planner_params.DT_FACTOR_INCREMENT = _factor_increment;
  _planner_params.SOLVER_TRAJ_DT = _solver_traj_dt;
  _planner_params.TRIM_DIST = _trigger_trim_dist;

  // this is actually number of polynomials in traj, not polys...
  _planner_params.N_SEGMENTS = _n_polys;
  _planner_params.MAX_POLYS = _max_polys;
  _planner_params.N_THREADS = _n_threads;
  _planner_params.FORCE_FINAL_CONSTRAINT = _force_final_const;
  _planner_params.VERBOSE = _solver_verbose;
  _planner_params.USE_MINVO = _use_minvo;
  _planner_params.PLAN_IN_FREE = _plan_in_free;
  _planner_params.SIMPLIFY_JPS = _simplify_jps;
  _planner_params.MAX_SOLVE_TIME = _max_solve_time;

  _planner.set_params(_planner_params);

  // Publishers
  trajVizPub = nh.advertise<visualization_msgs::Marker>("/MINCO_path", 0);
  trajPub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/reference_trajectory", 0);
  meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
  edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
  helperMeshPub =
      nh.advertise<visualization_msgs::Marker>("/visualizer/mesh_helper", 1000);
  helperEdgePub =
      nh.advertise<visualization_msgs::Marker>("/visualizer/edge_helper", 1000);
  goalPub = nh.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 0);
  paddedLaserPub = nh.advertise<visualization_msgs::Marker>("/paddedObs", 0);
  jpsPub = nh.advertise<nav_msgs::Path>("/jpsPath", 0);
  trajPubNoReset = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/reference_trajectory_no_reset", 0);

  // Subscribers
  goalSub = nh.subscribe("/planner_goal", 1, &PlanNode::goalcb, this);
  odomSub = nh.subscribe("/odometry/filtered", 1, &PlanNode::odomcb, this);

  // Timers
  _planLoop = nh.createTimer(ros::Duration(1.0), &PlanNode::plan_loop, this);

  _is_occ = false;
  _is_init = false;
  _planned = false;
  _is_goal_set = false;
  _map_received = false;
  _primitive_started = false;
  _is_costmap_started = false;
  _is_grid_map_started = false;
  _mpc_backwards = false;

  _prev_plan_status = SUCCESS;

  _prev_jps_cost = -1;
  _curr_horizon = _max_dist_horizon;

  ROS_INFO("Initialized planner!");

  sentTraj.points.clear();
}

void PlanNode::spin() {
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  if (_use_global_costmap)
    _costmap =
        std::make_unique<costmap_2d::Costmap2DROS>("global_costmap", tfBuffer);
  else
    _costmap =
        std::make_unique<costmap_2d::Costmap2DROS>("local_costmap", tfBuffer);

  _costmap->start();

  _is_costmap_started = true;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::waitForShutdown();
}

void PlanNode::odomcb(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  _odom = Eigen::VectorXd(3);
  _odom(0) = msg->pose.pose.position.x;
  _odom(1) = msg->pose.pose.position.y;
  _odom(2) = yaw;

  if (_is_barn && !_is_goal_set) {
    _goal = Eigen::VectorXd(2);
    _goal(0) = _barn_goal_dist * cos(yaw) + _odom(0);
    _goal(1) = _barn_goal_dist * sin(yaw) + _odom(1);
    _is_goal_set = true;

    ROS_INFO("yaw is %.4f", yaw);
    ROS_INFO("goal is %.4f\t%.4f", _goal(0), _goal(1));
  }

  _is_init = true;
}

void PlanNode::goalcb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  _goal = Eigen::VectorXd(2);
  _goal(0) = msg->pose.position.x;
  _goal(1) = msg->pose.position.y;

  _is_goal_set = true;
  _is_goal_reset = true;

  ROS_INFO("goal received!");
}

void PlanNode::visualizeTraj(const std::vector<rfn_state_t> &states) {
  if (sentTraj.points.size() > 0) {
    ros::Time start = ros::Time::now();

    if (states.size() == 0) {
      ROS_WARN("arc length reparameterization failed!");
      return;
    }

    std::vector<double> ss, xs, ys;
    ss.resize(states.size());
    xs.resize(states.size());
    ys.resize(states.size());

    visualization_msgs::Marker sample_msg;
    sample_msg.header.frame_id = _frame_str;
    sample_msg.header.stamp = ros::Time::now();
    sample_msg.ns = "sampemsg";
    sample_msg.id = 870;
    sample_msg.action = visualization_msgs::Marker::ADD;
    sample_msg.type = visualization_msgs::Marker::POINTS;
    sample_msg.scale.x = .05;
    sample_msg.scale.y = .05;
    sample_msg.pose.orientation.w = 1;
    sample_msg.color.r = 1;
    sample_msg.color.a = 1;

    for (int i = 0; i < states.size(); ++i) {
      xs[i] = states[i].pos(0);
      ys[i] = states[i].pos(1);
      ss[i] = states[i].t;

      geometry_msgs::Point point_msg;
      point_msg.x = xs[i];
      point_msg.y = ys[i];
      point_msg.z = 0.2;

      sample_msg.points.push_back(point_msg);
    }

    // std::vector<spline_t> d0 = _planner.get_tube();

    tk::spline splineX(ss, xs, tk::spline::cspline);
    tk::spline splineY(ss, ys, tk::spline::cspline);

    visualization_msgs::Marker arclenmsg;
    arclenmsg.header.frame_id = _frame_str;
    arclenmsg.header.stamp = ros::Time::now();
    arclenmsg.ns = "planTraj";
    arclenmsg.id = 80;
    arclenmsg.action = visualization_msgs::Marker::ADD;
    arclenmsg.type = visualization_msgs::Marker::LINE_STRIP;
    arclenmsg.scale.x = .1;
    arclenmsg.pose.orientation.w = 1;

    for (double s = 0; s < ss.back(); s += .1) {
      // get point and tangent to curve
      double px = splineX(s);
      double py = splineY(s);

      double tx = splineX.deriv(1, s);
      double ty = splineY.deriv(1, s);

      std_msgs::ColorRGBA color_msg;
      color_msg.r = 0.0;
      color_msg.g = 1.0;
      color_msg.b = 0.0;
      color_msg.a = 1.0;
      arclenmsg.colors.push_back(color_msg);

      geometry_msgs::Point point_msg;
      point_msg.x = px;
      point_msg.y = py;
      point_msg.z = 0.0;
      arclenmsg.points.push_back(point_msg);
    }

    trajVizPub.publish(arclenmsg);
  }
}

void PlanNode::plan_loop(const ros::TimerEvent &) {
  if (!_is_init || !_is_goal_set || _planned)
    return;

  std::vector<rfn_state_t> full_traj;
  std::vector<rfn_state_t> arclen_traj;
  for (int i = 0; i < 100; ++i) {

    Eigen::MatrixXd initialPVAJ(3, 4);

    // set initial point and run planner
    if (full_traj.size() == 0) {

      initialPVAJ << Eigen::Vector3d(_odom(0), _odom(1), 0),
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero();

      std::vector<rfn_state_t> traj = plan(initialPVAJ, 0.);

      full_traj.insert(full_traj.end(), traj.begin(), traj.end());

      std::vector<rfn_state_t> arc_traj = _planner.get_arclen_traj();
      arclen_traj.insert(arclen_traj.end(), arc_traj.begin(), arc_traj.end());

    } else {
      if (full_traj.size() < 2)
        continue;

      rfn_state_t initial_pt = full_traj[full_traj.size() - 1];

      initialPVAJ << initial_pt.pos, initial_pt.vel, initial_pt.accel,
          initial_pt.jerk;

      std::vector<rfn_state_t> traj = plan(initialPVAJ, full_traj.back().t);

      full_traj.insert(full_traj.end(), traj.begin() + 1, traj.end());

      std::vector<rfn_state_t> arc_traj = _planner.get_arclen_traj();
      for (rfn_state_t &x : arc_traj) {
        x.t += arclen_traj.back().t;
      }
      arclen_traj.insert(arclen_traj.end(), arc_traj.begin() + 1,
                         arc_traj.end());
    }

    ROS_INFO("%d: TRAJ SIZE %lu", i, full_traj.size());

    // if reached final goal, done
    if (full_traj.size() > 0) {

      if ((full_traj.back().pos.head(2) - _goal).norm() < 1e-1)
        break;
    }
  }

  sentTraj.points.clear();
  sentTraj.header.stamp = ros::Time::now();
  sentTraj.header.frame_id = _frame_str;

  double s_offset = 0;
  // std::vector<rfn_state_t> arclen_traj = _planner.get_arclen_traj(full_traj);

  for (rfn_state_t &x : arclen_traj) {
    // time from start is actually arc len in this case...
    /*ROS_INFO("%.2f: x pos is: %.2f\t%.2f", x.t, x.pos(0), x.pos(1));*/
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(x.pos(0));
    p.positions.push_back(x.pos(1));
    p.velocities.push_back(x.vel(0));
    p.velocities.push_back(x.vel(1));
    p.time_from_start = ros::Duration(x.t + s_offset);

    sentTraj.points.push_back(p);
  }

  trajPub.publish(sentTraj);
  visualizeTraj(arclen_traj);

  _planned = true;
}

std::vector<rfn_state_t> PlanNode::plan(const Eigen::MatrixXd &initialPVAJ,
                                        double start_t) {

  std::vector<rfn_state_t> traj;

  // update costmap
  if (!_is_grid_map_started) {
    _occ_grid =
        std::make_unique<map_util::occupancy_grid_t>(*_costmap->getCostmap());
    _is_grid_map_started = true;
  } else
    _occ_grid->update(*_costmap->getCostmap());

  Eigen::MatrixXd finalPVAJ(3, 4);

  finalPVAJ << Eigen::Vector3d(_goal(0), _goal(1), 0), Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

  ros::Time a = ros::Time::now();
  double p_start_t = 0.;

  // if velocity is 0, set to small value in direction of robot heading

  /*************************************
  **************** PLAN ****************
  **************************************/

  ROS_INFO_STREAM("initialPVAJ: " << initialPVAJ);

  /*map_util::occupancy_grid_t occ_grid =*/
  /*map_util::costmap_to_occgrid(*global_costmap->getCostmap());*/

  _planner.set_costmap(*_occ_grid);
  _planner.set_start(initialPVAJ);
  _planner.set_goal(finalPVAJ);

  std::vector<Eigen::Vector2d> jpsPath;

  ROS_INFO_STREAM("GOAL IS " << finalPVAJ.col(0).transpose());
  ROS_INFO("horizon is %.2f", _curr_horizon);

  std::vector<Eigen::MatrixX4d> hPolys;
  ros::Time before = ros::Time::now();
  _prev_plan_status = _planner.plan(_curr_horizon, jpsPath, hPolys);
  std::cout << "planner finished in " << (ros::Time::now() - before).toSec()
            << " with status" << _prev_plan_status << std::endl;
  if (_prev_plan_status) {
    ROS_WARN("Planner failed to find path");
    // check if initial velocity, acceleration, or jerk are higher than max
    double vel = initialPVAJ.col(1).norm();
    double acc = initialPVAJ.col(2).norm();
    double jerk = initialPVAJ.col(3).norm();
    ROS_WARN("initial conditions: %.2f\t%.2f\t%.2f!", vel, acc, jerk);
  }

  visualizePolytope(hPolys, meshPub, edgePub);

  /*************************************
  ******** PUBLISH JPS TO RVIZ *********
  **************************************/

  nav_msgs::Path jpsMsg;
  jpsMsg.header.stamp = ros::Time::now();
  jpsMsg.header.frame_id = _frame_str;

  for (Eigen::Vector2d p : jpsPath) {
    geometry_msgs::PoseStamped pMsg;
    pMsg.header = jpsMsg.header;
    pMsg.pose.position.x = p(0);
    pMsg.pose.position.y = p(1);
    pMsg.pose.position.z = 0;
    pMsg.pose.orientation.w = 1;
    jpsMsg.poses.push_back(pMsg);
  }

  jpsPub.publish(jpsMsg);
  ROS_INFO("published jps path: %lu", jpsPath.size());

  /*************************************
  ******** STITCH  TRAJECTORIES ********
  **************************************/

  std::vector<rfn_state_t> planned_trajectory = _planner.get_trajectory();

  for (rfn_state_t &x : planned_trajectory) {
    x.t += start_t;
  }

  if (planned_trajectory.size() == 0) {
    ROS_WARN("planned trajectory is empty!");
    return traj;
  }

  double totalT = (ros::Time::now() - a).toSec();

  return planned_trajectory;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_planner");
  ros::NodeHandle nh;

  PlanNode planner(nh);
  planner.spin();

  return 0;
}
