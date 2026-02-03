#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <grid_map_msgs/GridMap.h>
#include <math.h>
#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/planner_gurobi.h>
#include <robust_fast_navigation/spline.h>
#include <robust_fast_navigation/utils.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#ifdef MRS_MSGS_FOUND
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#endif

#include <cmath>
#include <iterator>
#include <string>
#include <unsupported/Eigen/Splines>
#include <vector>

#include "costmap_2d/cost_values.h"
#include "robust_fast_navigation/planner_core.h"
#include "ros/console.h"
#include "trajectory_msgs/JointTrajectory.h"

typedef Eigen::Spline<double, 1, 3> Spline1D;
typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;

/**********************************************************************
  Constructor to read in ROS params, setup subscribers & publishers,
  ros timers, and initialize class fields.
***********************************************************************/
PlannerROS::PlannerROS(ros::NodeHandle &nh)
{
    // ROS Params
    nh.param("robust_planner/w_max", _max_w, 3.);
    nh.param("robust_planner/v_max", _max_vel, 1.0);
    nh.param("robust_planner/a_max", _max_acc, 1.2);
    nh.param("robust_planner/j_max", _max_jerk, 4.0);
    nh.param("robust_planner/traj_dt", _traj_dt, .1);
    nh.param("robust_planner/is_barn", _is_barn, false);
    nh.param("robust_planner/teleop", _is_teleop, false);
    nh.param("robust_planner/is_drone", _is_drone, false);
    nh.param("robust_planner/lookahead", _lookahead, .15);
    nh.param("robust_planner/planner_frequency", _dt, .1);
    nh.param("robust_planner/max_deviation", _max_dev, 1.);
    nh.param("robust_planner/plan_once", _plan_once, false);
    nh.param("robust_planner/use_arclen", _use_arclen, false);
    nh.param("robust_planner/plan_in_free", _plan_in_free, false);
    nh.param("robust_planner/simplify_jps", _simplify_jps, false);
    nh.param("robust_planner/failsafe_count", _failsafe_count, 2);
    nh.param("robust_planner/barn_goal_dist", _barn_goal_dist, 10.);
    nh.param<std::string>("robust_planner/frame", _frame_str, "map");
    nh.param("robust_planner/jps_hysteresis", _jps_hysteresis, false);
    nh.param("robust_planner/max_dist_horizon", _max_dist_horizon, 4.);
    nh.param<std::string>("robust_planner/solver", _solver_str, "faster");
    nh.param("robust_planner/use_global_costmap", _use_global_costmap, true);
    nh.param("robust_planner/trigger_trim_dist", _trigger_trim_dist, -100.);

    nh.param("robust_planner/n_polys", _n_polys, 6);
    nh.param("robust_planner/max_polys", _max_polys, 4);
    nh.param("robust_planner/threads", _n_threads, 0);
    nh.param("robust_planner/verbose", _solver_verbose, 0);
    nh.param("robust_planner/use_minvo", _use_minvo, false);
    nh.param("robust_planner/factor_init", _factor_init, 1.0);
    nh.param("robust_planner/factor_final", _factor_final, 10.0);
    nh.param("robust_planner/max_solve_time", _max_solve_time, .2);
    nh.param("robust_planner/solver_traj_dt", _solver_traj_dt, .05);
    nh.param("robust_planner/factor_increment", _factor_increment, 1.0);
    nh.param("robust_planner/force_final_const", _force_final_const, true);
    nh.param("robust_planner/min_turn_clearance", _min_turn_clearance, 0.1);

    // params
    _planner_params.SOLVER = _solver_str;

    _planner_params.W_MAX               = _max_w;
    _planner_params.V_MAX               = _max_vel;
    _planner_params.A_MAX               = _max_acc;
    _planner_params.J_MAX               = _max_jerk;
    _planner_params.DT_FACTOR_INIT      = _factor_init;
    _planner_params.DT_FACTOR_FINAL     = _factor_final;
    _planner_params.DT_FACTOR_INCREMENT = _factor_increment;
    _planner_params.SOLVER_TRAJ_DT      = _solver_traj_dt;
    _planner_params.TRIM_DIST           = _trigger_trim_dist;

    // this is actually number of polynomials in traj, not polys...
    _planner_params.N_SEGMENTS             = _n_polys;
    _planner_params.MAX_POLYS              = _max_polys;
    _planner_params.N_THREADS              = _n_threads;
    _planner_params.FORCE_FINAL_CONSTRAINT = _force_final_const;
    _planner_params.VERBOSE                = _solver_verbose;
    _planner_params.USE_MINVO              = _use_minvo;
    _planner_params.PLAN_IN_FREE           = _plan_in_free;
    _planner_params.SIMPLIFY_JPS           = _simplify_jps;
    _planner_params.MAX_SOLVE_TIME         = _max_solve_time;

    _planner.set_params(_planner_params);

    // Publishers
    /*gridMapPub = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 0);*/
    gridMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 0);
    trajVizPub = nh.advertise<visualization_msgs::Marker>("/MINCO_path", 0);
    wptVizPub  = nh.advertise<visualization_msgs::Marker>("/MINCO_wpts", 0);
    trajPub    = nh.advertise<trajectory_msgs::JointTrajectory>("/reference_trajectory", 0);
    meshPub    = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    edgePub    = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
    helperMeshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh_helper", 1000);
    helperEdgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge_helper", 1000);
    initPointPub  = nh.advertise<geometry_msgs::PointStamped>("/initPoint", 0);
    goalPub       = nh.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 0);
    paddedLaserPub = nh.advertise<visualization_msgs::Marker>("/paddedObs", 0);
    jpsPub         = nh.advertise<nav_msgs::Path>("/jpsPath", 0);
    jpsPubFree     = nh.advertise<nav_msgs::Path>("/jpsPathFree", 0);
    jpsPointsPub   = nh.advertise<visualization_msgs::Marker>("/jpsPoints", 0);
    corridorPub    = nh.advertise<geometry_msgs::PoseArray>("/polyCorridor", 0);
    intGoalPub     = nh.advertise<geometry_msgs::PoseArray>("/intermediate_goal", 0);
    cmdVelPub      = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    initialPVAJPub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/initial_pvaj", 0);
    ctrlPointPub   = nh.advertise<geometry_msgs::PoseArray>("/ctrl_points", 0);
    tubeVizPub     = nh.advertise<visualization_msgs::Marker>("/sample_points", 0);
    unionCorridorPub = nh.advertise<visualization_msgs::Marker>("/unionCorridor", 0);
    trajPubNoReset =
        nh.advertise<trajectory_msgs::JointTrajectory>("/reference_trajectory_no_reset", 0);

    // Services
    estop_client = nh.serviceClient<std_srvs::Empty>("/switch_mode");

    // topic is: /uav1/control_manager/velocity_reference
    // msg type is mrs_msgs/VelocityReferenceStamped
#ifdef MRS_MSGS_FOUND
    nh.param("robust_planner/use_mpc", _use_mpc, true);

    _mrs_traj_client = nh.serviceClient<mrs_msgs::TrajectoryReferenceSrv>(
        "/uav1/control_manager/trajectory_reference");
#endif

    // Subscribers
    mapSub          = nh.subscribe("/map", 1, &PlannerROS::mapcb, this);
    goalSub         = nh.subscribe("/planner_goal", 1, &PlannerROS::goalcb, this);
    laserSub        = nh.subscribe("/front/scan", 1, &PlannerROS::lasercb, this);
    odomSub         = nh.subscribe("/odometry/filtered", 1, &PlannerROS::odomcb, this);
    occSub          = nh.subscribe("/occ_points", 10, &PlannerROS::occlusionPointscb, this);
    MPCHorizonSub   = nh.subscribe("/mpc_horizon", 1, &PlannerROS::mpcHorizoncb, this);
    clickedPointSub = nh.subscribe("/clicked_point", 1, &PlannerROS::clickedPointcb, this);
    pathSub =
        nh.subscribe("/global_planner/planner/plan", 1, &PlannerROS::globalPathcb, this);

    // Timers
    safetyTimer  = nh.createTimer(ros::Duration(0.5), &PlannerROS::mapPublisher, this);
    goalTimer    = nh.createTimer(ros::Duration(_dt / 2.0), &PlannerROS::goalLoop, this);
    controlTimer = nh.createTimer(ros::Duration(_dt), &PlannerROS::controlLoop, this);
    publishTimer = nh.createTimer(ros::Duration(_dt * 2), &PlannerROS::publishOccupied, this);

    _is_occ              = false;
    _is_init             = false;
    _planned             = false;
    _is_goal_set         = false;
    _map_received        = false;
    _primitive_started   = false;
    _is_costmap_started  = false;
    _is_grid_map_started = false;
    _mpc_backwards       = false;

    _mpc_backup_client = nh.serviceClient<std_srvs::Empty>("/mpc_backup");

    _prev_plan_status = SUCCESS;

    _prev_jps_path.clear();

    _prev_jps_cost = -1;
    _curr_horizon  = _max_dist_horizon;

    ROS_INFO("Initialized planner!");

    sentTraj.points.clear();
}

PlannerROS::~PlannerROS() {}

/**********************************************************************
  Function to start main ros loop for planning node. Also starts the
  costmap processes.

  TODOS: Add mutex's to class fields to allow for multi-threading
***********************************************************************/
void PlannerROS::spin()
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    if (_use_global_costmap)
        _costmap = std::make_unique<costmap_2d::Costmap2DROS>("global_costmap", tfBuffer);
    else
        _costmap = std::make_unique<costmap_2d::Costmap2DROS>("local_costmap", tfBuffer);

    _costmap->start();

    _is_costmap_started = true;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::waitForShutdown();
}

/**********************************************************************
  Function callback which reads goal position from a clicked point in
  RVIZ. When called, this function will set the _is_goal_set field to
  true, which is required for planning to begin.

  Inputs:
    - PointStamped message
***********************************************************************/
void PlannerROS::clickedPointcb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    goal           = Eigen::VectorXd(2);
    goal(0)        = msg->point.x;
    goal(1)        = msg->point.y;
    _is_goal_set   = true;
    _is_goal_reset = true;
}

/**********************************************************************
  Function to project a point into the costmap for planning purposes.

  Inputs:
    - Goal coordinates

  TODOS: Implement :)
***********************************************************************/
void PlannerROS::projectIntoMap(const Eigen::Vector2d &goal) {}

/**********************************************************************
  Function callback which reads a published goal pose. As with the
  clicked point callback function, this will set the _is_goal_set
  field to true, which is required for planning to begin.

  Inputs:
    - PoseStamped message
***********************************************************************/
void PlannerROS::goalcb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal    = Eigen::VectorXd(2);
    goal(0) = msg->pose.position.x;
    goal(1) = msg->pose.position.y;

    _is_goal_set   = true;
    _is_goal_reset = true;

    ROS_INFO("goal received!");
}

void PlannerROS::occlusionPointscb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    _occ_point = Eigen::MatrixXd(2, 3);

    if (msg->poses.size() > 0)
    {
        geometry_msgs::Pose p1 = msg->poses[0];
        geometry_msgs::Pose p2 = msg->poses[1];

        _occ_point.row(0) = Eigen::Vector3d(p1.position.x, p1.position.y, p1.position.z);
        _occ_point.row(1) = Eigen::Vector3d(p2.position.x, p2.position.y, p2.position.z);
        _is_occ           = true;
    }
}

void PlannerROS::mpcHorizoncb(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
    mpcHorizon = *msg;
    _mpc_dt    = mpcHorizon.points[1].time_from_start.toSec() -
              mpcHorizon.points[0].time_from_start.toSec();
}

/**********************************************************************
  Function which publishes the occupied cells of a global costmap.
***********************************************************************/
void PlannerROS::publishOccupied(const ros::TimerEvent &)
{
    static int prev_points = -1;

    if (!_is_costmap_started || !_is_grid_map_started) return;

    std::vector<Eigen::VectorXd> padded = _occ_grid->get_occupied(2);

    visualization_msgs::Marker paddedMsg;
    paddedMsg.header.frame_id    = _frame_str;
    paddedMsg.header.stamp       = ros::Time::now();
    paddedMsg.ns                 = "padded";
    paddedMsg.id                 = 420;
    paddedMsg.type               = visualization_msgs::Marker::CUBE_LIST;
    paddedMsg.action             = visualization_msgs::Marker::ADD;
    paddedMsg.scale.x            = _occ_grid->get_resolution();
    paddedMsg.scale.y            = _occ_grid->get_resolution();
    paddedMsg.scale.z            = .1;
    paddedMsg.pose.orientation.w = 1;
    paddedMsg.color.r            = 0.42;
    paddedMsg.color.g            = 0.153;
    paddedMsg.color.b            = 0.216;
    paddedMsg.color.a            = .55;

    paddedMsg.points.clear();
    paddedMsg.points.reserve(padded.size());
    for (Eigen::Vector2d p : padded)
    {
        geometry_msgs::Point &pMs = paddedMsg.points.emplace_back();
        pMs.x                     = p[0];
        pMs.y                     = p[1];
        pMs.z                     = 0;
    }

    paddedLaserPub.publish(paddedMsg);
}

/**********************************************************************
  Function callba__ck which reads a published map. This map is different
  from the local __and global costmap, since it doesn't contain an
  inflate layer a__round the obstacles.

  Inputs:
    - OccupancyGrid message

  TODOS: Decide if this map is needed or not for planning...
***********************************************************************/
void PlannerROS::mapcb(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map           = *msg;
    _map_received = true;
}

/**********************************************************************
  Function callback which reads in a published odometry message. To
  make arithmetic easier for planning purposes, it is converted in an
  (x,y,theta) format within an Eigen::Vector. The function also sets the
  _is_init flag to true, which is necessary for planning to begin.

  Inputs:
    - Odometry message

  TODOS: is vel field really necessary?
***********************************************************************/
void PlannerROS::odomcb(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    _odom    = Eigen::VectorXd(3);
    _odom(0) = msg->pose.pose.position.x;
    _odom(1) = msg->pose.pose.position.y;
    _odom(2) = yaw;

    if (_is_drone) _flying_height = msg->pose.pose.position.z;

    if (_is_barn && !_is_goal_set)
    {
        goal         = Eigen::VectorXd(2);
        goal(0)      = _barn_goal_dist * cos(yaw) + _odom(0);
        goal(1)      = _barn_goal_dist * sin(yaw) + _odom(1);
        _is_goal_set = true;

        ROS_INFO("yaw is %.4f", yaw);
        ROS_INFO("goal is %.4f\t%.4f", goal(0), goal(1));
    }

    _is_init = true;
}

/**********************************************************************
  Function callback which reads in a laserscan message. Only runs when
  _is_init flag is set to true in order to save on computation. In
  order to be useable by polygon generation code, the scan is saved
  into a vector of Eigen::Vector2d.

  Inputs:
    - LaserScan message

***********************************************************************/
void PlannerROS::lasercb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (!_is_init) return;

    _obs.clear();

    for (int i = 0; i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min) continue;

        double d = msg->ranges[i];

        double angle = msg->angle_min + i * msg->angle_increment + _odom(2);
        double x     = d * cos(angle) + _odom(0);
        double y     = d * sin(angle) + _odom(1);

        _obs.push_back(Eigen::Vector2f(x, y));
    }
}

/**********************************************************************
  Function callback which reads in a global path plan from an external
  node. This was used before JPS was implemented and will probably be
  removed in the near future if I don't find a need for it...

  Inputs:
    - Path message

  TODOS: Decide if this is a useful function now that JPS is
  implemented.
***********************************************************************/
void PlannerROS::globalPathcb(const nav_msgs::Path::ConstPtr &msg)
{
    for (geometry_msgs::PoseStamped p : msg->poses)
        astarPath.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
}

/**********************************************************************
  Function which visualizes a given trajectory in RVIZ as a LINE_STRIP
  visualization_msg. Currently uses an external header library called
  tinycolormap to get the color associated with a normalized velocity.

  Inputs:
    - No inputs, works on the sentTraj field.

  TODOS:
    - Break out hard coded velocity normalization constant into a
      ros param?
    - Move this to utils.h?
***********************************************************************/
void PlannerROS::visualizeTraj()
{
    if (sentTraj.points.size() > 0)
    {
        if (!_use_arclen)
        {
            visualization_msgs::Marker msg;
            msg.header.frame_id = _frame_str;
            msg.header.stamp    = ros::Time::now();
            msg.ns              = "planTraj";
            msg.id              = 80;
            msg.action          = visualization_msgs::Marker::ADD;
            msg.type            = visualization_msgs::Marker::LINE_STRIP;

            msg.scale.x            = .1;
            msg.pose.orientation.w = 1;

            for (trajectory_msgs::JointTrajectoryPoint p : sentTraj.points)
            {
                Eigen::Vector3d vel_vec(p.velocities[0], p.velocities[1], p.velocities[2]);
                double vel = vel_vec.norm();

                tinycolormap::Color color =
                    tinycolormap::GetColor(vel / 1.0, tinycolormap::ColormapType::Plasma);

                std_msgs::ColorRGBA color_msg;
                color_msg.r = color.r();
                color_msg.g = color.g();
                color_msg.b = color.b();
                color_msg.a = 1.0;
                msg.colors.push_back(color_msg);

                geometry_msgs::Point point_msg;
                point_msg.x = p.positions[0];
                point_msg.y = p.positions[1];
                point_msg.z = p.positions[2];
                msg.points.push_back(point_msg);
            }

            trajVizPub.publish(msg);
        }
        else
        {
            ros::Time start                 = ros::Time::now();
            std::vector<rfn_state_t> states = _planner.get_arclen_traj();

            if (states.size() == 0)
            {
                ROS_WARN("arc length reparameterization failed!");
                return;
            }

            std::vector<double> ss, xs, ys;
            ss.resize(states.size());
            xs.resize(states.size());
            ys.resize(states.size());

            visualization_msgs::Marker sample_msg;
            sample_msg.header.frame_id    = _frame_str;
            sample_msg.header.stamp       = ros::Time::now();
            sample_msg.ns                 = "sampemsg";
            sample_msg.id                 = 870;
            sample_msg.action             = visualization_msgs::Marker::ADD;
            sample_msg.type               = visualization_msgs::Marker::POINTS;
            sample_msg.scale.x            = .05;
            sample_msg.scale.y            = .05;
            sample_msg.pose.orientation.w = 1;
            sample_msg.color.r            = 1;
            sample_msg.color.a            = 1;

            for (int i = 0; i < states.size(); ++i)
            {
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
            arclenmsg.header.frame_id    = _frame_str;
            arclenmsg.header.stamp       = ros::Time::now();
            arclenmsg.ns                 = "planTraj";
            arclenmsg.id                 = 80;
            arclenmsg.action             = visualization_msgs::Marker::ADD;
            arclenmsg.type               = visualization_msgs::Marker::LINE_STRIP;
            arclenmsg.scale.x            = .1;
            arclenmsg.pose.orientation.w = 1;

            for (double s = 0; s < ss.back(); s += .1)
            {
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

            tubeVizPub.publish(sample_msg);
            trajVizPub.publish(arclenmsg);
        }
    }
}

/**********************************************************************
  Function for the main control loop of the planner, calls the plan()
  method which takes care of overall planning logic.

  Inputs:
    - Just a timer event which controls the rate at which the control
      loop is run.
***********************************************************************/
void PlannerROS::controlLoop(const ros::TimerEvent &)
{
    static int count = 0;

    if (!_is_init || !_is_goal_set || !_is_costmap_started) return;

    if ((_odom.head(2) - goal.head(2)).norm() < .5)
    {
        // publish empty trajecotry
        trajectory_msgs::JointTrajectory empty_traj;
        empty_traj.header.frame_id = _frame_str;
        empty_traj.header.stamp    = ros::Time::now();
        trajPub.publish(empty_traj);
    }

    /*************************************
    ************* UPDATE MAP *************
    **************************************/

    _costmap->updateMap();

    // initializer list can't be converted to const reference std::vector<T>
    std::vector<unsigned char> occ_vals = {costmap_2d::LETHAL_OBSTACLE,
                                           costmap_2d::INSCRIBED_INFLATED_OBSTACLE};

    ros::Time start = ros::Time::now();
    /*_occ_grid =*/
    /*    std::make_unique<map_util::occupancy_grid_t>(_grid_map, obs_layer, occ_vals, rad);*/
    /**/
    if (!_is_grid_map_started)
    {
        _occ_grid = std::make_unique<map_util::occupancy_grid_t>(*_costmap->getCostmap());
        _is_grid_map_started = true;
    }
    else
        _occ_grid->update(*_costmap->getCostmap());

    ROS_INFO("Occupancy grid created in %.4f seconds", (ros::Time::now() - start).toSec());

    if (sentTraj.points.size() > 1)
    {
        // get heading of trajectory without using velocity field
        double traj_theta =
            atan2(sentTraj.points[1].positions[1] - sentTraj.points[0].positions[1],
                  sentTraj.points[1].positions[0] - sentTraj.points[0].positions[0]);

        double e = atan2(sin(traj_theta - _odom(2)), cos(traj_theta - _odom(2)));

        // use mpc reverse mode if no space to turn around
        if (_occ_grid->get_signed_dist(_odom(0), _odom(1)) >= _min_turn_clearance &&
            _mpc_backwards)
        {
            std_srvs::Empty srv;
            if (_mpc_backup_client.call(srv))
            {
                _mpc_backwards = false;
                ROS_WARN("MPC reverse mode turned off!");
            }
            else
                ROS_ERROR("Failed to call MPC backup mode");
        }
    }

    /*************************************
    **************** PLAN ****************
    **************************************/

    if (_plan_once && _planned)
    {
        trajPub.publish(sentTraj);
        return;
    }

    if (!plan(count >= _failsafe_count))
    {
        count++;
        if (count >= _failsafe_count) _curr_horizon *= .9;
    }
    else
    {
        count = 0;
        _curr_horizon /= .9;
        if (_curr_horizon > _max_dist_horizon) _curr_horizon = _max_dist_horizon;
    }

    ROS_INFO("full planning time is %.4f", (ros::Time::now() - start).toSec());
}

/**********************************************************************
  This function is run on a timer controlled by the _dt ros parameter,
  and only runs if both the _is_init and is_goal_set flags are set to
  true. The overall process is:
    1.  Find JPS path from a point along previous trajectory to goal.
    1a. If first iteration, plan starting from robot position instead.
    2.  Expand intersecting polytopes around JPS path.
    3.  Send polytope and other physical constraints into optimizer.
    4.  Stitch resulting trajectory to previous one and publish to
        external trajectory tracker.

    Inputs:
      - is_failsafe flag which dictates if robot will stop for next
        planning phase. This is set true when planning failed
        several times in a row.
***********************************************************************/
bool PlannerROS::plan(bool is_failsafe)
{
    if (is_failsafe)
    {
        ROS_WARN("*******************************");
        ROS_WARN("*** FAILSAFE MODE  ENGAGED ****");
        ROS_WARN("*******************************");
    }

    ROS_INFO("MPC horizon size is %lu", mpcHorizon.points.size());

    Eigen::MatrixXd initialPVAJ(3, 4);
    Eigen::MatrixXd finalPVAJ(3, 4);

    finalPVAJ << Eigen::Vector3d(goal(0), goal(1), 0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    ros::Time a      = ros::Time::now();
    double p_start_t = 0.;

    if (sentTraj.points.size() == 0 || _is_teleop || _is_goal_reset ||
        (mpcHorizon.points.size() == 0 && _use_arclen))
    {
        initialPVAJ << Eigen::Vector3d(_odom(0), _odom(1), 0), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        ROS_INFO("initializing PVAJ where sentTraj is 0, mpchorizon is empty");
    }
    else if (mpcHorizon.points.size() == 0 &&
             !_use_arclen)  // use trajectory for init point if no horizon
    {
        ROS_INFO("trying to initialize PVAJ using sentTraj");
        double t = (a - start).toSec() + _lookahead;

        int trajInd = std::min((int)(t / _traj_dt), (int)sentTraj.points.size() - 1);

        ROS_INFO("trajInd: %d", trajInd);
        ROS_INFO("size: %lu", sentTraj.points.size());

        trajectory_msgs::JointTrajectoryPoint p = sentTraj.points[trajInd];

        p_start_t = p.time_from_start.toSec();

        ROS_INFO("making point stamped");
        geometry_msgs::PointStamped poseMsg;
        poseMsg.header.frame_id = _frame_str;
        poseMsg.header.stamp    = ros::Time::now();
        poseMsg.point.x         = p.positions[0];
        poseMsg.point.y         = p.positions[1];
        poseMsg.point.z         = p.positions[2];
        initPointPub.publish(poseMsg);
        ROS_INFO("done");

        // position
        initialPVAJ.col(0) = Eigen::Vector3d(p.positions[0], p.positions[1], p.positions[2]);

        // velocity
        initialPVAJ.col(1) =
            Eigen::Vector3d(p.velocities[0], p.velocities[1], p.velocities[2]);

        // acceleration
        initialPVAJ.col(2) =
            Eigen::Vector3d(p.accelerations[0], p.accelerations[1], p.accelerations[2]);

        ROS_INFO("done with position stuff :)");

        // jerk (stored in effort)
        initialPVAJ.col(3) = Eigen::Vector3d(p.effort[0], p.effort[1], p.effort[2]);
        ROS_INFO("initializing PVAJ where mpchorizon is empty");
    }
    else if (mpcHorizon.points.size() > 0)
    {
        // double t = (a - start).toSec() + _lookahead;

        // ROS_INFO("t is %.4f", t);
        double t = _lookahead;

        int trajInd = std::min((int)(t / _mpc_dt), (int)mpcHorizon.points.size() - 1);

        ROS_INFO("mpc horizon trajInd is %d", trajInd);

        trajectory_msgs::JointTrajectoryPoint p = mpcHorizon.points[trajInd];

        p_start_t = p.time_from_start.toSec();

        geometry_msgs::PointStamped poseMsg;
        poseMsg.header.frame_id = _frame_str;
        poseMsg.header.stamp    = ros::Time::now();
        poseMsg.point.x         = p.positions[0];
        poseMsg.point.y         = p.positions[1];
        poseMsg.point.z         = p.positions[2];
        initPointPub.publish(poseMsg);

        // position
        initialPVAJ.col(0) = Eigen::Vector3d(p.positions[0], p.positions[1], p.positions[2]);

        // velocity
        initialPVAJ.col(1) =
            Eigen::Vector3d(p.velocities[0], p.velocities[1], p.velocities[2]);

        // acceleration
        initialPVAJ.col(2) =
            Eigen::Vector3d(p.accelerations[0], p.accelerations[1], p.accelerations[2]);

        // jerk (stored in effort)
        initialPVAJ.col(3) = Eigen::Vector3d(p.effort[0], p.effort[1], p.effort[2]);
        ROS_INFO("initializing PVAJ using mpc horizon");
    }
    else
    {
        ROS_ERROR("Could not get initial position");
        return false;
    }

    // if velocity is 0, set to small value in direction of robot heading

    if (is_failsafe)
    {
        initialPVAJ.col(1) = Eigen::Vector3d::Zero();
        initialPVAJ.col(2) = Eigen::Vector3d::Zero();
        initialPVAJ.col(3) = Eigen::Vector3d::Zero();
    }

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

    if (_jps_hysteresis && !is_failsafe)
    {
        ROS_INFO("setting jps path as prev path with size %lu", _prev_jps_path.size());
        jpsPath = _prev_jps_path;
    }

    // if (is_failsafe)
    // {
    //     ROS_INFO_STREAM("initialPVAJ:\n" << initialPVAJ);
    //     ROS_INFO_STREAM("odom:\n" << _odom.tranpose());
    // }

    std::vector<Eigen::MatrixX4d> hPolys;
    ros::Time before  = ros::Time::now();
    _prev_plan_status = _planner.plan(_curr_horizon, jpsPath, hPolys);
    std::cout << "planner finished in " << (ros::Time::now() - before).toSec()
              << " with status " << _prev_plan_status << std::endl;
    if (_prev_plan_status)
    {
        ROS_WARN("Planner failed to find path");
        // check if initial velocity, acceleration, or jerk are higher than max
        double vel  = initialPVAJ.col(1).norm();
        double acc  = initialPVAJ.col(2).norm();
        double jerk = initialPVAJ.col(3).norm();
        ROS_WARN("initial conditions: %.2f\t%.2f\t%.2f!", vel, acc, jerk);
    }

    // ROS_INFO("planning took %.4f", (ros::Time::now() - start).toSec());

    visualizePolytope(hPolys, meshPub, edgePub);
    // visualizeBoundary(_planner.get_corridor_boundary(), unionCorridorPub,
    // _frame_str);

    ROS_INFO("visualized polytope %lu", hPolys.size());

    // publish initial pvaj
    ROS_INFO("publishing init pvaj: %.2f", p_start_t);
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions       = {initialPVAJ(0, 0), initialPVAJ(1, 0), initialPVAJ(2, 0)};
    p.velocities      = {initialPVAJ(0, 1), initialPVAJ(1, 1), initialPVAJ(2, 1)};
    p.accelerations   = {initialPVAJ(0, 2), initialPVAJ(1, 2), initialPVAJ(2, 2)};
    p.effort          = {initialPVAJ(0, 3), initialPVAJ(1, 3), initialPVAJ(2, 3)};
    p.time_from_start = ros::Duration(p_start_t);
    initialPVAJPub.publish(p);
    ROS_INFO("finished publish initpvaj");

    /*************************************
    ******** PUBLISH JPS TO RVIZ *********
    **************************************/

    nav_msgs::Path jpsMsg;
    jpsMsg.header.stamp    = ros::Time::now();
    jpsMsg.header.frame_id = _frame_str;

    for (Eigen::Vector2d p : jpsPath)
    {
        geometry_msgs::PoseStamped pMsg;
        pMsg.header             = jpsMsg.header;
        pMsg.pose.position.x    = p(0);
        pMsg.pose.position.y    = p(1);
        pMsg.pose.position.z    = 0;
        pMsg.pose.orientation.w = 1;
        jpsMsg.poses.push_back(pMsg);
    }

    jpsPub.publish(jpsMsg);
    ROS_INFO("published jps path: %lu", jpsPath.size());

    if (_prev_plan_status)
    {
        if (_use_arclen)
        {
            mpcHorizon.points.clear();
            std::vector<rfn_state_t> arclen_traj = _planner.get_arclen_traj();

            sentTraj.points.clear();
            for (rfn_state_t &x : arclen_traj)
            {
                // time from start is actually arc len in this case...
                trajectory_msgs::JointTrajectoryPoint p;
                p.positions.push_back(x.pos(0));
                p.positions.push_back(x.pos(1));
                p.velocities.push_back(x.vel(0));
                p.velocities.push_back(x.vel(1));
                p.time_from_start = ros::Duration(x.t);

                sentTraj.points.push_back(p);
            }
        }

        if (!_is_teleop) trajPub.publish(sentTraj);
        return false;
    }

    _prev_jps_path = jpsPath;

    /*************************************
    ******** STITCH  TRAJECTORIES ********
    **************************************/

    std::vector<rfn_state_t> planned_trajectory = _planner.get_trajectory();

    if (planned_trajectory.size() == 0)
    {
        ROS_WARN("planned trajectory is empty!");
        return false;
    }

    if (_use_arclen)
    {
        sentTraj.points.clear();
        sentTraj.header.stamp    = ros::Time::now();
        sentTraj.header.frame_id = _frame_str;

        double s_offset                      = 0;
        std::vector<rfn_state_t> arclen_traj = _planner.get_arclen_traj();

        for (rfn_state_t &x : arclen_traj)
        {
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
    }
    else if (sentTraj.points.size() != 0)
    {
        int startInd, trajInd;
        trajectory_msgs::JointTrajectory aTraj, bTraj;
        bTraj = convertTrajToMsg(planned_trajectory, _traj_dt, _frame_str);

        if (mpcHorizon.points.size() == 0)
        {
            double t1 = std::round((ros::Time::now() - start).toSec() * 10.) / 10.;
            double t2 = std::round(((a - start).toSec() + _lookahead) * 10.) / 10.;
            startInd  = std::min((int)(t1 / _traj_dt), (int)sentTraj.points.size() - 1) + 1;
            trajInd   = std::min((int)(t2 / _traj_dt), (int)sentTraj.points.size() - 1);

            for (int i = startInd; i < trajInd; i++)
            {
                aTraj.points.push_back(sentTraj.points[i]);
                aTraj.points.back().time_from_start =
                    ros::Duration((i - startInd) * _traj_dt);
            }

            double startTime = (trajInd - startInd) * _traj_dt;

            for (int i = 0; i < bTraj.points.size(); i++)
            {
                aTraj.points.push_back(bTraj.points[i]);
                aTraj.points.back().time_from_start = ros::Duration(startTime + i * _traj_dt);
            }
        }
        else
        {
            double t1 = std::round((ros::Time::now() - a).toSec() * 10.) / 10.;
            double t2 = std::round(_lookahead * 10.) / 10.;
            startInd  = std::min((int)(t1 / _mpc_dt), (int)mpcHorizon.points.size() - 1) + 1;
            trajInd   = std::min((int)(t2 / _mpc_dt), (int)mpcHorizon.points.size() - 1);

            for (int i = startInd; i < trajInd; i++)
            {
                aTraj.points.push_back(mpcHorizon.points[i]);
                aTraj.points.back().time_from_start = ros::Duration((i - startInd) * _mpc_dt);
            }

            double startTime = (trajInd - startInd) * _mpc_dt;
            // double startTime = 0;

            for (int i = 0; i < bTraj.points.size(); i++)
            {
                aTraj.points.push_back(bTraj.points[i]);
                aTraj.points.back().time_from_start = ros::Duration(startTime + i * _traj_dt);
            }
        }

        // ROS_INFO("[%.2f] startInd is %d\ttrajInd is %d",
        // (ros::Time::now()-start).toSec(),startInd, trajInd);

        aTraj.header.frame_id = _frame_str;
        aTraj.header.stamp    = ros::Time::now();
        sentTraj              = aTraj;
        start                 = ros::Time::now();
    }
    else
    {
        sentTraj = convertTrajToMsg(planned_trajectory, _traj_dt, _frame_str);
        start    = ros::Time::now();
    }

    if (sentTraj.points.size() > 2)
    {
        double traj_theta = atan2(sentTraj.points[2].positions[1] - _odom(1),
                                  sentTraj.points[2].positions[0] - _odom(0));

        double e = atan2(sin(traj_theta - _odom(2)), cos(traj_theta - _odom(2)));

        // use mpc reverse mode if no space to turn around
        if (_occ_grid->get_signed_dist(_odom(0), _odom(1)) < _min_turn_clearance &&
            fabs(e) > M_PI / 2.0 && !_mpc_backwards)
        {
            ROS_WARN("MPC reverse mode engaged!");
            std_srvs::Empty srv;
            if (_mpc_backup_client.call(srv))
            {
                _mpc_backwards = true;
                ROS_INFO("MPC backup mode called");
            }
            else
                ROS_ERROR("Failed to call MPC backup mode");
        }
    }

#ifdef MRS_MSGS_FOUND
    if (_is_drone && !_use_mpc)
    {
        mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference =
            convert_traj_to_mrs_srv(sentTraj, _frame_str, _flying_height);
        bool srv_status = _mrs_traj_client.call(srv_trajectory_reference);

        if (!srv_status)
        {
            ROS_ERROR("Failed to call mrs_traj_client");
            return false;
        }
        else if (!srv_trajectory_reference.response.success)
            ROS_ERROR("Service call for trajectory reference failed %s",
                      srv_trajectory_reference.response.message.c_str());
    }
#endif

    if (!_is_teleop) trajPub.publish(sentTraj);

    visualizeTraj();
    /*publishCPS();*/

    _planned = true;
    mpcHorizon.points.clear();

    double totalT = (ros::Time::now() - a).toSec();

    _is_goal_reset = false;
    // ROS_INFO("total time is %.4f", totalT);

    return true;
}

void PlannerROS::mapPublisher(const ros::TimerEvent &)
{
    if (!_is_costmap_started) return;

    _costmap->updateMap();
    grid_map::Costmap2DConverter<grid_map::GridMap> costmap_converter;
    const costmap_2d::Costmap2D &cmap = *_costmap->getCostmap();

    nav_msgs::OccupancyGrid grid;
    grid.header.stamp    = ros::Time::now();
    grid.header.frame_id = _costmap->getGlobalFrameID();

    // Set Info
    grid.info.resolution = cmap.getResolution();
    grid.info.width      = cmap.getSizeInCellsX();
    grid.info.height     = cmap.getSizeInCellsY();

    double origin_x, origin_y;
    cmap.mapToWorld(0, 0, origin_x, origin_y);
    grid.info.origin.position.x    = origin_x;
    grid.info.origin.position.y    = origin_y;
    grid.info.origin.orientation.w = 1.0;

    // Rescale values to 0-100 (OccupancyGrid standard)
    grid.data.resize(grid.info.width * grid.info.height);

    unsigned char *data = cmap.getCharMap();
    for (unsigned int i = 0; i < grid.data.size(); ++i)
    {
        unsigned char cost = data[i];

        if (cost == costmap_2d::NO_INFORMATION)
        {
            grid.data[i] = -1;  // Unknown
        }
        else if (cost == costmap_2d::FREE_SPACE)
        {
            grid.data[i] = 0;  // Free
        }
        else if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            grid.data[i] = 100;  // Lethal/Inscribed
        }
        else
        {
            grid.data[i] = 0;
        }
    }
    gridMapPub.publish(grid);

    /*grid_map::GridMap grid;*/
    /*costmap_converter.initializeFromCostmap2D(cmap, grid);*/
    /*costmap_converter.addLayerFromCostmap2D(cmap, "layer", grid);*/
    /**/
    /*grid_map_msgs::GridMap map_msg;*/
    /*grid_map::GridMapRosConverter::toMessage(grid, map_msg);*/
    /*gridMapPub.publish(map_msg);*/
}

/**********************************************************************
  Function to publish current goal on a timer.

  Inputs:
    - Just a timer event which controls the rate at which the function
      is run.

  TODOS:
    - Decide if this function is necesary or not.
***********************************************************************/
void PlannerROS::goalLoop(const ros::TimerEvent &)
{
    if (!_is_goal_set) return;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = _frame_str;

    msg.pose.position.x    = goal(0);
    msg.pose.position.y    = goal(1);
    msg.pose.position.z    = 0;
    msg.pose.orientation.w = 1;

    goalPub.publish(msg);
}

void PlannerROS::publishCPS()
{
    // publish control points as a posearray
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = _frame_str;
    msg.header.stamp    = ros::Time::now();

    std::vector<Eigen::Vector3d> cps = _planner.get_cps();

    for (Eigen::Vector3d cp : cps)
    {
        geometry_msgs::Pose p;
        p.position.x = cp(0);
        p.position.y = cp(1);
        p.position.z = cp(2);
        msg.poses.push_back(p);
    }

    ROS_INFO("msg poses size is %lu", msg.poses.size());
    ctrlPointPub.publish(msg);
}
