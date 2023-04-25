#ifndef PLANNER_H
#define PLANNER_H

#include <string>
#include <ros/ros.h>
#include "gcopter/gcopter.hpp"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <decomp_util/seed_decomp.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>


class Planner {
public:
    Planner(ros::NodeHandle& nh);

    void visualizeTraj();
    bool plan(bool is_failsafe = false);
    void spin();

private:

    // callbacks
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
    void globalPathcb(const nav_msgs::Path::ConstPtr& msg);
    void mapcb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void clickedPointcb(const geometry_msgs::PointStamped::ConstPtr& msg);

    // timers
    void goalLoop(const ros::TimerEvent&);
    void controlLoop(const ros::TimerEvent&);
    void publishOccupied(const ros::TimerEvent&);
    
    // utilities
    void pubPolys();
    void projectIntoMap(const Eigen::Vector2d& goal);

    template <int D>
    trajectory_msgs::JointTrajectory convertTrajToMsg(const Trajectory<D> &traj);

    Eigen::VectorXd _odom, _vel, goal;

    vec_Vec2f _obs;

    bool _is_init, _started_costmap, _is_goal_set, _is_teleop, _is_goal_reset,
         _plan_once, _simplify_jps, _is_costmap_started, _map_received, 
         _plan_in_free;

    std::string _frame_str;

    trajectory_msgs::JointTrajectory sentTraj;
    
    ros::Time start;
    ros::Timer controlTimer, goalTimer, publishTimer;
    ros::Subscriber laserSub, odomSub, pathSub, goalSub, clickedPointSub, mapSub;
    ros::Publisher trajVizPub, wptVizPub, trajPub, trajPubNoReset, meshPub, intGoalPub,
    edgePub, goalPub, paddedLaserPub, jpsPub, jpsPubFree, jpsPointsPub, currPolyPub, 
    initPointPub;

    costmap_2d::Costmap2DROS* local_costmap, *global_costmap;
    std::vector<Eigen::Vector2d> astarPath;
    std::vector<Eigen::Vector2d> _prev_jps_path;

    std::vector<Eigen::MatrixX4d> hPolys;

    Trajectory<5> traj;

    const double JACKAL_MAX_VEL = 1.0;
    double _max_vel, _dt, _const_factor, _lookahead, _traj_dt, 
    _prev_jps_cost, _max_dist_horizon;

    int _failsafe_count;

    nav_msgs::OccupancyGrid map;
    
    EllipsoidDecomp2D ellip_decomp_util_;

};

#endif
