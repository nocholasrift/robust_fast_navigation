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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <decomp_util/seed_decomp.h>
#include <decomp_util/line_segment.h>
#include <decomp_geometry/geometric_utils.h>

class Planner {
public:
    Planner(ros::NodeHandle& nh);

    template <int D>
    void visualizeTraj(const Trajectory<D> &traj);
    void spin();

private:

    void clickedPointcb(const geometry_msgs::PointStamped::ConstPtr& msg);
    void goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
    void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void globalPathcb(const nav_msgs::Path::ConstPtr& msg);
    void controlLoop(const ros::TimerEvent&);
    void goalLoop(const ros::TimerEvent&);
    void pubCurrPoly();

    template <int D>
    trajectory_msgs::JointTrajectory convertTrajToMsg(const Trajectory<D> &traj);

    Eigen::VectorXd _odom, _vel, goal;

    vec_Vec2f _obs;

    bool _is_init, _started_costmap, _is_goal_set, _is_teleop;

    std::string _frame_str;

    ros::Timer controlTimer, goalTimer;
    ros::Subscriber laserSub, odomSub, pathSub, goalSub, clickedPointSub;
    ros::Publisher trajVizPub, wptVizPub, trajPub, trajPubNoReset, meshPub, 
    edgePub, goalPub, paddedLaserPub, jpsPub, jpsPointsPub, currPolyPub, initPointPub;

    costmap_2d::Costmap2DROS* costmap;
    std::vector<Eigen::Vector2d> astarPath;

    std::vector<Eigen::MatrixX4d> hPolys;

    Trajectory<5> traj;

    const double JACKAL_MAX_VEL = 1.0;
    double _max_vel, _dt, _const_factor;
};

#endif