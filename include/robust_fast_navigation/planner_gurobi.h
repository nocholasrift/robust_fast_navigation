#ifndef PLANNER_GUROBI_H
#define PLANNER_GUROBI_H

#include <mutex>
#include <atomic>
#include <thread>
#include <string>
#include <condition_variable>

#include <ros/ros.h>
#include "gcopter/gcopter.hpp"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <faster/solver.hpp>
#include <robust_fast_navigation/SolverState.h>
#include <robust_fast_navigation/motion_primitives.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <decomp_util/seed_decomp.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/geometric_utils.h>

enum RobotState
{
    NOMINAL,
    RECOVERY
};

class Planner {
public:
    Planner(ros::NodeHandle& nh);
    ~Planner();
    
    void visualizeTraj();
    bool plan(bool is_failsafe = false);
    void spin();

protected:
    robust_fast_navigation::SolverState solver_state;
    std::atomic<RobotState> _robo_state;

private:

    // callbacks
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
    void globalPathcb(const nav_msgs::Path::ConstPtr& msg);
    void mapcb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void clickedPointcb(const geometry_msgs::PointStamped::ConstPtr& msg);
    void occlusionPointscb(const geometry_msgs::PoseArray::ConstPtr& msg);

    // timers
    void goalLoop(const ros::TimerEvent&);
    void controlLoop(const ros::TimerEvent&);
    void primitivesLoop(const ros::TimerEvent&);
    void publishOccupied(const ros::TimerEvent&);
    
    void safetyLoop();

    // utilities
    void pubPolys();
    void pubCurrPoly();
    void safetyTimerThread();
    void buildPrimitiveTree();
    void projectIntoMap(const Eigen::Vector2d& goal);

    trajectory_msgs::JointTrajectory convertTrajToMsg(const std::vector<state>& trajectory);

    Eigen::VectorXd _odom, _vel, goal;

    vec_Vec2f _obs;

    bool _is_init, _started_costmap, _is_goal_set, _is_teleop, _is_goal_reset,
         _plan_once, _simplify_jps, _is_costmap_started, _map_received, _is_barn, 
         _plan_in_free, _planned, _is_occ, _primitive_started, _enable_recovery;

    std::atomic<bool> _generate_primitives;
    std::atomic<bool> _primitives_ready;

    std::string _frame_str;

    trajectory_msgs::JointTrajectory sentTraj;

    SolverGurobi solver;
    
    MotionPrimitiveTree* _prim_tree;
    std::vector<const MotionPrimitiveNode*> _recovery_traj;
    
    ros::Time start, state_transition_start_t, primitive_segment_start;
    ros::Timer controlTimer, goalTimer, publishTimer, primitiveTimer;
    ros::Subscriber laserSub, odomSub, pathSub, goalSub, clickedPointSub, mapSub, occSub;

    ros::Publisher trajVizPub;
    ros::Publisher wptVizPub;
    ros::Publisher trajPub; 
    ros::Publisher trajPubNoReset; 
    ros::Publisher meshPub; 
    ros::Publisher edgePub; 
    ros::Publisher helperMeshPub; 
    ros::Publisher helperEdgePub; 
    ros::Publisher intGoalPub;
    ros::Publisher goalPub;
    ros::Publisher paddedLaserPub;
    ros::Publisher jpsPub;
    ros::Publisher jpsPubFree;
    ros::Publisher jpsPointsPub;
    ros::Publisher currPolyPub;
    ros::Publisher initPointPub;
    ros::Publisher corridorPub;
    ros::Publisher helperPolyPub;
    ros::Publisher solverStatePub;
    ros::Publisher cmdVelPub;

    costmap_2d::Costmap2DROS* local_costmap, *global_costmap;
    std::vector<Eigen::Vector2d> astarPath;
    std::vector<Eigen::Vector2d> _prev_jps_path;

    std::vector<Eigen::MatrixX4d> hPolys;

    Eigen::MatrixXd _occ_point;
    // Trajectory<5> traj;

    const double JACKAL_MAX_VEL = 1.0;
    double _max_vel, _dt, _const_factor, _lookahead, _traj_dt, 
    _prev_jps_cost, _max_dist_horizon, _barn_goal_dist;

    int _failsafe_count;

    nav_msgs::OccupancyGrid map;
    
    EllipsoidDecomp2D ellip_decomp_util_;

    std::thread safety_thread;
    std::thread primitive_tree_thread;

    std::condition_variable _primitive_cv;
    std::condition_variable _generate_primitive_cv;

    std::mutex _recovery_mutex;
    std::mutex _robo_state_mutex;
    std::mutex _solver_state_mutex;

};

void populateSolverState(robust_fast_navigation::SolverState& state, 
                         const std::vector<Eigen::MatrixX4d>& hPolys,
                         const trajectory_msgs::JointTrajectory& traj,
                         const Eigen::MatrixXd& initialPVA,
                         const Eigen::MatrixXd& finalPVA,
                         int status);

#endif
