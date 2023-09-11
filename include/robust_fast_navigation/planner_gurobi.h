#ifndef PLANNER_GUROBI_H
#define PLANNER_GUROBI_H

#include <mutex>
#include <atomic>
#include <thread>
#include <string>
#include <condition_variable>

#include <ros/ros.h>
#include "gcopter/gcopter.hpp"

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <faster/solver.hpp>
#include <robust_fast_navigation/SolverStateArray.h>
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
    
    void spin();
    void visualizeTraj();
    bool plan(bool is_failsafe = false);
    bool solveFromSolverState(const robust_fast_navigation::SolverState& solver_state);

protected:
    robust_fast_navigation::SolverState solver_state;
    std::atomic<RobotState> _robo_state;

private:

    // callbacks
    void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
    void globalPathcb(const nav_msgs::Path::ConstPtr& msg);
    void mapcb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void mpcGoalReachedcb(const std_msgs::Bool::ConstPtr& msg);
    void goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void predictionscb(const std_msgs::Float32MultiArray::ConstPtr& msg);
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

    Eigen::VectorXd _odom, _vel, goal;

    vec_Vec2f _obs;

    bool _is_init;
    bool _started_costmap;
    bool _is_goal_set;
    bool _is_teleop;
    bool _is_goal_reset;
    bool _plan_once;
    bool _simplify_jps;
    bool _is_costmap_started;
    bool _map_received;
    bool _is_barn;
    bool _plan_in_free;
    bool _planned;
    bool _is_occ;
    bool _primitive_started;
    bool _enable_recovery;
    bool _use_minvo;

    std::atomic<bool> _primitives_ready;
    std::atomic<bool> _predictions_ready;
    std::atomic<bool> _generate_primitives;
    std::atomic<bool> _mpc_goal_reached;

    std::string _frame_str;

    trajectory_msgs::JointTrajectory sentTraj;

    SolverGurobi solver;
    
    MotionPrimitiveTree* _prim_tree;
    std::vector<const MotionPrimitiveNode*> _recovery_traj;
    
    ros::Time start, state_transition_start_t, primitive_segment_start;
    ros::Timer controlTimer, goalTimer, publishTimer, primitiveTimer;
    
    // Services
    ros::ServiceClient estop_client;

    // Subscribers
    ros::Subscriber mapSub;
    ros::Subscriber occSub;
    ros::Subscriber odomSub;
    ros::Subscriber pathSub;
    ros::Subscriber goalSub;
    ros::Subscriber laserSub;
    ros::Subscriber predictionsSub;
    ros::Subscriber clickedPointSub;
    ros::Subscriber mpcGoalReachedSub;

    // Publishers
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
    ros::Publisher recoveryPolyPub;
    ros::Publisher solverStatePub;
    ros::Publisher solverStateArrayPub;
    ros::Publisher candidatePointsVizPub;
    ros::Publisher cmdVelPub;
    ros::Publisher recoveryGoalPub;
    ros::Publisher expectedFailurePub;

    costmap_2d::Costmap2DROS* local_costmap, *global_costmap;

    std::vector<Eigen::Vector2d> astarPath;
    std::vector<Eigen::Vector2d> _prev_jps_path;
    std::vector<Eigen::MatrixX4d> hPolys;
    std::vector<float> _predictions;
    
    std::vector<std::tuple<Eigen::Vector3d, double> > _expected_failure_odoms;

    Eigen::MatrixXd _occ_point;
    // Trajectory<5> traj;

    double _max_vel;
    double _max_acc;
    double _max_jerk;
    double _max_w;
    double _dt;
    double _const_factor;
    double _lookahead;
    double _traj_dt;
    double _prev_jps_cost;
    double _max_dist_horizon;
    double _barn_goal_dist;
    double _recovery_thresh;
    double _curr_horizon;
    double _max_dev;

    int _failsafe_count;
    int _recovery_samples;
    int _recovery_horizon;

    nav_msgs::OccupancyGrid map;
    
    EllipsoidDecomp2D ellip_decomp_util_;

    std::thread safety_thread;
    std::thread primitive_tree_thread;

    std::condition_variable _primitive_cv;
    std::condition_variable _predictions_cv;
    std::condition_variable _generate_primitive_cv;
    std::condition_variable _mpc_goal_cv;

    std::mutex _recovery_mutex;
    std::mutex _robo_state_mutex;
    std::mutex _mpc_goal_mutex;
    std::mutex _predictions_mutex;

};

void populateSolverState(robust_fast_navigation::SolverState& state, 
                         const nav_msgs::Path& jpsPath,
                         const std::vector<Eigen::MatrixX4d>& hPolys,
                         const trajectory_msgs::JointTrajectory& traj,
                         const Eigen::MatrixXd& initialPVA,
                         const Eigen::MatrixXd& finalPVA,
                         int status);

#endif
