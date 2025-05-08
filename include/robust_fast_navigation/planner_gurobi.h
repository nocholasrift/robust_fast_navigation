#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
/*#include <robust_fast_navigation/grid_map_util.h>*/
#include <robust_fast_navigation/map_util.h>
#include <robust_fast_navigation/planner_core.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <faster/solver.hpp>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string>

class PlannerROS
{
   public:
    PlannerROS(ros::NodeHandle &nh);
    ~PlannerROS();

    void spin();
    void visualizeTraj();
    bool plan(bool is_failsafe = false);

   private:
    // callbacks
    void odomcb(const nav_msgs::Odometry::ConstPtr &msg);
    void globalPathcb(const nav_msgs::Path::ConstPtr &msg);
    void mapcb(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void lasercb(const sensor_msgs::LaserScan::ConstPtr &msg);
    void goalcb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void clickedPointcb(const geometry_msgs::PointStamped::ConstPtr &msg);
    void occlusionPointscb(const geometry_msgs::PoseArray::ConstPtr &msg);
    void mpcHorizoncb(const trajectory_msgs::JointTrajectory::ConstPtr &msg);

    // timers
    void safetyLoop(const ros::TimerEvent &);
    void goalLoop(const ros::TimerEvent &);
    void controlLoop(const ros::TimerEvent &);
    void publishOccupied(const ros::TimerEvent &);

    // utilities
    void publishCPS();
    void getMPCParams(ros::NodeHandle &nh);
    void projectIntoMap(const Eigen::Vector2d &goal);

    Eigen::VectorXd _odom;
    Eigen::VectorXd _vel;
    Eigen::VectorXd goal;

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

    std::string _frame_str;
    std::string _solver_str;

    trajectory_msgs::JointTrajectory sentTraj;
    trajectory_msgs::JointTrajectory mpcHorizon;

    Planner _planner;
    PlannerStatus _prev_plan_status;
    planner_params_t _planner_params;

    ros::Time start, state_transition_start_t;
    ros::Timer controlTimer, goalTimer, publishTimer, safetyTimer;

    // Services
    ros::ServiceClient estop_client;
    ros::ServiceClient _mpc_backup_client;

    // Subscribers
    ros::Subscriber mapSub;
    ros::Subscriber occSub;
    ros::Subscriber odomSub;
    ros::Subscriber pathSub;
    ros::Subscriber goalSub;
    ros::Subscriber laserSub;
    ros::Subscriber MPCHorizonSub;
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
    ros::Publisher initPointPub;
    ros::Publisher corridorPub;
    ros::Publisher recoveryPolyPub;
    ros::Publisher solverStatePub;
    ros::Publisher solverStateArrayPub;
    ros::Publisher unionCorridorPub;
    ros::Publisher cmdVelPub;
    ros::Publisher tubeVizPub;
    ros::Publisher expectedFailurePub;
    ros::Publisher initialPVAJPub;
    ros::Publisher currentReferencePub;
    ros::Publisher ctrlPointPub;

    std::unique_ptr<costmap_2d::Costmap2DROS> _costmap;

    grid_map::GridMap _grid_map;
    std::unique_ptr<map_util::occupancy_grid_t> _occ_grid;
    /*std::unique_ptr<BezierSdfNLP> _sdf_solver;*/

    std::vector<float> _predictions;
    std::vector<Eigen::Vector2f> _obs;
    std::vector<Eigen::Vector2d> astarPath;
    std::vector<Eigen::Vector2d> _prev_jps_path;

    Eigen::MatrixXd _occ_point;

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

    int _n_polys;
    int _n_threads;
    int _solver_verbose;
    int _failsafe_count;

    nav_msgs::OccupancyGrid map;
};
