#include <tf2/utils.h>

#include "robust_fast_navigation/utils.h"
#include "robust_fast_navigation/planner_ros2.h"
#include "robust_fast_navigation/tinycolormap.hpp"

using namespace std::chrono_literals;

PlannerROS::PlannerROS() : Node("robust_planner_node")
{
    // --- 1. Declare and Get Parameters ---
    // Note: ROS 2 requires declaration. Using this->declare_parameter(name, default_value)
    // General Planner Params
    _solver_str         = this->declare_parameter("solver", "faster");
    _dt                 = this->declare_parameter("planner_frequency", 0.1);
    _lookahead          = this->declare_parameter("lookahead", 0.15);
    _traj_dt            = this->declare_parameter("traj_dt", 0.1);
    _max_solve_time     = this->declare_parameter("max_solve_time", 0.2);
    _solver_traj_dt     = this->declare_parameter("solver_traj_dt", 0.05);

    // Logic and Flags
    _max_dev            = this->declare_parameter("max_deviation", 1.0);
    _simplify_jps       = this->declare_parameter("simplify_jps", false);
    _jps_hysteresis     = this->declare_parameter("jps_hysteresis", false);
    _failsafe_count     = this->declare_parameter("failsafe_count", 2);
    _plan_in_free       = this->declare_parameter("plan_in_free", false);
    _max_dist_horizon   = this->declare_parameter("max_dist_horizon", 4.0);
    _frame_str          = this->declare_parameter("frame", "map");
    _is_teleop          = this->declare_parameter("teleop", false);
    _max_polys          = this->declare_parameter("max_polys", 4);
    _use_arclen         = this->declare_parameter("use_arclen", false);
    _use_global_costmap = this->declare_parameter("use_global_costmap", true);
    _plan_once          = this->declare_parameter("plan_once", false);

    // Clearance and Triggers
    _min_turn_clearance = this->declare_parameter("min_turn_clearance", 0.1);
    _trigger_trim_dist  = this->declare_parameter("trigger_trim_dist", -100.0);

    // Task Specific
    _is_barn            = this->declare_parameter("is_barn", false);
    _barn_goal_dist     = this->declare_parameter("barn_goal_dist", 10.0);
    _is_drone           = this->declare_parameter("is_drone", false);

    // Solver Optimization Params
    _n_polys            = this->declare_parameter("n_polys", 6);
    _force_final_const  = this->declare_parameter("force_final_const", true);
    _factor_init        = this->declare_parameter("factor_init", 1.0);
    _factor_final       = this->declare_parameter("factor_final", 10.0);
    _factor_increment   = this->declare_parameter("factor_increment", 1.0);
    _n_threads          = this->declare_parameter("threads", 0);
    _solver_verbose     = this->declare_parameter("verbose", 0);

    // --- 2. Initialize Planner Core Params ---
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

    // --- 3. Publishers ---
    // QoS: 10 is standard for "last will" or "reliable". Use 1 or KeepLast(1) for high-freq.
    gridMapPub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid_map", 10);
    trajVizPub = this->create_publisher<visualization_msgs::msg::Marker>("/MINCO_path", 10);
    trajPub    = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/reference_trajectory", 10);
    meshPub    = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/mesh", 10);
    edgePub    = this->create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", 10);
    initPointPub  = this->create_publisher<geometry_msgs::msg::PointStamped>("/initPoint", 10);
    goalPub       = this->create_publisher<geometry_msgs::msg::PoseStamped>("/global_planner/goal", 10);
    paddedLaserPub = this->create_publisher<visualization_msgs::msg::Marker>("/paddedObs", 10);
    jpsPub         = this->create_publisher<nav_msgs::msg::Path>("/jpsPath", 10);
    corridorPub    = this->create_publisher<geometry_msgs::msg::PoseArray>("/polyCorridor", 10);
    initialPVAJPub = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>("/initial_pvaj", 10);

    // --- 4. Services (Clients) ---
    estop_client = this->create_client<std_srvs::srv::Empty>("/switch_mode");
    _mpc_backup_client = this->create_client<std_srvs::srv::Empty>("/mpc_backup");

#ifdef MRS_MSGS_FOUND
    _use_mpc = this->declare_parameter("robust_planner.use_mpc", true);
    // Note: Service type needs to be the ROS2 equivalent if available
    _mrs_traj_client = this->create_client<mrs_msgs::srv::TrajectoryReferenceSrv>(
        "/uav1/control_manager/trajectory_reference");
#endif

    // --- 5. Subscribers ---
    mapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&PlannerROS::mapcb, this, std::placeholders::_1));
    goalSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/planner_goal", 1, std::bind(&PlannerROS::goalcb, this, std::placeholders::_1));
    laserSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/front/scan", 1, std::bind(&PlannerROS::lasercb, this, std::placeholders::_1));
    odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 1, std::bind(&PlannerROS::odomcb, this, std::placeholders::_1));
    MPCHorizonSub = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/mpc_horizon", 1, std::bind(&PlannerROS::mpcHorizoncb, this, std::placeholders::_1));
    clickedPointSub = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 1, std::bind(&PlannerROS::clickedPointcb, this, std::placeholders::_1));

    // --- 6. Timers ---
    // In ROS2, timers use chrono durations
    safetyTimer = this->create_wall_timer(500ms, std::bind(&PlannerROS::mapPublisher, this));
    goalTimer = this->create_wall_timer(std::chrono::duration<double>(_dt / 2.0), std::bind(&PlannerROS::goalLoop, this));
    controlTimer = this->create_wall_timer(std::chrono::duration<double>(_dt), std::bind(&PlannerROS::controlLoop, this));
    publishTimer = this->create_wall_timer(std::chrono::duration<double>(_dt * 2.0), std::bind(&PlannerROS::publishOccupied, this));

    // --- 7. State Initialization ---
    _is_occ = false;
    _is_init = false;
    _planned = false;
    _is_goal_set = false;
    _map_received = false;
    _primitive_started = false;
    _is_costmap_started = false;
    _is_grid_map_started = false;
    _mpc_backwards = false;

    _prev_plan_status = PlannerStatus::SUCCESS;
    _prev_jps_path.clear();
    _prev_jps_cost = -1;
    _curr_horizon = _max_dist_horizon;

    RCLCPP_INFO(this->get_logger(), "Initialized planner!");
    sentTraj.points.clear();
}

PlannerROS::~PlannerROS() {
    if (_costmap){
        _costmap->stop();
    }
}

void PlannerROS::spin()
{
    std::string costmap_name = _use_global_costmap ? "global_costmap" : "local_costmap";

    _costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        costmap_name,                        // name / local namespace
        std::string{this->get_namespace()},  // parent namespace
        costmap_name
    );

    // Costmap2DROS is itself a lifecycle node — it needs its own executor thread
    _costmap_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _costmap_executor->add_node(_costmap->get_node_base_interface());
    _costmap_thread = std::make_unique<std::thread>([this]() {
        _costmap_executor->spin();
    });

    _costmap->on_configure(rclcpp_lifecycle::State());
    _costmap->on_activate(rclcpp_lifecycle::State());
    _is_costmap_started = true;
    _costmap->start();

    RCLCPP_INFO(this->get_logger(), "Planner spinning with %s", costmap_name.c_str());
    rclcpp::spin(this->shared_from_this());

    // Cleanup on shutdown
    _costmap_executor->cancel();
    _costmap_thread->join();
}

void PlannerROS::clickedPointcb(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Initialize goal vector (Assuming goal is Eigen::VectorXd)
    _goal_vec = Eigen::VectorXd(2);
    _goal_vec(0) = msg->point.x;
    _goal_vec(1) = msg->point.y;
    
    _is_goal_set   = true;
    _is_goal_reset = true;

    RCLCPP_INFO(this->get_logger(), "New goal received from clicked point: [%.2f, %.2f]", 
                _goal_vec(0), _goal_vec(1));
}

void PlannerROS::mpcHorizoncb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
    // Copy the message to the member variable
    mpcHorizon = *msg;

    // Ensure there are at least two points to calculate dt
    if (mpcHorizon.points.size() >= 2) {
        // In ROS 2, rclcpp::Duration provides the seconds() method
        // which returns a double.
        double t1 = rclcpp::Duration(mpcHorizon.points[1].time_from_start).seconds();
        double t0 = rclcpp::Duration(mpcHorizon.points[0].time_from_start).seconds();
        
        _mpc_dt = t1 - t0;
    } else {
        RCLCPP_WARN(this->get_logger(), "MPC Horizon received with insufficient points (%zu)", 
                    mpcHorizon.points.size());
    }
}

void PlannerROS::goalcb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Use the Eigen vector we defined earlier
    _goal_vec = Eigen::VectorXd(2);
    _goal_vec(0) = msg->pose.position.x;
    _goal_vec(1) = msg->pose.position.y;

    _is_goal_set   = true;
    _is_goal_reset = true;

    RCLCPP_WARN(this->get_logger(), "goal received: x=%.2f, y=%.2f", _goal_vec(0), _goal_vec(1));
}

void PlannerROS::publishOccupied()
{
    // static int prev_points = -1; // Keep if you use it for logic later

    if (!_is_costmap_started || !_is_grid_map_started) {
        return;
    }

    // Assuming _occ_grid is your map utility and returns Eigen vectors
    std::vector<Eigen::VectorXd> padded = _occ_grid->get_occupied(2);

    visualization_msgs::msg::Marker paddedMsg;
    paddedMsg.header.frame_id    = _frame_str;
    paddedMsg.header.stamp       = this->now(); // ROS 2 version of ros::Time::now()
    paddedMsg.ns                 = "padded";
    paddedMsg.id                 = 420;
    paddedMsg.type               = visualization_msgs::msg::Marker::CUBE_LIST;
    paddedMsg.action             = visualization_msgs::msg::Marker::ADD;
    
    // Set scale using the grid resolution
    double res = _occ_grid->get_resolution();
    paddedMsg.scale.x            = res;
    paddedMsg.scale.y            = res;
    paddedMsg.scale.z            = 0.1;
    
    paddedMsg.pose.orientation.w = 1.0;
    
    // Colors
    paddedMsg.color.r            = 0.42f;
    paddedMsg.color.g            = 0.153f;
    paddedMsg.color.b            = 0.216f;
    paddedMsg.color.a            = 0.55f;

    paddedMsg.points.clear();
    paddedMsg.points.reserve(padded.size());
    
    for (const auto& p : padded)
    {
        // ROS 2 messages use standard vectors; emplace_back returns a reference in C++17+
        geometry_msgs::msg::Point pMs;
        pMs.x = p[0];
        pMs.y = p[1];
        pMs.z = 0.0;
        paddedMsg.points.push_back(pMs);
    }

    paddedLaserPub->publish(paddedMsg);
}

void PlannerROS::mapcb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_msg = *msg;
    _map_received = true;
}

void PlannerROS::odomcb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // ROS 2 replacement for the tf::Quaternion/Matrix3x3 RPY extraction
    // tf2::getYaw is the most direct way to get heading from a orientation msg
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    _odom = Eigen::VectorXd(3);
    _odom(0) = msg->pose.pose.position.x;
    _odom(1) = msg->pose.pose.position.y;
    _odom(2) = yaw;

    if (_is_drone) {
        _flying_height = msg->pose.pose.position.z;
    }

    if (_is_barn && !_is_goal_set)
    {
        _goal_vec = Eigen::VectorXd(2);
        _goal_vec(0) = _barn_goal_dist * cos(yaw) + _odom(0);
        _goal_vec(1) = _barn_goal_dist * sin(yaw) + _odom(1);
        _is_goal_set = true;

        RCLCPP_INFO(this->get_logger(), "yaw is %.4f", yaw);
        RCLCPP_INFO(this->get_logger(), "goal is %.4f\t%.4f", _goal_vec(0), _goal_vec(1));
    }

    _is_init = true;
}

void PlannerROS::lasercb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Ensure we have an initial position estimate before processing scans
    if (!_is_init) {
        return;
    }

    _obs.clear();
    // Pre-allocate memory to avoid reallocations during the loop
    _obs.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        float d = msg->ranges[i];

        // Filter out invalid readings based on sensor limits
        if (d > msg->range_max || d < msg->range_min || std::isnan(d)) {
            continue;
        }

        // Project polar coordinates (distance, angle) to Cartesian (x, y) 
        // while accounting for the robot's current heading (_odom(2))
        double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment + _odom(2);
        double x     = static_cast<double>(d) * cos(angle) + _odom(0);
        double y     = static_cast<double>(d) * sin(angle) + _odom(1);

        _obs.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
}

void PlannerROS::visualizeTraj()
{
    RCLCPP_INFO(this->get_logger(), "in visualize traj: %lu", sentTraj.points.size());
    if (!sentTraj.points.empty())
    {
        if (!_use_arclen)
        {
            visualization_msgs::msg::Marker msg;
            msg.header.frame_id = _frame_str;
            msg.header.stamp    = this->now();
            msg.ns              = "planTraj";
            msg.id              = 80;
            msg.action          = visualization_msgs::msg::Marker::ADD;
            msg.type            = visualization_msgs::msg::Marker::LINE_STRIP;

            msg.scale.x            = 0.1;
            msg.pose.orientation.w = 1.0;

            for (const auto & p : sentTraj.points)
            {
                Eigen::Vector3d vel_vec(p.velocities[0], p.velocities[1], p.velocities[2]);
                double vel = vel_vec.norm();

                // Assuming tinycolormap is available
                tinycolormap::Color color =
                    tinycolormap::GetColor(vel / 1.0, tinycolormap::ColormapType::Plasma);

                std_msgs::msg::ColorRGBA color_msg;
                color_msg.r = static_cast<float>(color.r());
                color_msg.g = static_cast<float>(color.g());
                color_msg.b = static_cast<float>(color.b());
                color_msg.a = 1.0f;
                msg.colors.push_back(color_msg);

                geometry_msgs::msg::Point point_msg;
                point_msg.x = p.positions[0];
                point_msg.y = p.positions[1];
                point_msg.z = p.positions[2];
                msg.points.push_back(point_msg);
            }

            trajVizPub->publish(msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "In use_arclen case!");
            // Arc-length reparameterization branch
            std::vector<rfn_state_t> states = _planner.get_arclen_traj();

            if (states.empty())
            {
                RCLCPP_WARN(this->get_logger(), "arc length reparameterization failed!");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "states has size %lu!", states.size());

            std::vector<double> ss, xs, ys;
            ss.resize(states.size());
            xs.resize(states.size());
            ys.resize(states.size());

            for(int i = 0; i < states.size(); ++i){
                xs[i] = states[i].pos(0);
                ys[i] = states[i].pos(1);
                ss[i] = states[i].t;
            }

            // tk::spline is likely an external dependency, logic remains same
            tk::spline splineX(ss, xs, tk::spline::cspline);
            tk::spline splineY(ss, ys, tk::spline::cspline);

            visualization_msgs::msg::Marker arclenmsg;
            arclenmsg.header.frame_id    = _frame_str;
            arclenmsg.header.stamp       = this->now();
            arclenmsg.ns                 = "planTraj";
            arclenmsg.id                 = 80;
            arclenmsg.action             = visualization_msgs::msg::Marker::ADD;
            arclenmsg.type               = visualization_msgs::msg::Marker::LINE_STRIP;
            arclenmsg.scale.x            = 0.1;
            arclenmsg.pose.orientation.w = 1.0;

            for (double s = 0; s < ss.back(); s += 0.1)
            {
                double px = splineX(s);
                double py = splineY(s);

                std_msgs::msg::ColorRGBA color_msg;
                color_msg.r = 0.0f;
                color_msg.g = 1.0f;
                color_msg.b = 0.0f;
                color_msg.a = 1.0f;
                arclenmsg.colors.push_back(color_msg);

                geometry_msgs::msg::Point point_msg;
                point_msg.x = px;
                point_msg.y = py;
                point_msg.z = 0.0;
                arclenmsg.points.push_back(point_msg);
            }

            RCLCPP_WARN(this->get_logger(), "arclenmsg points sz: %lu", arclenmsg.points.size());

            trajVizPub->publish(arclenmsg);
        }
    }
}

void PlannerROS::controlLoop()
{
    static int count = 0;

    // 1. Initial Checks
    if (!_is_init || !_is_goal_set || !_is_costmap_started) return;

    // Check goal proximity (using _goal_vec established in clickedPoint/odom callbacks)
    if ((_odom.head(2) - _goal_vec.head(2)).norm() < 0.5)
    {
        trajectory_msgs::msg::JointTrajectory empty_traj;
        empty_traj.header.frame_id = _frame_str;
        empty_traj.header.stamp    = this->now();
        trajPub->publish(empty_traj);
    }

    /*************************************
    ************* UPDATE MAP *************
    **************************************/
   _costmap->updateMap();
    
    // In ROS 2, costmap updates happen in the background. 
    // We get a pointer to the underlying Costmap2D object.
    auto start_time = this->now();

    // Lock the costmap to prevent it from updating while we are reading it
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*( _costmap->getCostmap()->getMutex() ));

    nav2_costmap_2d::Costmap2D* cmap = _costmap->getCostmap();
    int width                        = cmap->getSizeInCellsX();
    int height                       = cmap->getSizeInCellsY();
    double resolution                = cmap->getResolution();
    double origin_x                  = cmap->getOriginX();
    double origin_y                  = cmap->getOriginY();
    unsigned char *data              = cmap->getCharMap();

    // Mapping ROS 1 constants to ROS 2 (they are largely the same values)
    std::vector<unsigned char> occupied_values = {nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE,
                                                  nav2_costmap_2d::LETHAL_OBSTACLE};
    std::vector<unsigned char> unknown_values  = {nav2_costmap_2d::NO_INFORMATION};

    if (!_is_grid_map_started)
    {
        _occ_grid = std::make_unique<map_util::occupancy_grid_t>(
            width, height, resolution, origin_x, origin_y, data, occupied_values,
            unknown_values);
        _is_grid_map_started = true;
    }
    else
    {
        _occ_grid->update(width, height, resolution, origin_x, origin_y, data,
                          occupied_values, unknown_values);
    }
    
    // Release the lock as soon as the grid update is done
    lock.unlock();

    double grid_time = (this->now() - start_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Occupancy grid created in %.4f seconds", grid_time);

    /*************************************
    **************** PLAN ****************
    **************************************/

    if (_plan_once && _planned)
    {
        trajPub->publish(sentTraj);
        return;
    }

    // Logic for failsafe and horizon scaling
    if (!plan(count >= _failsafe_count))
    {
        count++;
        if (count >= _failsafe_count) _curr_horizon *= 0.9;
    }
    else
    {
        count = 0;
        _curr_horizon /= 0.9;
        if (_curr_horizon > _max_dist_horizon) _curr_horizon = _max_dist_horizon;
    }

    double total_time = (this->now() - start_time).seconds();
    RCLCPP_INFO(this->get_logger(), "full planning time is %.4f", total_time);
}

bool PlannerROS::plan(bool is_failsafe)
{
    if (is_failsafe)
    {
        RCLCPP_WARN(this->get_logger(), "*******************************");
        RCLCPP_WARN(this->get_logger(), "*** FAILSAFE MODE  ENGAGED ****");
        RCLCPP_WARN(this->get_logger(), "*******************************");
    }

    RCLCPP_INFO(this->get_logger(), "MPC horizon size is %zu", mpcHorizon.points.size());

    Eigen::MatrixXd initialPVAJ(3, 4);
    Eigen::MatrixXd finalPVAJ(3, 4);

    // goal(0) etc. refers to the Eigen::VectorXd established in odomcb/clickedPointcb
    finalPVAJ << Eigen::Vector3d(_goal_vec(0), _goal_vec(1), 0), Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    rclcpp::Time a = this->now();
    double p_start_t = 0.0;

    // Initialization Logic
    if (sentTraj.points.empty() || _is_teleop || _is_goal_reset ||
        (mpcHorizon.points.empty() && _use_arclen))
    {
        initialPVAJ << Eigen::Vector3d(_odom(0), _odom(1), 0), Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        RCLCPP_INFO(this->get_logger(), "initializing PVAJ where sentTraj is 0, mpchorizon is empty");
    }
    else if (mpcHorizon.points.empty() && !_use_arclen)
    {
        RCLCPP_INFO(this->get_logger(), "trying to initialize PVAJ using sentTraj");
        double t = (a - start).seconds() + _lookahead;

        int trajInd = std::min((int)(t / _traj_dt), (int)sentTraj.points.size() - 1);

        trajectory_msgs::msg::JointTrajectoryPoint p = sentTraj.points[trajInd];
        p_start_t = rclcpp::Duration(p.time_from_start).seconds();

        geometry_msgs::msg::PointStamped poseMsg;
        poseMsg.header.frame_id = _frame_str;
        poseMsg.header.stamp    = this->now();
        poseMsg.point.x         = p.positions[0];
        poseMsg.point.y         = p.positions[1];
        poseMsg.point.z         = p.positions[2];
        initPointPub->publish(poseMsg);

        initialPVAJ.col(0) = Eigen::Vector3d(p.positions[0], p.positions[1], p.positions[2]);
        initialPVAJ.col(1) = Eigen::Vector3d(p.velocities[0], p.velocities[1], p.velocities[2]);
        initialPVAJ.col(2) = Eigen::Vector3d(p.accelerations[0], p.accelerations[1], p.accelerations[2]);
        initialPVAJ.col(3) = Eigen::Vector3d(p.effort[0], p.effort[1], p.effort[2]);
    }
    else if (!mpcHorizon.points.empty())
    {
        double t = _lookahead;
        int trajInd = std::min((int)(t / _mpc_dt), (int)mpcHorizon.points.size() - 1);

        trajectory_msgs::msg::JointTrajectoryPoint p = mpcHorizon.points[trajInd];
        p_start_t = rclcpp::Duration(p.time_from_start).seconds();

        geometry_msgs::msg::PointStamped poseMsg;
        poseMsg.header.frame_id = _frame_str;
        poseMsg.header.stamp    = this->now();
        poseMsg.point.x         = p.positions[0];
        poseMsg.point.y         = p.positions[1];
        poseMsg.point.z         = p.positions[2];
        initPointPub->publish(poseMsg);

        initialPVAJ.col(0) = Eigen::Vector3d(p.positions[0], p.positions[1], p.positions[2]);
        initialPVAJ.col(1) = Eigen::Vector3d(p.velocities[0], p.velocities[1], p.velocities[2]);
        initialPVAJ.col(2) = Eigen::Vector3d(p.accelerations[0], p.accelerations[1], p.accelerations[2]);
        initialPVAJ.col(3) = Eigen::Vector3d(p.effort[0], p.effort[1], p.effort[2]);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Could not get initial position");
        return false;
    }

    if (is_failsafe)
    {
        initialPVAJ.col(1).setZero();
        initialPVAJ.col(2).setZero();
        initialPVAJ.col(3).setZero();
    }

    /*************************************
    **************** PLAN ****************
    **************************************/

    _planner.set_costmap(*_occ_grid);
    _planner.set_start(initialPVAJ);
    _planner.set_goal(finalPVAJ);

    std::vector<Eigen::Vector2d> jpsPath;
    if (_jps_hysteresis && !is_failsafe) jpsPath = _prev_jps_path;

    std::vector<Eigen::MatrixX4d> hPolys;
    auto before = this->now();
    _prev_plan_status = _planner.plan(_curr_horizon, jpsPath, hPolys);

    RCLCPP_INFO(this->get_logger(), "planner finished in %.4f with status %d", 
                (this->now() - before).seconds(), static_cast<int>(_prev_plan_status));

    if (_prev_plan_status != PlannerStatus::SUCCESS)
    {
        RCLCPP_WARN(this->get_logger(), "Planner failed to find path. Conditions: V:%.2f A:%.2f J:%.2f", 
                    initialPVAJ.col(1).norm(), initialPVAJ.col(2).norm(), initialPVAJ.col(3).norm());
    }

    ros2_utils::visualizePolytope(hPolys, meshPub, edgePub, this->now(), _frame_str);

    // Publish current start point for debugging
    trajectory_msgs::msg::JointTrajectoryPoint p_start;
    p_start.positions     = {initialPVAJ(0, 0), initialPVAJ(1, 0), initialPVAJ(2, 0)};
    p_start.velocities    = {initialPVAJ(0, 1), initialPVAJ(1, 1), initialPVAJ(2, 1)};
    p_start.accelerations = {initialPVAJ(0, 2), initialPVAJ(1, 2), initialPVAJ(2, 2)};
    p_start.effort        = {initialPVAJ(0, 3), initialPVAJ(1, 3), initialPVAJ(2, 3)};
    p_start.time_from_start = rclcpp::Duration::from_seconds(p_start_t);
    initialPVAJPub->publish(p_start);

    /*************************************
    ******** PUBLISH JPS TO RVIZ *********
    **************************************/
    nav_msgs::msg::Path jpsMsg;
    jpsMsg.header.stamp    = this->now();
    jpsMsg.header.frame_id = _frame_str;

    for (const auto& pt : jpsPath)
    {
        geometry_msgs::msg::PoseStamped pMsg;
        pMsg.header             = jpsMsg.header;
        pMsg.pose.position.x    = pt(0);
        pMsg.pose.position.y    = pt(1);
        pMsg.pose.orientation.w = 1.0;
        jpsMsg.poses.push_back(pMsg);
    }
    jpsPub->publish(jpsMsg);

    // Fail case handling
    if (_prev_plan_status != PlannerStatus::SUCCESS)
    {
        if (_use_arclen)
        {
            mpcHorizon.points.clear();
            std::vector<rfn_state_t> arclen_traj = _planner.get_arclen_traj();
            sentTraj.points.clear();
            for (auto &x : arclen_traj)
            {
                trajectory_msgs::msg::JointTrajectoryPoint pt;
                pt.positions = {x.pos(0), x.pos(1)};
                pt.velocities = {x.vel(0), x.vel(1)};
                pt.time_from_start = rclcpp::Duration::from_seconds(x.t);
                sentTraj.points.push_back(pt);
            }
        }
        if (!_is_teleop) trajPub->publish(sentTraj);
        return false;
    }

    _prev_jps_path = jpsPath;

    /*************************************
    ******** STITCH  TRAJECTORIES ********
    **************************************/
    std::vector<rfn_state_t> planned_trajectory = _planner.get_trajectory();
    if (planned_trajectory.empty()) return false;

    if (_use_arclen)
    {
        sentTraj.points.clear();
        sentTraj.header.stamp    = this->now();
        sentTraj.header.frame_id = _frame_str;
        std::vector<rfn_state_t> arclen_traj = _planner.get_arclen_traj();

        for (auto &x : arclen_traj)
        {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = {x.pos(0), x.pos(1)};
            pt.velocities = {x.vel(0), x.vel(1)};
            pt.time_from_start = rclcpp::Duration::from_seconds(x.t);
            sentTraj.points.push_back(pt);
        }
    }
    else if (!sentTraj.points.empty())
    {
        // Stitching logic remains similar, but use rclcpp::Duration and seconds()
        trajectory_msgs::msg::JointTrajectory aTraj;
        trajectory_msgs::msg::JointTrajectory bTraj = 
            convertTrajToMsg(
                planned_trajectory,
                _traj_dt, 
                _frame_str,
                this->now()
            );

        if (mpcHorizon.points.empty())
        {
            double t1 = std::round((this->now() - start).seconds() * 10.0) / 10.0;
            double t2 = std::round(((a - start).seconds() + _lookahead) * 10.0) / 10.0;
            int startInd = std::min((int)(t1 / _traj_dt), (int)sentTraj.points.size() - 1) + 1;
            int trajInd  = std::min((int)(t2 / _traj_dt), (int)sentTraj.points.size() - 1);

            for (int i = startInd; i < trajInd; i++)
            {
                aTraj.points.push_back(sentTraj.points[i]);
                aTraj.points.back().time_from_start = rclcpp::Duration::from_seconds((i - startInd) * _traj_dt);
            }
            double startTime = (trajInd - startInd) * _traj_dt;
            for (size_t i = 0; i < bTraj.points.size(); i++)
            {
                aTraj.points.push_back(bTraj.points[i]);
                aTraj.points.back().time_from_start = rclcpp::Duration::from_seconds(startTime + i * _traj_dt);
            }
        }
        else
        {
            double t1 = std::round((this->now() - a).seconds() * 10.0) / 10.0;
            double t2 = std::round(_lookahead * 10.0) / 10.0;
            int startInd = std::min((int)(t1 / _mpc_dt), (int)mpcHorizon.points.size() - 1) + 1;
            int trajInd  = std::min((int)(t2 / _mpc_dt), (int)mpcHorizon.points.size() - 1);

            for (int i = startInd; i < trajInd; i++)
            {
                aTraj.points.push_back(mpcHorizon.points[i]);
                aTraj.points.back().time_from_start = rclcpp::Duration::from_seconds((i - startInd) * _mpc_dt);
            }
            double startTime = (trajInd - startInd) * _mpc_dt;
            for (size_t i = 0; i < bTraj.points.size(); i++)
            {
                aTraj.points.push_back(bTraj.points[i]);
                aTraj.points.back().time_from_start = rclcpp::Duration::from_seconds(startTime + i * _traj_dt);
            }
        }
        aTraj.header.frame_id = _frame_str;
        aTraj.header.stamp    = this->now();
        sentTraj = aTraj;
        start = this->now();
    }
    else
    {
        sentTraj = convertTrajToMsg(planned_trajectory, _traj_dt, _frame_str, this->now());
        start = this->now();
    }

    if (!_is_teleop) trajPub->publish(sentTraj);

    visualizeTraj();
    _planned = true;
    _is_goal_reset = false;
    mpcHorizon.points.clear();

    return true;
}

void PlannerROS::mapPublisher()
{
    if (!_is_costmap_started) return;

    // In ROS 2, costmaps are updated by a background thread.
    // Access the underlying costmap and lock it for thread safety.
    auto cmap = _costmap->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(cmap->getMutex()));

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp    = this->now();
    grid.header.frame_id = _costmap->getGlobalFrameID();

    // Set Map Info
    grid.info.resolution = cmap->getResolution();
    grid.info.width      = cmap->getSizeInCellsX();
    grid.info.height     = cmap->getSizeInCellsY();

    double origin_x, origin_y;
    // Map coordinate (0,0) is the origin in world coordinates
    cmap->mapToWorld(0, 0, origin_x, origin_y);
    grid.info.origin.position.x    = origin_x;
    grid.info.origin.position.y    = origin_y;
    grid.info.origin.position.z    = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Prepare data vector
    grid.data.resize(grid.info.width * grid.info.height);

    const unsigned char *data = cmap->getCharMap();
    for (unsigned int i = 0; i < grid.data.size(); ++i)
    {
        unsigned char cost = data[i];

        if (cost == nav2_costmap_2d::NO_INFORMATION)
        {
            grid.data[i] = -1;  // Unknown in OccupancyGrid
        }
        else if (cost == nav2_costmap_2d::FREE_SPACE)
        {
            grid.data[i] = 0;   // Free
        }
        else if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            grid.data[i] = 100; // Occupied (Lethal or Inscribed)
        }
        else
        {
            // Optional: You could scale intermediate costs (1-99) here if desired.
            // Your original logic treats everything below Inscribed as free.
            grid.data[i] = 0;
        }
    }
    
    // Explicitly unlock before publishing to keep the lock duration short
    lock.unlock();

    gridMapPub->publish(grid);
}

void PlannerROS::goalLoop()
{
    // 1. Guard check
    if (!_is_goal_set) {
        return;
    }

    // 2. Construct the message
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp    = this->now(); // ROS 2 Node clock
    msg.header.frame_id = _frame_str;

    // Use _goal_vec (the Eigen::VectorXd we updated in odom/clicked callbacks)
    msg.pose.position.x    = _goal_vec(0);
    msg.pose.position.y    = _goal_vec(1);
    msg.pose.position.z    = 0.0;
    
    // Standard identity orientation (no rotation)
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    // 3. Publish
    goalPub->publish(msg);
}

trajectory_msgs::msg::JointTrajectory PlannerROS::convertTrajToMsg(
    const std::vector<rfn_state_t> &trajectory, 
    double traj_dt, 
    std::string_view frame_str,
    const rclcpp::Time & current_time) // We need to pass the time in ROS 2
{
    trajectory_msgs::msg::JointTrajectory msg;
    msg.header.stamp    = current_time;
    msg.header.frame_id = std::string(frame_str); // Convert string_view to std::string

    double next_t = 0.0;
    
    // Pre-reserve memory to avoid reallocations
    msg.points.reserve(trajectory.size());

    for (const auto & x : trajectory)
    {
        if (x.t >= next_t)
        {
            trajectory_msgs::msg::JointTrajectoryPoint p;
            
            // Using initializer lists is cleaner in ROS 2 / C++17
            p.positions     = {x.pos(0), x.pos(1), x.pos(2)};
            p.velocities    = {x.vel(0), x.vel(1), x.vel(2)};
            p.accelerations = {x.accel(0), x.accel(1), x.accel(2)};
            
            // Jerk stored in effort as per your original logic
            p.effort        = {x.jerk(0), x.jerk(1), x.jerk(2)};

            // Convert double seconds to rclcpp::Duration
            p.time_from_start = rclcpp::Duration::from_seconds(x.t);

            msg.points.push_back(p);

            next_t += traj_dt;
        }
    }

    return msg;
}