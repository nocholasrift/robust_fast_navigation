#include <math.h>
#include <cmath>
#include <vector>
#include <string>

#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>

#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/utils.h>
#include <robust_fast_navigation/spline.h>
#include <robust_fast_navigation/planner_gurobi.h>
#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/tinycolormap.hpp>

/**********************************************************************
  Constructor to read in ROS params, setup subscribers & publishers,
  ros timers, and initialize class fields.
***********************************************************************/
Planner::Planner(ros::NodeHandle& nh){

    // ROS Params
    nh.param("robust_planner/traj_dt", _traj_dt, .1);
    nh.param("robust_planner/is_barn", _is_barn, false);
    nh.param("robust_planner/teleop", _is_teleop, false);
    nh.param("robust_planner/lookahead", _lookahead, .15);
    nh.param("robust_planner/planner_frequency", _dt, .1);
    nh.param("robust_planner/max_velocity", _max_vel, 1.);
    nh.param("robust_planner/plan_once", _plan_once, false);
    nh.param("robust_planner/const_factor", _const_factor, 6.);
    nh.param("robust_planner/plan_in_free", _plan_in_free, false);
    nh.param("robust_planner/simplify_jps", _simplify_jps, false);
    nh.param("robust_planner/failsafe_count", _failsafe_count, 2);
    nh.param<std::string>("robust_planner/frame", _frame_str, "map");
    nh.param("robust_planner/barn_goal_dist", _barn_goal_dist, 10.);
    nh.param("robust_planner/max_dist_horizon", _max_dist_horizon, 4.);
    nh.param("robust_planner/enable_recovery", _enable_recovery, true);

    // Publishers 
    trajVizPub = 
        nh.advertise<visualization_msgs::Marker>("/MINCO_path", 0);
    wptVizPub = 
        nh.advertise<visualization_msgs::Marker>("/MINCO_wpts", 0);
    trajPub = 
        nh.advertise<trajectory_msgs::JointTrajectory>("/reference_trajectory", 0);
    trajPubNoReset = 
        nh.advertise<trajectory_msgs::JointTrajectory>("/reference_trajectory_no_reset", 0);
    
    meshPub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    edgePub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);

    helperMeshPub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/mesh_helper", 1000);
    helperEdgePub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/edge_helper", 1000);

    initPointPub = 
        nh.advertise<geometry_msgs::PointStamped>("/initPoint", 0);

    goalPub = 
        nh.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 0);

    paddedLaserPub =
        nh.advertise<visualization_msgs::Marker>("/paddedObs", 0);

    jpsPub = 
        nh.advertise<nav_msgs::Path>("/jpsPath", 0);
    jpsPubFree = 
        nh.advertise<nav_msgs::Path>("/jpsPathFree", 0);

    jpsPointsPub = 
        nh.advertise<visualization_msgs::Marker>("/jpsPoints", 0);

    currPolyPub = 
        nh.advertise<geometry_msgs::PoseArray>("/currPoly", 0);

    corridorPub = 
        nh.advertise<geometry_msgs::PoseArray>("/polyCorridor", 0);

    helperPolyPub = 
        nh.advertise<geometry_msgs::PoseArray>("/helperPolys", 0);

    intGoalPub = 
        nh.advertise<geometry_msgs::PoseArray>("/intermediate_goal", 0);

    solverStatePub = 
        nh.advertise<robust_fast_navigation::SolverState>("/solverState", 0);

    cmdVelPub = 
        nh.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

    // Subscribers
    mapSub = nh.subscribe("/map", 1, &Planner::mapcb, this);
    goalSub = nh.subscribe("/planner_goal", 1, &Planner::goalcb, this);
    laserSub = nh.subscribe("/front/scan", 1, &Planner::lasercb, this);
    odomSub = nh.subscribe("/odometry/filtered", 1, &Planner::odomcb, this);
    occSub = nh.subscribe("/occ_points", 10, &Planner::occlusionPointscb, this);
    clickedPointSub = nh.subscribe("/clicked_point", 1, &Planner::clickedPointcb, this);
    pathSub = nh.subscribe("/global_planner/planner/plan", 1, &Planner::globalPathcb, this);

    // Timers
    goalTimer = nh.createTimer(ros::Duration(_dt/2.0), &Planner::goalLoop, this);
    controlTimer = nh.createTimer(ros::Duration(_dt), &Planner::controlLoop, this);
    publishTimer = nh.createTimer(ros::Duration(_dt*2), &Planner::publishOccupied, this);

    if (_enable_recovery)
        primitiveTimer = nh.createTimer(ros::Duration(.01),&Planner::primitivesLoop, this);

    _is_occ = false;
    _is_init = false;
    _planned = false;
    _is_goal_set = false;
    _map_received = false;
    _primitive_started = false;
    _is_costmap_started = false;
    
    // atomics
    _primitives_ready = false;
    _generate_primitives = false;

    _robo_state = NOMINAL;

    _prev_jps_cost = -1;

    _prim_tree = nullptr;
    global_costmap = nullptr;

    ROS_INFO("Initialized planner!");

    double limits[3] = {_max_vel,1.2,4};

    solver.setN(6);
    solver.createVars();
    solver.setDC(.05);
    solver.setBounds(limits);
    solver.setForceFinalConstraint(true);
    solver.setFactorInitialAndFinalAndIncrement(1,10,1.0);
    solver.setThreads(0);
    solver.setWMax(4.0);
    solver.setVerbose(0);
}

Planner::~Planner(){

    if (global_costmap)
        delete global_costmap;

    if (_prim_tree)
        delete _prim_tree;

    if (safety_thread.joinable()){
        std::cout << "rejoined safety thread" << std::endl;
        safety_thread.join();
    }

    if (primitive_tree_thread.joinable()){
        std::cout << "rejoined primitive tree thread" << std::endl;
        primitive_tree_thread.join();
    }
}

/**********************************************************************
  Function to start main ros loop for planning node. Also starts the 
  costmap processes.

  TODOS: Add mutex's to class fields to allow for multi-threading
***********************************************************************/
void Planner::spin(){
    //tf::TransformListener tfListener(ros::Duration(10));

    // ROS_INFO("starting local costmap");
    // local_costmap = new costmap_2d::Costmap2DROS("local_costmap", tfBuffer);
    // local_costmap->start();
    
    // allow time for transforms and things to be published
    // ros::Duration(3).sleep();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ROS_INFO("initializing global costmap");
    global_costmap = new costmap_2d::Costmap2DROS("global_costmap", tfBuffer);
    ROS_INFO("starting global costmap");
    global_costmap->start();
    ROS_INFO("done!");

    _is_costmap_started = true;

    _prim_tree = new MotionPrimitiveTree(global_costmap, 1.5, .1);

    // Start the safety timer in a separate thread
    if (_enable_recovery){
        ROS_INFO("starting safety thread");
        safety_thread = std::thread(&Planner::safetyTimerThread, this);

        ROS_INFO("starting primitive tree thread");
        primitive_tree_thread = std::thread(&Planner::buildPrimitiveTree, this);
    }

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
void Planner::clickedPointcb(const geometry_msgs::PointStamped::ConstPtr& msg){
    goal = Eigen::VectorXd(2);
    goal(0) = msg->point.x;
    goal(1) = msg->point.y;
    _is_goal_set = true;
    _is_goal_reset = true;
}


/**********************************************************************
  Function to project a point into the costmap for planning purposes.

  Inputs:
    - Goal coordinates

  TODOS: Implement :)
***********************************************************************/
void Planner::projectIntoMap(const Eigen::Vector2d& goal){
    
}

/**********************************************************************
  Function callback which reads a published goal pose. As with the 
  clicked point callback function, this will set the _is_goal_set 
  field to true, which is required for planning to begin.

  Inputs:
    - PoseStamped message
***********************************************************************/
void Planner::goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal = Eigen::VectorXd(2);
    goal(0) = msg->pose.position.x;
    goal(1) = msg->pose.position.y;

    _is_goal_set = true;
    ROS_INFO("goal received!");
}


void Planner::occlusionPointscb(const geometry_msgs::PoseArray::ConstPtr& msg){
    _occ_point = Eigen::MatrixXd(2,3);

    if (msg->poses.size() > 0){
        geometry_msgs::Pose p1 = msg->poses[0];
        geometry_msgs::Pose p2 = msg->poses[1];

        _occ_point.row(0) = Eigen::Vector3d(p1.position.x, p1.position.y, p1.position.z);
        _occ_point.row(1) = Eigen::Vector3d(p2.position.x, p2.position.y, p2.position.z);
        _is_occ = true;
    }
}


/**********************************************************************
  Function which publishes the occupied cells of a global costmap.
***********************************************************************/
void Planner::publishOccupied(const ros::TimerEvent&){

    if (!_is_costmap_started)
        return;

    costmap_2d::Costmap2D* _map = global_costmap->getCostmap();
    vec_Vec2f padded = corridor::getOccupied(*_map);
    visualization_msgs::Marker paddedMsg;
    paddedMsg.header.frame_id = _frame_str;
    paddedMsg.header.stamp = ros::Time::now();
    paddedMsg.ns = "padded";
    paddedMsg.id = 420;
    paddedMsg.type=visualization_msgs::Marker::CUBE_LIST;
    paddedMsg.action=visualization_msgs::Marker::ADD;
    paddedMsg.scale.x = _map->getResolution();
    paddedMsg.scale.y = paddedMsg.scale.x;
    paddedMsg.scale.z = .1;
    paddedMsg.pose.orientation.w = 1;
    paddedMsg.color.r = 0.42;
    paddedMsg.color.g = 0.153;
    paddedMsg.color.b = 0.216;
    paddedMsg.color.a = .55;

    for(Eigen::Vector2d p : padded){
        geometry_msgs::Point pMs;
        pMs.x = p[0];
        pMs.y = p[1];
        pMs.z = 0;
        paddedMsg.points.push_back(pMs);
    }

    paddedLaserPub.publish(paddedMsg);
}

/**********************************************************************
  Function callback which reads a published map. This map is different
  from the local and global costmap, since it doesn't contain an 
  inflate layer around the obstacles.

  Inputs:
    - OccupancyGrid message
    
  TODOS: Decide if this map is needed or not for planning...
***********************************************************************/
void Planner::mapcb(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map = *msg;
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
void Planner::odomcb(const nav_msgs::Odometry::ConstPtr& msg){

    static ros::Time start;
    static Eigen::VectorXd _prevOdom;

	tf::Quaternion q(
	    msg->pose.pose.orientation.x,
	    msg->pose.pose.orientation.y,
	    msg->pose.pose.orientation.z,
	    msg->pose.pose.orientation.w
	);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	_odom = Eigen::VectorXd(3);
	_odom(0) = msg->pose.pose.position.x;
	_odom(1) = msg->pose.pose.position.y; 
	_odom(2) = yaw;

    if (_is_barn && !_is_goal_set){
        goal = Eigen::VectorXd(2);
        goal(0) = _barn_goal_dist*cos(yaw);
        goal(1) = _barn_goal_dist*sin(yaw);
        _is_goal_set = true;
    }

    _prevOdom = _odom;
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
void Planner::lasercb(const sensor_msgs::LaserScan::ConstPtr& msg){
    
    if (!_is_init)
        return;

    _obs.clear();

	for (int i = 0; i < msg->ranges.size(); i++){
		if (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min)
			continue;

        double d = msg->ranges[i];

		double angle = msg->angle_min + i*msg->angle_increment + _odom(2);
		double x = d*cos(angle) + _odom(0);
		double y = d*sin(angle) + _odom(1);

		_obs.push_back(Vec2f(x,y));
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
void Planner::globalPathcb(const nav_msgs::Path::ConstPtr& msg){
    
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
void Planner::visualizeTraj(){

    if (sentTraj.points.size() > 0) {
        visualization_msgs::Marker msg;
        msg.header.frame_id = _frame_str;
        msg.header.stamp = ros::Time::now();
        msg.ns = 'planTraj';
        msg.id = 80;
        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::LINE_STRIP;

        msg.scale.x = .1;
        msg.pose.orientation.w = 1;

        for(trajectory_msgs::JointTrajectoryPoint p : sentTraj.points){
            Eigen::Vector3d vel_vec(p.velocities[0],
                                    p.velocities[1],
                                    p.velocities[2]);
            double vel = vel_vec.norm();

            tinycolormap::Color color = tinycolormap::GetColor(vel/1.0, tinycolormap::ColormapType::Plasma);
            
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

}

/**********************************************************************
  Function which converts a Trajectory<D> into a JointTrajectory msg.
  This discretizes the Trajectory<D> and packages it such that the
  optimizes trajectory can be broadcast over ros to an external 
  tracking node. A _const_factor is used to adjust the speeds, since
  the solver doesn't do a very good job of respecting maximum velocity
  constraints.

  Inputs:
    - Trajectory generated by optimizer

  TODOS: 
    - Clean up some unnecessary lines (especially concerning factor)
***********************************************************************/
trajectory_msgs::JointTrajectory Planner::convertTrajToMsg(const std::vector<state>& trajectory){

    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = _frame_str;

    double next_t = 0.;
    for(int i = 0; i < trajectory.size(); i++){
        state x = trajectory[i];
        if(x.t >= next_t){
            
            trajectory_msgs::JointTrajectoryPoint p;
            p.positions.push_back(x.pos(0));
            p.positions.push_back(x.pos(1));
            p.positions.push_back(x.pos(2));

            p.velocities.push_back(x.vel(0));
            p.velocities.push_back(x.vel(1));
            p.velocities.push_back(x.vel(2));

            p.accelerations.push_back(x.accel(0));
            p.accelerations.push_back(x.accel(1));
            p.accelerations.push_back(x.accel(2));

            p.effort.push_back(x.jerk(0));
            p.effort.push_back(x.jerk(1));
            p.effort.push_back(x.jerk(2));

            p.time_from_start = ros::Duration(x.t);

            msg.points.push_back(p);

            next_t += _traj_dt;
        }
    }

    return msg;
}

void Planner::safetyTimerThread(){

    constexpr double safety_loop_rate_hz = 50;
    const std::chrono::milliseconds safety_loop_period(static_cast<int>(1000.0/safety_loop_rate_hz));

    while (ros::ok()){
        safetyLoop();

        std::this_thread::sleep_for(safety_loop_period);
    }
}


void Planner::buildPrimitiveTree(){

    while (ros::ok){
        // hold recovery lock when building tree, this will prevent the
        // motion primitives execution loop from running until tree is generated.
        std::unique_lock<std::mutex> recovery_lock(_recovery_mutex);
        _generate_primitive_cv.wait(recovery_lock, [this] {return _generate_primitives.load(); });

        _primitives_ready.store(false);

        ROS_WARN("generating motion primitive tree");
        _prim_tree->generate_motion_primitive_tree(3, {_odom[0], _odom[1], _odom[2]});
        _prim_tree->update_costmap(global_costmap);

        ROS_WARN("finding best trajectory");
        ros::Time s = ros::Time::now();
        _recovery_traj = _prim_tree->find_best_trajectory();
        ROS_WARN("Time was %.4f", (ros::Time::now()-s).toSec());

        _primitives_ready.store(true);
        _generate_primitives.store(false);
        _primitive_cv.notify_one();
    }

}


void Planner::safetyLoop(){

    if (!_is_init)
        return;

    std::unique_lock<std::mutex> robo_state_lock(_robo_state_mutex, std::defer_lock);
    std::unique_lock<std::mutex> solver_state_lock(_solver_state_mutex, std::defer_lock);

    // already in recovery state, no need to do anything
    if (_robo_state == RECOVERY){
        return;
    }
    
    const costmap_2d::Costmap2D& costmap = *global_costmap->getCostmap();
    unsigned int mx, my;
    
    if (costmap.worldToMap(_odom(0), _odom(1), mx, my)){
        unsigned char cost = costmap.getCost(mx, my);
        if (cost != costmap_2d::INSCRIBED_INFLATED_OBSTACLE && cost != costmap_2d::LETHAL_OBSTACLE)
            return;
    }

    // by this point we are close to obstacles and recovery behavior should be triggered
    solver_state_lock.lock();
    robo_state_lock.lock();

    if (solver_state.status.data != 0){
        _robo_state.store(RECOVERY);
        _generate_primitives.store(true);
        _generate_primitive_cv.notify_one();

        solver_state_lock.unlock();
        robo_state_lock.unlock();

        ROS_WARN("entering recovery mode");

        trajectory_msgs::JointTrajectory msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = _frame_str;

        trajPub.publish(msg);

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        cmdVelPub.publish(cmd_vel);

        state_transition_start_t = ros::Time::now();
        
        return;
    }

    robo_state_lock.unlock();
    solver_state_lock.unlock();

}

void Planner::primitivesLoop(const ros::TimerEvent&){
    
    static int index = 0;

    if (_robo_state != RECOVERY)
        return;

    // attempting to acquire this mutex means that it will wait to start
    // if primitives are not yet generated.
    std::unique_lock<std::mutex> recovery_lock(_recovery_mutex);
    if (!_primitives_ready)
        _primitive_cv.wait(recovery_lock, [this] {return _primitives_ready.load(); });

    if ((ros::Time::now()-state_transition_start_t).toSec() < 2)
        return;

    if (!_primitive_started){
        _primitive_started = true;
        primitive_segment_start= ros::Time::now();
    }

    const MotionPrimitiveNode* n = _recovery_traj[index];

    if(n->isDone((ros::Time::now()-primitive_segment_start).toSec())){

        ROS_INFO("switching primitives");
        if (++index >= _recovery_traj.size()){
            index = 0;
            _recovery_traj.clear();

            std::unique_lock<std::mutex> robo_state_lock(_robo_state_mutex);
            _robo_state.store(NOMINAL);
            robo_state_lock.unlock();

            _primitive_started = false;

            ROS_INFO("switching back to nominal behavior");

            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            cmdVelPub.publish(cmd_vel);

            state_transition_start_t = ros::Time::now();

            return;
        }

        n = _recovery_traj[index];
        primitive_segment_start = ros::Time::now();
        // n->isDone((ros::Time::now()-_primitive_segment_start).toSec());
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = n->motion_primitive.at("speed");
    cmd_vel.angular.z = n->motion_primitive.at("angular_velocity");
    cmdVelPub.publish(cmd_vel);
    
}

/**********************************************************************
  Function for the main control loop of the planner, calls the plan()
  method which takes care of overall planning logic.

  Inputs:
    - Just a timer event which controls the rate at which the control
      loop is run.
***********************************************************************/
void Planner::controlLoop(const ros::TimerEvent&){

    static int count = 0;

    if (!_is_init || !_is_goal_set || !_is_costmap_started)
        return;

    if ((_odom(1)-goal(1))*(_odom(1)-goal(1))+(_odom(0)-goal(0))*(_odom(0)-goal(0)) < .2)
        return;

    if (_robo_state == RECOVERY)
        return;

    if (_robo_state == NOMINAL && (ros::Time::now()-state_transition_start_t).toSec() < 2)
        return;
    

    /*************************************
    ************* UPDATE MAP *************
    **************************************/

    // global_costmap->resetLayers();
    global_costmap->updateMap();

    /*************************************
    **************** PLAN ****************
    **************************************/

    if (_plan_once && _planned)
        return;

    const costmap_2d::Costmap2D& _map = *global_costmap->getCostmap();

    unsigned int map_x, map_y;
    _map.worldToMap(_odom(0), _odom(1), map_x, map_y);

    if (!plan())
        count++;
    else
        count = 0;

    solverStatePub.publish(solver_state);

    if (count >= _failsafe_count){
        if (plan(true))
            count = 0;

        solverStatePub.publish(solver_state);
    }

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
bool Planner::plan(bool is_failsafe){

    if (is_failsafe){
        ROS_INFO("*******************************");
        ROS_INFO("*** FAILSAFE MODE  ENGAGED ****");
        ROS_INFO("*******************************");
    }

    costmap_2d::Costmap2D* _map = global_costmap->getCostmap();
    hPolys.clear();
    
    ros::Time a = ros::Time::now();

    bool not_first = false;

    ROS_INFO("Setting up initial and final conditions");
    Eigen::MatrixXd initialPVAJ(3,4);
    Eigen::MatrixXd finalPVAJ(3,4);

    finalPVAJ << Eigen::Vector3d(goal(0),goal(1),0), 
                Eigen::Vector3d::Zero(), 
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero();

    if (sentTraj.points.size() == 0 || _is_teleop || _is_goal_reset){
        initialPVAJ <<   Eigen::Vector3d(_odom(0),_odom(1),0), 
                        Eigen::Vector3d::Zero(), 
                        Eigen::Vector3d::Zero(),
                        Eigen::Vector3d::Zero();
    }
    else{
        not_first = true;
        double factor = _const_factor;
      
        double t = (a-start).toSec() + _lookahead;
        double tmpT = t;
        
        int trajInd = std::min((int) (t/_traj_dt), (int) sentTraj.points.size()-1);

        trajectory_msgs::JointTrajectoryPoint p = sentTraj.points[trajInd];

        geometry_msgs::PointStamped poseMsg;
        poseMsg.header.frame_id = "map";
        poseMsg.header.stamp = ros::Time::now();
        poseMsg.point.x = p.positions[0];
        poseMsg.point.y = p.positions[1];
        poseMsg.point.z = p.positions[2];
        initPointPub.publish(poseMsg);

        initialPVAJ.col(0) = Eigen::Vector3d(p.positions[0], 
                                            p.positions[1], 
                                            p.positions[2]);
                                            
        // start from robot not moving if failsafe
        if (is_failsafe){
            initialPVAJ.col(1) = Eigen::Vector3d(0,0,0);
            initialPVAJ.col(2) = Eigen::Vector3d(0,0,0);
            initialPVAJ.col(3) = Eigen::Vector3d(0,0,0);
        }else{
            initialPVAJ.col(1) = Eigen::Vector3d(p.velocities[0],
                                                 p.velocities[1],
                                                 p.velocities[2]);
            initialPVAJ.col(2) = Eigen::Vector3d(p.accelerations[0],
                                                 p.accelerations[1],
                                                 p.accelerations[2]);

            // effort contains jerk information in our case
            initialPVAJ.col(3) = Eigen::Vector3d(p.effort[0],
                                                 p.effort[1],
                                                 p.effort[2]);
        } 
    }

    /*************************************
    ************ PERFORM  JPS ************
    **************************************/

    ROS_INFO("Running JPS");
    JPSPlan jps;
    unsigned int sX, sY, eX, eY;
    _map->worldToMap(_odom(0), _odom(1), sX, sY);
    // _map->worldToMap(initialPVAJ.col(0)[0], initialPVAJ.col(0)[1], sX, sY);
    _map->worldToMap(goal(0), goal(1), eX, eY);

    jps.set_start(sX, sY);
    jps.set_destination(eX, eY);
    jps.set_occ_value(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    jps.set_map(_map->getCharMap(), _map->getSizeInCellsX(), _map->getSizeInCellsY(),
                _map->getOriginX(), _map->getOriginY(), _map->getResolution());
    jps.JPS();

    ROS_INFO("getting path");
    std::vector<Eigen::Vector2d> jpsPath = jps.getPath(_simplify_jps);
    ROS_INFO("got path");

    trajectory_msgs::JointTrajectory empty_msg; 
    populateSolverState(solver_state, hPolys, empty_msg, initialPVAJ, finalPVAJ, 0);
    if (jpsPath.size() == 0){
        ROS_ERROR("JPS failed to find path");
        solver_state.status.data = 1;
        return false;
    }

    ROS_INFO("finished JPS");
    
    /*************************************
    ************* REFINE JPS *************
    **************************************/

    Eigen::Vector2d goalPoint;
    if (_plan_in_free){

        nav_msgs::Path jpsMsgFree;
        jpsMsgFree.header.stamp = ros::Time::now();
        jpsMsgFree.header.frame_id = _frame_str;
        std::vector<Eigen::Vector2d> jpsFree = getJPSInFree(jpsPath, *_map);
        ROS_INFO("jpsFree size: %lu", jpsFree.size());

        for(Eigen::Vector2d p : jpsFree){
            geometry_msgs::PoseStamped pMsg;
            pMsg.header = jpsMsgFree.header;
            pMsg.pose.position.x = p(0);
            pMsg.pose.position.y = p(1);
            pMsg.pose.position.z = 0;
            pMsg.pose.orientation.w = 1;
            jpsMsgFree.poses.push_back(pMsg);
            
        }

        jpsPubFree.publish(jpsMsgFree);

        finalPVAJ << Eigen::Vector3d(jpsFree.back()[0],jpsFree.back()[1],0), 
                Eigen::Vector3d::Zero(), 
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero();

        if (jpsFree.size() == 0){
            ROS_ERROR("JPSFree has size 0");
            solver_state.status.data = 1;
            return false;
        }

        jpsPath = jpsFree;
    } else {
        std::vector<Eigen::Vector2d> newJPSPath;
        if (getJPSIntersectionWithSphere(
            jpsPath, 
            newJPSPath,
            Eigen::Vector2d(_odom[0], _odom[1]),
            _max_dist_horizon, goalPoint)){

            jpsPath = newJPSPath;

            finalPVAJ << Eigen::Vector3d(goalPoint[0], goalPoint[1],0), 
                Eigen::Vector3d::Zero(), 
                Eigen::Vector3d::Zero(),
                Eigen::Vector3d::Zero();

            ROS_INFO_STREAM("JPS path intersected circle at " << goalPoint);
        } else{
            ROS_WARN("JPS path did not intersect circle around robot");
        }

    }

    /*************************************
    ******** PUBLISH JPS TO RVIZ *********
    **************************************/

    nav_msgs::Path jpsMsg;
    jpsMsg.header.stamp = ros::Time::now();
    jpsMsg.header.frame_id = _frame_str;

    for(Eigen::Vector2d p : jpsPath){
        geometry_msgs::PoseStamped pMsg;
        pMsg.header = jpsMsg.header;
        pMsg.pose.position.x = p(0);
        pMsg.pose.position.y = p(1);
        pMsg.pose.position.z = 0;
        pMsg.pose.orientation.w = 1;
        jpsMsg.poses.push_back(pMsg);
        
    }

    jpsPub.publish(jpsMsg);

    /*************************************
    ********* GENERATE POLYTOPES *********
    **************************************/
    
    ROS_INFO("creating corridor");
    // don't need to clear hPolys before calling because method will do it
    if (!corridor::createCorridorJPS(jpsPath, *_map, _obs, hPolys)){
        ROS_ERROR("CORRIDOR GENERATION FAILED");
        solver_state.status.data = 2;
        return false;
    }
    
    if (!isInPoly(hPolys[0], Eigen::Vector2d(initialPVAJ(0,0), initialPVAJ(1,0))) )
        hPolys.insert(hPolys.begin(), corridor::genPoly(*_map, initialPVAJ(0,0), initialPVAJ(1,0)));
    
    if (!isInPoly(hPolys.back(), Eigen::Vector2d(finalPVAJ(0,0), finalPVAJ(1,0))) ){
        ROS_WARN("end was not in poly, adding extra polygon to correct this");
        hPolys.insert(hPolys.end(), corridor::genPoly(*_map, finalPVAJ(0,0), finalPVAJ(1,0)));
    }

    bool is_in_corridor = false;

    // if adjacent polytopes don't overlap, don't plan
    for(int p = 0; p < hPolys.size()-1; p++){
        if (!geo_utils::overlap(hPolys[p], hPolys[p+1])){
            ROS_ERROR("CORRIDOR IS NOT FULLY CONNECTED");
            solver_state.status.data = 2;
            return false;
        }

        if (!is_in_corridor)
            is_in_corridor = isInPoly(hPolys[p], Eigen::Vector2d(initialPVAJ(0,0), initialPVAJ(1,0)));
    }

    if (!is_in_corridor)
        ROS_WARN("(%.2f, %.2f) NOT IN CORRIDOR", initialPVAJ(0,0), initialPVAJ(1,0));

    ROS_INFO("%d\t%d",!is_in_corridor, is_failsafe);


    solver_state.polys.poses.clear();
    corridor::corridorToMsg(hPolys, solver_state.polys);

    corridor::visualizePolytope(hPolys, meshPub, edgePub);

    ROS_INFO("generated corridor of size %lu", hPolys.size());

    /*************************************
    ******** GENERATE  TRAJECTORY ********
    **************************************/

    state initialState;
    state finalState;

    initialState.setPos(initialPVAJ(0,0), initialPVAJ(1,0), initialPVAJ(2,0));
    initialState.setVel(initialPVAJ(0,1), initialPVAJ(1,1), initialPVAJ(2,1));
    initialState.setAccel(initialPVAJ(0,2), initialPVAJ(1,2), initialPVAJ(2,2));
    initialState.setJerk(initialPVAJ(0,3), initialPVAJ(1,3), initialPVAJ(2,3));

    finalState.setPos(finalPVAJ.col(0));
    finalState.setVel(finalPVAJ.col(1));
    finalState.setAccel(finalPVAJ.col(2));
    finalState.setJerk(finalPVAJ.col(3));

    ROS_INFO("setting up");
    solver.setX0(initialState);
    solver.setXf(finalState);
    solver.setPolytopes(hPolys);

    if (_is_occ)
        solver.setOcc(_occ_point);

    ROS_INFO("solving");
    if (!solver.genNewTraj()){
        ROS_ERROR("solver could not find trajectory");
        solver_state.status.data = 3;
        return false;
    } else{
        std::cout << termcolor::green << "solver found trajectory" << std::endl;
    }


    solver.fillX();
    /*************************************
    ******** STITCH  TRAJECTORIES ********
    **************************************/

    ROS_INFO("stitching trajectories");

    if (sentTraj.points.size() != 0){


        double t1 = std::round((ros::Time::now()-start).toSec()*10.)/10.;
        double t2 = std::round(((a-start).toSec() + _lookahead)*10.)/10.;

        // ROS_INFO("[%.2f] t1 is %.2f\tt2 is %.2f", (ros::Time::now()-start).toSec(),t1, t2);

        int startInd = std::min((int)(t1/_traj_dt), (int) sentTraj.points.size()-1)+1;
        int trajInd = std::min((int) (t2/_traj_dt), (int) sentTraj.points.size()-1);

        // ROS_INFO("[%.2f] startInd is %d\ttrajInd is %d", (ros::Time::now()-start).toSec(),startInd, trajInd);

        trajectory_msgs::JointTrajectory aTraj, bTraj;
        bTraj = convertTrajToMsg(solver.X_temp_);
        
        // if current trajectory violates corridor constraints but previous trajectory
        // isn't intersecting any lethal obstacles, just keep the old trajectory
        // if (isTrajOutsidePolys(bTraj, hPolys, .2) && !isTrajOverlappingObs(sentTraj, *_map) ){
        //     ROS_ERROR("corridor violation was too high and sentTraj isn't overlapping obs");
        //     ROS_ERROR("sentTraj overlapping obstacles? %d", isTrajOverlappingObs(sentTraj, *_map));
        //     solver_state.status.data = 4;
        //     return false;
        // }

        for(int i = startInd; i < trajInd; i++){
            aTraj.points.push_back(sentTraj.points[i]);
            aTraj.points.back().time_from_start = ros::Duration((i-startInd)*_traj_dt);
        }

        double startTime = (trajInd-startInd)*_traj_dt;

        for(int i = 0; i < bTraj.points.size(); i++){
            aTraj.points.push_back(bTraj.points[i]);
            aTraj.points.back().time_from_start = ros::Duration(startTime+i*_traj_dt);
        }

        aTraj.header.frame_id = _frame_str;
        aTraj.header.stamp = ros::Time::now();

        sentTraj = aTraj;
        _is_goal_reset = false;
        start = ros::Time::now();
    }else{
        sentTraj = convertTrajToMsg(solver.X_temp_);
        start = ros::Time::now();
    }

    if(!_is_teleop)
        trajPub.publish(sentTraj);

    ROS_INFO("visualizing");
    visualizeTraj();
    // pubCurrPoly();

    pubPolys();
    _planned = true;
    
    populateSolverState(solver_state, hPolys, sentTraj, initialPVAJ, finalPVAJ, 0);

    double totalT = (ros::Time::now() - a).toSec();
    std::cout << "total time is " << totalT << std::endl;

    return true;

}


/**********************************************************************
  Function to publish current goal on a timer. 

  Inputs:
    - Just a timer event which controls the rate at which the function
      is run.

  TODOS: 
    - Decide if this function is necesary or not.
***********************************************************************/
void Planner::goalLoop(const ros::TimerEvent&){

    if (!_is_goal_set)
        return;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = _frame_str;

    msg.pose.position.x = goal(0);
    msg.pose.position.y = goal(1);
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1;

    goalPub.publish(msg);
}

/**********************************************************************
  Function to publish the current polytope corridor as a PoseArray
  message. This function can be ignored as it's primary use is for an
  external backward reachable set project.

  Inputs:
    - None

  TODOS: 
    - Come up with a better way to generate intermediate goals
***********************************************************************/
void Planner::pubPolys(){
    
    geometry_msgs::PoseArray corridorMsg;
    geometry_msgs::PoseArray wptMsg;

    corridor::corridorToMsg(hPolys, corridorMsg);
    // for(Eigen::MatrixX4d& poly : hPolys){
    //     std::cout << "\033[1;31m******************POLY******************\033[0m" << std::endl;
    //     std::cout << poly << std::endl;
    // }
    // intGoalPub.publish(wptMsg);
    corridorPub.publish(corridorMsg);

    // std::vector<Eigen::MatrixX4d> helpers = corridor::addHelperPolysChebyshev(*global_costmap->getCostmap(), hPolys);
    std::vector<Eigen::MatrixX4d> helpers = corridor::addHelperPolysTrajectory(*global_costmap->getCostmap(), hPolys, sentTraj);
    
    geometry_msgs::PoseArray helperCorridorMsg;
    corridor::corridorToMsg(helpers, helperCorridorMsg);
    // corridor::visualizePolytope(helpers, helperMeshPub, helperEdgePub, true);

    // helperPolyPub.publish(helperCorridorMsg);
    // corridorPub.publish(helperCorridorMsg);
    
    return;
}

void Planner::pubCurrPoly(){

    geometry_msgs::PoseArray msg;
    
    // go backwards because we want the poly closest to end of corridor in
    // case of intersections
    for(int i = hPolys.size()-1; i >=0; i--){
        Eigen::MatrixX4d poly = hPolys[i];

        if (isInPoly(poly, Eigen::Vector2d(_odom(0), _odom(1)))){
            for(int j = 0; j < poly.rows(); j++){
                geometry_msgs::Pose p;
                p.orientation.x = poly.row(j)[0];
                p.orientation.y = poly.row(j)[1];
                p.orientation.z = poly.row(j)[2];
                p.orientation.w = poly.row(j)[3];
                msg.poses.push_back(p);
            }

            ROS_INFO("PUBLISHING POLY");
            currPolyPub.publish(msg);
            break;
        }
    }


    return;
}

void populateSolverState(robust_fast_navigation::SolverState& state, 
                         const std::vector<Eigen::MatrixX4d>& hPolys,
                         const trajectory_msgs::JointTrajectory& traj,
                         const Eigen::MatrixXd& initialPVAJ,
                         const Eigen::MatrixXd& finalPVAJ,
                         int status){

    state.polys.poses.clear();
    corridor::corridorToMsg(hPolys, state.polys);

    state.initialPVA.positions = {initialPVAJ(0,0), initialPVAJ(1,0), initialPVAJ(2,0)};
    state.initialPVA.velocities = {initialPVAJ(0,1), initialPVAJ(1,1), initialPVAJ(2,1)};
    state.initialPVA.accelerations = {initialPVAJ(0,2), initialPVAJ(1,2), initialPVAJ(2,2)};

    state.finalPVA.positions = {finalPVAJ(0,0), finalPVAJ(1,0), finalPVAJ(2,0)};
    state.finalPVA.velocities = {finalPVAJ(0,1), finalPVAJ(1,1), finalPVAJ(2,1)};
    state.finalPVA.accelerations = {finalPVAJ(0,2), finalPVAJ(1,2), finalPVAJ(2,2)};

    state.trajectory = traj;
    
    state.status.data = status;

    return;
    
}
