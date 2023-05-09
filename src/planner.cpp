#include <math.h>
#include <cmath>
#include <vector>
#include <string>


#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>


#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/utils.h>
#include <robust_fast_navigation/spline.h>
#include <robust_fast_navigation/planner.h>
#include <robust_fast_navigation/corridor.h>
#include <robust_fast_navigation/tinycolormap.hpp>

/**********************************************************************
  Constructor to read in ROS params, setup subscribers & publishers,
  ros timers, and initialize class fields.
***********************************************************************/
Planner::Planner(ros::NodeHandle& nh){

    // ROS Params
    nh.param("robust_planner/traj_dt", _traj_dt, .1);
    nh.param("robust_planner/teleop", _is_teleop, false);
    nh.param("robust_planner/lookahead", _lookahead, .15);
    nh.param("robust_planner/planner_frequency", _dt, .1);
    nh.param("robust_planner/max_velocity", _max_vel, 1.0);
    nh.param("robust_planner/plan_once", _plan_once, false);
    nh.param("robust_planner/const_factor", _const_factor, 6.0);
    nh.param("robust_planner/simplify_jps", _simplify_jps, false);
    nh.param("robust_planner/failsafe_count", _failsafe_count, 2);
    nh.param("robust_planner/is_barn", _is_barn, false);
    nh.param<std::string>("robust_planner/frame", _frame_str, "map");

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
    initPointPub = 
        nh.advertise<geometry_msgs::PointStamped>("/initPoint", 0);

    goalPub = 
        nh.advertise<geometry_msgs::PoseStamped>("/global_planner/goal", 0);

    paddedLaserPub =
        nh.advertise<visualization_msgs::Marker>("/paddedObs", 0);

    jpsPub = 
        nh.advertise<nav_msgs::Path>("/jpsPath", 0);

    jpsPointsPub = 
        nh.advertise<visualization_msgs::Marker>("/jpsPoints", 0);

    currPolyPub = 
        nh.advertise<geometry_msgs::PoseArray>("/currPoly", 0);

    intGoalPub = 
        nh.advertise<geometry_msgs::PoseArray>("/intermediate_goal", 0);

    // Subscribers
    mapSub = nh.subscribe("/map", 1, &Planner::mapcb, this);
    goalSub = nh.subscribe("/planner_goal", 1, &Planner::goalcb, this);
    laserSub = nh.subscribe("/front/scan", 1, &Planner::lasercb, this);
    odomSub = nh.subscribe("/odometry/filtered", 1, &Planner::odomcb, this);
    clickedPointSub = nh.subscribe("/clicked_point", 1, &Planner::clickedPointcb, this);
    pathSub = nh.subscribe("/global_planner/planner/plan", 1, &Planner::globalPathcb, this);

    // Timers
    goalTimer = nh.createTimer(ros::Duration(_dt/2.0), &Planner::goalLoop, this);
    publishTimer = nh.createTimer(ros::Duration(_dt*2), &Planner::publishOccupied, this);
    controlTimer = nh.createTimer(ros::Duration(_dt), &Planner::controlLoop, this);

    _is_init = false;
    _is_goal_set = false;

    _map_received = false;
    _is_costmap_started = false;

    _prev_jps_cost = -1;

    ROS_INFO("Initialized planner!");
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
    // ROS_INFO("goal received!");
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
        goal(0) = 10*cos(yaw);
        goal(1) = 10*sin(yaw);
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
template <int D>
trajectory_msgs::JointTrajectory Planner::convertTrajToMsg(const Trajectory<D> &traj){

    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = _frame_str;

    double factor = traj.getMaxVelRate() / _max_vel;
    // ROS_ERROR("MAX VEL RATE IS %.2f", traj.getMaxVelRate());
    factor = _const_factor;
    double t = 0.;

    while (t < traj.getTotalDuration()){
        Eigen::Vector3d pos = traj.getPos(t);
        Eigen::Vector3d vel = traj.getVel(t);
        Eigen::Vector3d acc = traj.getAcc(t);

        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions.push_back(pos(0));
        pt.positions.push_back(pos(1));
        pt.positions.push_back(pos(2));

        pt.velocities.push_back(vel(0)/factor);
        pt.velocities.push_back(vel(1)/factor);
        pt.velocities.push_back(vel(2)/factor);

        pt.accelerations.push_back(acc(0)/(factor*factor));
        pt.accelerations.push_back(acc(1)/(factor*factor));
        pt.accelerations.push_back(acc(2)/(factor*factor));

        pt.time_from_start = ros::Duration(t*factor);

        msg.points.push_back(pt);
        
       
        t += _traj_dt/factor;
    }

    return msg;
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

    // local_costmap->resetLayers();
    // local_costmap->updateMap();

    /*************************************
    ************* UPDATE MAP *************
    **************************************/

    // global_costmap->resetLayers();
    global_costmap->updateMap();

    // costmap_2d::Costmap2D* _map = local_costmap->getCostmap();
    
    if (!plan())
        count++;
    else
        count = 0;

    if (count >= _failsafe_count)
        plan(true);
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
    
    ros::Time a = ros::Time::now();

    Eigen::Matrix3d initialPVA;
    bool not_first = false;

    Eigen::Matrix3d finalPVA;
    finalPVA << Eigen::Vector3d(goal(0),goal(1),0), 
                Eigen::Vector3d::Zero(), 
                Eigen::Vector3d::Zero();

    if (sentTraj.points.size() == 0 || _is_teleop || _is_goal_reset){
        initialPVA <<   Eigen::Vector3d(_odom(0),_odom(1),0), 
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

        initialPVA.col(0) = Eigen::Vector3d(p.positions[0], 
                                            p.positions[1], 
                                            p.positions[2]);
        // start from robot not moving
        if (is_failsafe){
            initialPVA.col(1) = Eigen::Vector3d(0,0,0);
            initialPVA.col(2) = Eigen::Vector3d(0,0,0);
        }else{
            initialPVA.col(1) = Eigen::Vector3d(p.velocities[0]*factor, 
                                                p.velocities[1]*factor, 
                                                p.velocities[2]*factor);
            initialPVA.col(2) = Eigen::Vector3d(p.accelerations[0]*factor*factor,
                                                p.accelerations[1]*factor*factor,
                                                p.accelerations[2]*factor*factor);
        }
    }

    /*************************************
    ************ PERFORM  JPS ************
    **************************************/

    JPSPlan jps;
    unsigned int sX, sY, eX, eY;
    _map->worldToMap(initialPVA.col(0)[0], initialPVA.col(0)[1], sX, sY);
    _map->worldToMap(goal(0), goal(1), eX, eY);

    jps.set_start(sX, sY);
    jps.set_destination(eX, eY);
    jps.set_occ_value(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    jps.set_map(_map->getCharMap(), _map->getSizeInCellsX(), _map->getSizeInCellsY(),
                _map->getOriginX(), _map->getOriginY(), _map->getResolution());
    jps.JPS();

    std::vector<Eigen::Vector2d> jpsPath = jps.getPath(_simplify_jps);

    if (jpsPath.size() == 0){
        ROS_ERROR("JPS failed to find path");
        return false;
    }

    double cost = 0;
    for(int i = 0; i < jpsPath.size(); i++){
        double x, y;
        _map->mapToWorld(jpsPath[i](0), jpsPath[i](1), x, y);
        jpsPath[i] = Eigen::Vector2d(x,y);

        if (i > 0)
            cost += (jpsPath[i]-jpsPath[i-1]).norm();
    }

    // bool _is_old_jps_valid = true;

    // if (_prev_jps_path.size() > 0){
    //     for(int i = 0; i < _prev_jps_path.size(); i++)

    //     for(int i = 0; i < _prev_jps_path.size()-1; i++){
    //         if (jps.isBlocked(_prev_jps_path[i], _prev_jps_path[i+1], false)){
    //             _is_old_jps_valid = false;
    //             break;
    //         } else{
    //             std::cout << _prev_jps_path[i].transpose()  << " to " << 
    //                 _prev_jps_path[i+1].transpose() << " is not blocked" << std::endl;
    //         }
    //     }
    // }

    // if (_prev_jps_path.size() > 0 && fabs(_prev_jps_cost/cost - 1) < .01 &&
    //     _is_old_jps_valid){
    //     ROS_INFO("sticking with old JPS as new one wasn't significantly different");
    //     jpsPath = _prev_jps_path;
    // }
    // else{
    //     _prev_jps_path = jpsPath;
    //     _prev_jps_cost = cost;
    // }


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
    // don't neet to clear hPolys before calling because method will do it
    if (!corridor::createCorridorJPS(jpsPath, *_map, _obs, hPolys)){
        ROS_ERROR("CORRIDOR GENERATION FAILED");
        return false;
    }
    
    // if adjacent polytopes don't overlap, don't plan
    for(int p = 0; p < hPolys.size()-1; p++){
        if (!geo_utils::overlap(hPolys[p], hPolys[p+1])){
            ROS_ERROR("CORRIDOR IS NOT FULLY CONNECTED");
            return false;
        }
    }

    corridor::visualizePolytope(hPolys, meshPub, edgePub);

    ROS_INFO("generated corridor of size %lu", hPolys.size());

    /*************************************
    ******** GENERATE  TRAJECTORY ********
    **************************************/

    // initial and final states

    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds(0) = 1.8;   //v_max
    magnitudeBounds(1) = .8;   //omg_max
    magnitudeBounds(2) = .8;    //theta_max
    magnitudeBounds(3) = -1;     //thrust_min
    magnitudeBounds(4) = .2;    //thrust_max
    penaltyWeights(0) = 1e4;    //pos_weight
    penaltyWeights(1) = 1e4;    //vel_weight
    penaltyWeights(2) = 1e4;    //omg_weight
    penaltyWeights(3) = 1e4;    //theta_weight
    penaltyWeights(4) = 1e5;    //thrust_weight
    physicalParams(0) = .1;    // mass
    physicalParams(1) = 9.81;   // gravity
    physicalParams(2) = 0;      // drag
    physicalParams(3) = 0;      // drag
    physicalParams(4) = 0;      // drag
    physicalParams(5) = .0001;  // speed smooth factor

    gcopter::GCOPTER_PolytopeSFC gcopter;

    ROS_INFO("setting up");
    if(!gcopter.setup(
        20.0,   //time weight
        initialPVA, 
        finalPVA,
        hPolys,
        1e6,    // lengthPerPiece
        1e-2,   // smoothing factor
        16,     // integral resolution
        magnitudeBounds,
        penaltyWeights,
        physicalParams
    )){
        ROS_ERROR("optimizer setup failed");
        return false;
    }

    ROS_INFO("solving");
    Trajectory<5> newTraj;
    if (std::isinf(gcopter.optimize(newTraj, 1e-5))){
        ROS_ERROR("solver could not find trajectory");
        return false;
    }

    if (newTraj.getMaxVelRate() > _const_factor){
        ROS_ERROR("new trajectory was way too fast (%.2f m/s)!", newTraj.getMaxVelRate());
        return false;
    }
    /*************************************
    ******** STITCH  TRAJECTORIES ********
    **************************************/

    ROS_INFO("stitching trajectories");
    // ROS_INFO("sentTraj size is %lu", sentTraj.points.size());
    if (sentTraj.points.size() != 0){// || _is_goal_reset){
        // ROS_INFO("sentTraj points size is %lu", sentTraj.points.size());

        double maxVel = traj.getMaxVelRate();
        double factor = maxVel / _max_vel;
        factor = _const_factor;

        double t1 = std::round((ros::Time::now()-start).toSec()*10.)/10.;
        double t2 = std::round(((a-start).toSec() + _lookahead)*10.)/10.;

        // ROS_INFO("[%.2f] t1 is %.2f\tt2 is %.2f", (ros::Time::now()-start).toSec(),t1, t2);

        int startInd = std::min((int)(t1/_traj_dt), (int) sentTraj.points.size()-1)+1;
        int trajInd = std::min((int) (t2/_traj_dt), (int) sentTraj.points.size()-1);

        // ROS_INFO("[%.2f] startInd is %d\ttrajInd is %d", (ros::Time::now()-start).toSec(),startInd, trajInd);

        trajectory_msgs::JointTrajectory aTraj, bTraj;
        bTraj = convertTrajToMsg(newTraj);
        
        // if current trajectory violates corridor constraints but previous trajectory
        // isn't intersecting any lethal obstacles, just keep the old trajectory
        if (isTrajOutsidePolys(bTraj, hPolys, .2) && !isTrajOverlappingObs(sentTraj, *_map) ){
            ROS_ERROR("corridor violation was too high and sentTraj isn't overlapping obs");
            ROS_ERROR("sentTraj overlapping obstacles? %d", isTrajOverlappingObs(sentTraj, *_map));
            return false;
        }

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
        // ROS_INFO("trajectory has been overwritten");
        traj = newTraj;
        sentTraj = convertTrajToMsg(newTraj);

        start = ros::Time::now();
    }

    if(!_is_teleop)
        trajPub.publish(sentTraj);

    visualizeTraj();

    if (_plan_once){
        pubPolys();
        exit(0);
    }

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

    geometry_msgs::PoseArray msg;
    geometry_msgs::PoseArray wptMsgs;
    wptMsgs.header.frame_id = _frame_str;
    wptMsgs.header.stamp = ros::Time::now();

    geometry_msgs::Pose pMsg;
    pMsg.position.x = _odom(0);
    pMsg.position.y = _odom(1);
    pMsg.position.z = 0;
    pMsg.orientation.w = 1;
    wptMsgs.poses.push_back(pMsg);

    int lastStopped = 1;

    for(int p = 0; p < hPolys.size(); p++){

        Eigen::MatrixX4d currPoly = hPolys[p];

        for(int i = 0; i < currPoly.rows(); ++i){
            // skip the Z-axis planes
            if (fabs(currPoly.row(i)[0]) < 1e-2 && fabs(currPoly.row(i)[1]) < 1e-2)
                continue;

            Eigen::MatrixX4d expandedPoly = expandPoly(currPoly, .05);
            geometry_msgs::Pose p;
            p.orientation.x = expandedPoly.row(i)[0];
            p.orientation.y = expandedPoly.row(i)[1];
            p.orientation.z = expandedPoly.row(i)[2];
            p.orientation.w = expandedPoly.row(i)[3];
            msg.poses.push_back(p);
        }
        ROS_INFO("**********");

        // all zeroes used as a separator between polytopes
        geometry_msgs::Pose separator;
        separator.orientation.x = 0;
        separator.orientation.y = 0;
        separator.orientation.z = 0;
        separator.orientation.w = 0;
        msg.poses.push_back(separator);

        for(int i = lastStopped; i < sentTraj.points.size(); i++){
            trajectory_msgs::JointTrajectoryPoint p = sentTraj.points[i];
            Eigen::Vector3d pos(p.positions[0], p.positions[1], p.positions[2]);
            bool is_in = true;
            for(int j = 0; j < currPoly.rows(); j++){
                Eigen::Vector4d plane = currPoly.row(j);
                double dot = plane[0]*pos[0]+plane[1]*pos[1]+plane[2]*pos[2]+plane[3];
                if (dot > 0){
                    is_in = false;
                    lastStopped = i;
                    break;
                }
            }

            if (!is_in){
                ROS_INFO("i is %d", i);
                trajectory_msgs::JointTrajectoryPoint p = sentTraj.points[std::max(i-1,0)];
                Eigen::Vector3d pos(p.positions[0], p.positions[1], p.positions[2]);
                geometry_msgs::Pose pM;
                pM.position.x = pos(0);
                pM.position.y = pos(1);
                pM.position.z = pos(2);
                pM.orientation.w = 1;
                wptMsgs.poses.push_back(pM);
                break;
            }

        }

    }

    ROS_INFO("last point!");
    trajectory_msgs::JointTrajectoryPoint p = sentTraj.points.back();
    Eigen::Vector3d pos(p.positions[0], p.positions[1], p.positions[2]);
    geometry_msgs::Pose pM;
    pM.position.x = pos(0);
    pM.position.y = pos(1);
    pM.position.z = pos(2);
    pM.orientation.w = 1;
    wptMsgs.poses.push_back(pM);

    intGoalPub.publish(wptMsgs);
    currPolyPub.publish(msg);

}
