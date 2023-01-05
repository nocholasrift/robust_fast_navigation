#include <math.h>
#include <vector>
#include <tf/tf.h>
#include <ros/ros.h>
#include "gcopter/gcopter.hpp"
#include "gcopter/quickhull.hpp"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <decomp_util/seed_decomp.h>
#include <decomp_util/line_segment.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <decomp_geometry/geometric_utils.h>

bool _is_init;
bool _is_traj_generated;
Trajectory<5> traj;
vec_Vec2f _obs;
Eigen::VectorXd _odom;

unsigned int offset = 1000000;
ros::Publisher polyPub, polyPub2, polyPub3, polyPub4, 
trajPub, wptPub, velPub, trajRefPub, meshPub, edgePub,
paddedScanPub, raytracePub;

const double JACKAL_MAX_VEL = 1.5;
costmap_2d::Costmap2DROS* costmap;

// following bresenham / raycast method taken with <3 from https://docs.ros.org/en/api/costmap_2d/html/costmap__2d_8h_source.html
void bresenham(costmap_2d::Costmap2D* _map, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
        int offset_b, unsigned int offset, unsigned int max_range, unsigned int& term){

    unsigned int end = std::min(max_range, abs_da);
    unsigned int mx, my;
    for (unsigned int i = 0; i < end; ++i) {
        offset += offset_a;
        error_b += abs_db;
        
        _map->indexToCells(offset, mx, my);
        if (_map->getCost(mx, my) != costmap_2d::FREE_SPACE)
            break;

        if ((unsigned int)error_b >= abs_da) {
            offset += offset_b;
            error_b -= abs_da;
        }

        _map->indexToCells(offset, mx, my);
        if (_map->getCost(mx, my) != costmap_2d::FREE_SPACE)
            break;
    }
    
    term = offset;
}

void raycast(costmap_2d::Costmap2D* _map, unsigned int sx, unsigned int sy, 
        unsigned int ex, unsigned int ey, double &x, double &y,
        unsigned int max_range = 1e6){
    
    unsigned int size_x = _map->getSizeInCellsX();

    int dx = ex - sx;
    int dy = ey - sy;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    
    int offset_dx = dx > 0 ? 1 : -1;
    int offset_dy = (dy > 0 ? 1 : -1) * _map->getSizeInCellsX();

    unsigned int offset = sy * size_x + sx;

    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_range / dist);

    unsigned int term; 
    if (abs_dx >= abs_dy){
        int error_y = abs_dx / 2;
        bresenham(_map, abs_dx, abs_dy, error_y, offset_dx, offset_dy, 
                offset, (unsigned int)(scale * abs_dx), term);
    } else{
        int error_x = abs_dy / 2;
        bresenham(_map, abs_dy, abs_dx, error_x, offset_dy, offset_dx,
                offset, (unsigned int)(scale * abs_dy), term);
    }

    // convert costmap index to world coordinates
    unsigned int mx, my;
    _map->indexToCells(term, mx, my);
    _map->mapToWorld(mx, my, x, y);

    
}

vec_Vec2f getPaddedScan(double seedX, double seedY){

    ROS_INFO("seedX: %.2f\tseedY: %.2f", seedX, seedY);
    static int count = 0;

    visualization_msgs::Marker msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.id = count;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.z = 0;
    msg.scale.x = .1;
    msg.scale.y = .1;
    msg.scale.z = .1;
    msg.color.r = count > 0 ? 1 : 0;
    msg.color.g = count > 1 ? 1 : 0;
    msg.color.b = count > 0 ? 0 : 1;
    msg.color.a = 1;
    count += 1;
    ROS_INFO("RGB: (%.2f, %.2f, %.2f)", msg.color.r, msg.color.g, msg.color.b);

    offset = 1000000;
    costmap_2d::Costmap2D* _map = costmap->getCostmap();

    double x, y;
    unsigned int sx, sy, ex, ey;

    vec_Vec2f paddedObs;

    if (!_map->worldToMap(seedX, seedY, sx, sy)){
        ROS_ERROR("FATAL: Odometry reading is NOT inside costmap!");
        // exit(-1);
        return paddedObs;
    }

    visualization_msgs::Marker lineMsg;
    lineMsg.header.stamp = ros::Time::now();
    lineMsg.header.frame_id = "odom";
    lineMsg.type = visualization_msgs::Marker::LINE_LIST;
    lineMsg.action = visualization_msgs::Marker::ADD;
    lineMsg.pose.position.z = 0;
    lineMsg.id = 420;
    lineMsg.scale.x = .1;
    lineMsg.scale.y = .1;
    lineMsg.scale.z = .1;
    lineMsg.color.r = 0;
    lineMsg.color.g = 1;
    lineMsg.color.b = 0;
    lineMsg.color.a = 1;
    for(Vec2f ob : _obs){
        if (!_map->worldToMap(ob[0], ob[1], ex, ey))
            continue;

        raycast(_map, sx,sy,ex,ey,x,y);
        paddedObs.push_back(Vec2f(x,y));

        // geometry_msgs::Point p;
        // p.x = x;
        // p.y = y;
        // p.z = 0;
        // msg.points.push_back(p);

        // geometry_msgs::Point p1; 
        // p1.x = seedX;
        // p1.y = seedY;
        // p1.z = 0;

        // geometry_msgs::Point p2;
        // p2.x = x;
        // p2.y = y;
        // p2.z = 0;

        // lineMsg.points.push_back(p1);
        // lineMsg.points.push_back(p2);
    }

    // raytracePub.publish(lineMsg);
    // paddedScanPub.publish(msg);
    return paddedObs;
}

void odomcb(const nav_msgs::Odometry::ConstPtr& msg){

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

    _is_init = true;
    _is_traj_generated = false;
}

void lasercb(const sensor_msgs::LaserScan::ConstPtr& msg){
    
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

Polyhedron<2> genPoly(double x, double y){
    SeedDecomp2D decomp(Vec2f(x, y));
    // decomp.set_obs(_obs);
    decomp.set_obs(getPaddedScan(x, y));
    decomp.set_local_bbox(Vec2f(2,2));
    decomp.dilate(.1);

    auto poly = decomp.get_polyhedron();
 
    return poly;
}

geometry_msgs::PolygonStamped poly2msg(const Polyhedron<2>& poly){
    auto vertices = cal_vertices(poly);

    geometry_msgs::PolygonStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    for (size_t i = 0; i < vertices.size(); i++) {
        geometry_msgs::Point32 point;
        point.x = vertices[i](0);
        point.y = vertices[i](1);
        point.z = 0;
        msg.polygon.points.push_back(point);
    }

    return msg;
}

Eigen::MatrixX4d getHyperPlanes(const Polyhedron<2>& poly, const Eigen::Vector2d& seed){
    
    vec_E<Hyperplane<2> > planes = poly.vs_;

    auto vertices = cal_vertices(poly);
    vertices.push_back(vertices[0]);

    Eigen::MatrixX4d hPoly(3*(vertices.size()-1), 4);

    for(int i = 0; i < vertices.size()-1; i++){

        // hyperplane from vertex to vertex
        Vecf<2> v1 = vertices[i];
        Vecf<2> v2 = vertices[i+1];
        Eigen::Vector4d hypEq;
        if (fabs(v1(0)-v2(0)) < 1e-6)
            hypEq << 1,0,0,-v1(0);
        else{
            double m = (v2(1)-v1(1))/(v2(0)-v1(0));
            hypEq << -m,1,0,-v1(1)+m*v1(0);
        }
        
        if (hypEq(0)*seed(0)+hypEq(1)*seed(1)+hypEq(2)*.5+hypEq(3) > 0)
            hypEq *= -1;

        Eigen::Vector4d hypEqZ0(0,0,1,-.5);

        if (hypEqZ0(0)*seed(0)+hypEqZ0(1)*seed(1)+hypEqZ0(2)*.5+ hypEqZ0(3) > 0)
            hypEqZ0 *= -1;

        Eigen::Vector4d hypEqZ1(0,0,-1,-.5);

        if (hypEqZ1(0)*seed(0)+hypEqZ1(1)*seed(1)+hypEqZ1(2)*.5+ hypEqZ1(3) > 0)
            hypEqZ1 *= -1;

        int j = 3*i;
        hPoly.row(j) = hypEq;
        hPoly.row(j+1) = hypEqZ0;
        hPoly.row(j+2) = hypEqZ1;
        // std::cout << j << ":\t" << hypEq.transpose() << std::endl;

        // hyperplane that goes through z = 1
        // Eigen::Vector4d hypEqZ1 = hypEq;
        // hypEqZ1(2) = 1;

        // if (hypEqZ1(0)*seed(0)+hypEqZ1(1)*seed(1)+hypEqZ1(2)*.5+hypEqZ1(3) > 0)
        //     hypEqZ1 *= -1;

        // hPoly.row(j+1) = hypEqZ1;
        // // std::cout << j+1 << ":\t" << hypEqZ1.transpose() << std::endl;

        // // hyperplane from <px,py,0> to <px,py,1>
        
        // // vector from <px,py,0> to <px,py,1>
        // Eigen::Vector3d vecA(0,0,1);
        // // vector from <px,py,0> to v2
        // Eigen::Vector3d vecB(v2[0]-v1[0], v2[1]-v1[1], 0);
        // // normal vector of hyperplane
        // Eigen::Vector3d vecN = vecA.cross(vecB);
        // Eigen::Vector4d hypEq2(vecN(0), vecN(1), vecN(2), -vecN(0)*v1[0]-vecN(1)*v1[1]);
        

        // if (hypEq2(0)*seed(0)+ hypEq2(1)*seed(1) + hypEq2(2)*.5 + hypEq2(3) > 0)
        //     hypEq2 *= -1;

        // hPoly.row(j) = hypEq2;
        // ROS_INFO("setupPlot(%.2f, %.2f, 0, %.2f, ax)",hypEq(0),hypEq(1),hypEq(3));
        // std::cout << j+2 << ":\t" << hypEq2.transpose() << std::endl;

    }

    return hPoly;
}

template <int D>
void visualizeTraj(const Trajectory<D> &traj){

    if (traj.getPieceNum() > 0){
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "odom";

        visualization_msgs::Marker wptMsg;
        wptMsg.header.stamp = ros::Time::now();
        wptMsg.header.frame_id = "odom";
        wptMsg.id = 0;
        wptMsg.type = visualization_msgs::Marker::SPHERE_LIST;
        wptMsg.ns = "waypoints";
        wptMsg.color.r = 1.00;
        wptMsg.color.g = 0.00;
        wptMsg.color.b = 0.00;
        wptMsg.color.a = 1.00;
        wptMsg.scale.x = 0.35;
        wptMsg.scale.y = 0.35;
        wptMsg.scale.z = 0.35;

        Eigen::MatrixXd wps = traj.getPositions();
        for (int i = 0; i < wps.cols(); i++){
            
            geometry_msgs::Point point;
            point.x = wps.col(i)(0);
            point.y = wps.col(i)(1);
            point.z = wps.col(i)(2);
            wptMsg.points.push_back(point);

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom";
            pose.pose.position.x = wps.col(i)(0);
            pose.pose.position.y = wps.col(i)(1);
            pose.pose.position.z = wps.col(i)(2);

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            msg.poses.push_back(pose);
        }

        wptPub.publish(wptMsg);
        trajPub.publish(msg);
        ROS_INFO("wptMsg size: %lu", wptMsg.points.size());
    }

}

trajectory_msgs::JointTrajectory convertTrajToMsg(){

    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    double factor = traj.getMaxVelRate() / JACKAL_MAX_VEL;
    double t = 0.;
    while (t < traj.getTotalDuration()){
        Eigen::Vector3d pos = traj.getPos(t);
        Eigen::Vector3d vel = traj.getVel(t);
        Eigen::Vector3d acc = traj.getAcc(t);

        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions.push_back(pos(0));
        pt.positions.push_back(pos(1));

        pt.velocities.push_back(vel(0)/factor);
        pt.velocities.push_back(vel(1)/factor);

        pt.accelerations.push_back(acc(0)/factor);
        pt.accelerations.push_back(acc(1)/factor);

        pt.time_from_start = ros::Duration(t*factor);

        msg.points.push_back(pt);
        
        // ROS_INFO("%.2f/%.2f:\t(%.2f, %.2f)",t*factor,traj.getTotalDuration()*factor, pos(0), pos(1));
        t += .1/factor;
    }

    return msg;
}

// Visualize some polytopes in H-representation
inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
{

    // Due to the fact that H-representation cannot be directly visualized
    // We first conduct vertex enumeration of them, then apply quickhull
    // to obtain triangle meshs of polyhedra
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    for (size_t id = 0; id < hPolys.size(); id++)
    {
        oldTris = mesh;
        Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
        geo_utils::enumerateVs(hPolys[id], vPoly);

        quickhull::QuickHull<double> tinyQH;
        const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
        const auto &idxBuffer = polyHull.getIndexBuffer();
        int hNum = idxBuffer.size() / 3;

        curTris.resize(3, hNum * 3);
        for (int i = 0; i < hNum * 3; i++)
        {
            curTris.col(i) = vPoly.col(idxBuffer[i]);
        }
        mesh.resize(3, oldTris.cols() + curTris.cols());
        mesh.leftCols(oldTris.cols()) = oldTris;
        mesh.rightCols(curTris.cols()) = curTris;
    }

    // RVIZ support tris for visualization
    visualization_msgs::Marker meshMarker, edgeMarker;

    meshMarker.id = 0;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "odom";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = 0.00;
    meshMarker.color.g = 0.00;
    meshMarker.color.b = 1.00;
    meshMarker.color.a = 0.15;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 0.00;
    edgeMarker.color.g = 1.00;
    edgeMarker.color.b = 1.00;
    edgeMarker.color.a = 1.00;
    edgeMarker.scale.x = 0.02;

    geometry_msgs::Point point;

    int ptnum = mesh.cols();

    for (int i = 0; i < ptnum; i++)
    {
        point.x = mesh(0, i);
        point.y = mesh(1, i);
        point.z = mesh(2, i);
        meshMarker.points.push_back(point);
    }

    for (int i = 0; i < ptnum / 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            point.x = mesh(0, 3 * i + j);
            point.y = mesh(1, 3 * i + j);
            point.z = mesh(2, 3 * i + j);
            edgeMarker.points.push_back(point);
            point.x = mesh(0, 3 * i + (j + 1) % 3);
            point.y = mesh(1, 3 * i + (j + 1) % 3);
            point.z = mesh(2, 3 * i + (j + 1) % 3);
            edgeMarker.points.push_back(point);
        }
    }

    meshPub.publish(meshMarker);
    edgePub.publish(edgeMarker);

    return;
}

void functor(unsigned int val){
    costmap_2d::Costmap2D* _map = costmap->getCostmap();
    if (_map->getCharMap()[val] == costmap_2d::LETHAL_OBSTACLE)
        offset = val;
    
}



void controlLoop(const ros::TimerEvent&){

    static ros::Time start;
    static int tmp = 0;

    if (!_is_init)
        return;


    ros::Time a = ros::Time::now();
    costmap->updateMap();
    traj.clear();
    Eigen::Vector2d seed1(_odom(0),_odom(1));
    // Eigen::Vector2d seed2(3.5,-1);
    Eigen::Vector2d seed2(4.25,-1);
    Eigen::Vector2d seed3(5.25,-.45);
    Eigen::Vector2d seed4(2.75,-.6);

    auto poly1 = genPoly(seed1(0), seed1(1));
    auto poly2 = genPoly(seed2(0), seed2(1));
    auto poly3 = genPoly(seed3(0), seed3(1));
    auto poly4 = genPoly(seed4(0), seed4(1));

    // geometry_msgs::PolygonStamped msg = poly2msg(poly1);
    // geometry_msgs::PolygonStamped msg2 = poly2msg(poly2);
    // geometry_msgs::PolygonStamped msg3 = poly2msg(poly3);
    // geometry_msgs::PolygonStamped msg4 = poly2msg(poly4);

    // polyPub.publish(msg);
    // polyPub2.publish(msg2);
    // polyPub3.publish(msg3);
    // polyPub4.publish(msg4);

    std::vector<Eigen::MatrixX4d> hPolys;

    Eigen::MatrixX4d hPoly1 = getHyperPlanes(poly1, seed1);
    Eigen::MatrixX4d hPoly2 = getHyperPlanes(poly2, seed2);
    Eigen::MatrixX4d hPoly3 = getHyperPlanes(poly3, seed3);
    Eigen::MatrixX4d hPoly4 = getHyperPlanes(poly4, seed4);
    
    hPolys.push_back(hPoly1);
    hPolys.push_back(hPoly4);
    hPolys.push_back(hPoly2);
    hPolys.push_back(hPoly3);

    visualizePolytope(hPolys);

    // initial and final states
    Eigen::Matrix3d initialPVA;
    initialPVA << Eigen::Vector3d(_odom(0),_odom(1),0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    Eigen::Matrix3d finalPVA;
    // finalPVA << Eigen::Vector3d(3.5,-1,0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    finalPVA << Eigen::Vector3d(5.25,-.45,0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    // finalPVA << Eigen::Vector3d(4.3,-1,0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    Eigen::VectorXd magnitudeBounds(5);
    Eigen::VectorXd penaltyWeights(5);
    Eigen::VectorXd physicalParams(6);
    magnitudeBounds(0) = 1.5;   //v_max
    magnitudeBounds(1) = .8;   //omg_max
    magnitudeBounds(2) = .3;    //theta_max
    magnitudeBounds(3) = -1;     //thrust_min
    magnitudeBounds(4) = 0;    //thrust_max
    penaltyWeights(0) = 1e4;    //pos_weight
    penaltyWeights(1) = 1e4;    //vel_weight
    penaltyWeights(2) = 1e4;    //omg_weight
    penaltyWeights(3) = 1e4;    //theta_weight
    penaltyWeights(4) = 1e5;    //thrust_weight
    physicalParams(0) = .61;    // mass
    physicalParams(1) = 9.81;   // gravity
    physicalParams(2) = 0;      // drag
    physicalParams(3) = 0;      // drag
    physicalParams(4) = 0;      // drag
    physicalParams(5) = .0001;  // speed smooth factor

    gcopter::GCOPTER_PolytopeSFC gcopter;

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
        ROS_INFO("uh oh");
        return;
    }

    if (std::isinf(gcopter.optimize(traj, 1e-5))){
        ROS_INFO("solver could not find trajectory");
        return;
    }

    visualizeTraj(traj);
    ROS_INFO("trajectory max velocity is: %.2f", traj.getMaxVelRate());
    
    if (tmp == 0){
        tmp += 1;
        trajRefPub.publish(convertTrajToMsg());
    }

    double totalT = (ros::Time::now() - a).toSec();
    std::cout << "total time is " << totalT << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "minco_test");
    ros::NodeHandle nh;
    
	ros::Timer timer = nh.createTimer(ros::Duration(.1), controlLoop);
    ros::Subscriber laserSub = nh.subscribe("/front/scan", 1, lasercb);
    ros::Subscriber odomSub = nh.subscribe("/odometry/filtered", 1, odomcb);
    polyPub = 
        nh.advertise<geometry_msgs::PolygonStamped>("/convex_free", 0);
    polyPub2 = 
        nh.advertise<geometry_msgs::PolygonStamped>("/convex_free2", 0);
    polyPub3 = 
        nh.advertise<geometry_msgs::PolygonStamped>("/convex_free3", 0);
    polyPub4 = 
        nh.advertise<geometry_msgs::PolygonStamped>("/convex_free4", 0);
    trajPub = 
        nh.advertise<nav_msgs::Path>("/MINCO_path", 0);
    wptPub = 
        nh.advertise<visualization_msgs::Marker>("/MINCO_wpts", 0);
    velPub =
        nh.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
    trajRefPub =
        nh.advertise<trajectory_msgs::JointTrajectory>("/reference_trajectory", 0);

    meshPub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    edgePub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);

    paddedScanPub = 
        nh.advertise<visualization_msgs::Marker>("/front/padded_scan", 0);

    raytracePub = 
        nh.advertise<visualization_msgs::Marker>("/front/raytrace", 0);

    _is_init = false;

    tf::TransformListener tfListener(ros::Duration(10));    
    costmap = new costmap_2d::Costmap2DROS("costmap", tfListener);
    costmap->start();
    ros::spin();
    return 0;
}
