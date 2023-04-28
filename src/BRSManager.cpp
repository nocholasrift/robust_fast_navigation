#include <signal.h>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <robust_fast_navigation/JPS.h>
#include <robust_fast_navigation/corridor.h>

bool init = false;
int angle = -1;
ros::Publisher gridPub, boundaryPub, jpsPub, meshPub, edgePub;
std_msgs::Float64MultiArray brsMsg;


void brscb(const std_msgs::Float64MultiArray::ConstPtr& msg){

    // std::cout << "Size is: " << msg->layout.dim.size() << std::endl;
    // float strideX = msg->layout.dim[0].stride;
    // float sizeX = msg->layout.dim[0].size;
    // float strideY = msg->layout.dim[1].stride;
    // float sizeY = msg->layout.dim[1].size;
    // float strideTheta = msg->layout.dim[2].stride;
    // float sizeTheta = msg->layout.dim[2].size;

    ROS_INFO("received brs");
    brsMsg = *msg;
    init = true;
}

void anglecb(const std_msgs::Int32::ConstPtr& msg){
    angle = msg->data;
}

bool hasFreeNeighbor(const nav_msgs::OccupancyGrid& msg, int x, int y){

    int height = msg.info.height;
    int width = msg.info.width;

    for(int i = -1; i <= 1; i++){
        if (x + i >= width)
            break;
        if (x + i < 0)
            continue;

        for(int j = -1; j <= 1; j++){
            if (y + j >= height)
                break;
            if (y + j < 0)
                continue;

            if (msg.data[(y+j)*width + (x+i)] == 0)
                return true;
        }
    }

    return false;
}

void mapPublisher(const ros::TimerEvent&){

    if (!init)
        return; 

    float sz = 82;

    float strideX = sz*sz;
    float sizeX = sz;
    float strideY = sz;
    float sizeY = sz;
    float strideTheta = 1;
    float sizeTheta = sz;

    nav_msgs::OccupancyGrid mapMsg;
    mapMsg.header.stamp = ros::Time::now();
    mapMsg.header.frame_id = "map";
    mapMsg.info.resolution=10./sizeX;
    mapMsg.info.width=sizeX;
    mapMsg.info.height=sizeY;
    mapMsg.info.origin.position.x = -5;
    mapMsg.info.origin.position.y = -5;
    mapMsg.info.origin.orientation.w = 1;

    mapMsg.data.resize(sizeX*sizeY);

    std::vector<Eigen::Vector2d> boundary;
    unsigned char map_unsigned[mapMsg.data.size()];

    int k = 0;
    for(int i = 0; i < sizeY; i++){
        for(int j = 0; j < sizeX; j++){

            bool isInBRS = false;
            if (angle >= 0){
                if (brsMsg.data[angle+strideX*j+strideY*i] <= 0)
                    isInBRS = true;
            }
            else{
                for(int theta = 0; theta < sizeTheta; theta++){
                    if (brsMsg.data[theta+strideX*j+strideY*i] <= 0){
                        isInBRS = true;
                        break;
                    }
                }
            }

            if (isInBRS)
                mapMsg.data[k++] = 0;
            else
                mapMsg.data[k++] = 100;

            map_unsigned[k-1] = mapMsg.data[k-1];

        }
    }
    gridPub.publish(mapMsg);

    visualization_msgs::Marker marker;
    marker.header.frame_id="map";
    marker.header.stamp=ros::Time::now();
    marker.ns = "brs_boundary";
    marker.id = 6509;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = .1;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;

    for(int i = 0; i < sizeX; i++){
        for(int j = 0; j < sizeY; j++){
            if (mapMsg.data[j*sizeX + i] == 100 && hasFreeNeighbor(mapMsg,i,j)){
                double x = mapMsg.info.origin.position.x + (i+.5)*mapMsg.info.resolution;
                double y = mapMsg.info.origin.position.y + (j+.5)*mapMsg.info.resolution;
                boundary.push_back(Eigen::Vector2d(x,y));
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = 0;
                marker.points.push_back(p);
            }
        }
    }

    boundaryPub.publish(marker);

    JPSPlan jps;
    jps.set_map(map_unsigned, mapMsg.info.width, mapMsg.info.height,
                mapMsg.info.origin.position.x, mapMsg.info.origin.position.y,
                mapMsg.info.resolution);

    unsigned int sX, sY, eX, eY;
    // world 0
    jps.worldToMap(0.05,0.,sX,sY);
    jps.worldToMap(4.4,-1.,eX,eY);

    // world 294
    // jps.worldToMap(0.05,0.,sX,sY);
    // jps.worldToMap(3.4,-.01,eX,eY);

    jps.set_start(sX,sY);
    jps.set_destination(eX,eY);
    jps.set_occ_value(100);
    jps.JPS();

    std::vector<Eigen::Vector2d> jpsPath = jps.getPath(true);

    ROS_INFO("path size is %lu", jpsPath.size());
    nav_msgs::Path jpsMsg;
    jpsMsg.header.stamp = ros::Time::now();
    jpsMsg.header.frame_id = "map";

    for(Eigen::Vector2d p : jpsPath){
        geometry_msgs::PoseStamped pMsg;
        pMsg.header = jpsMsg.header;
        pMsg.pose.position.x = p(0);
        pMsg.pose.position.y = p(1);
        pMsg.pose.position.z = 0;
        pMsg.pose.orientation.w = 1;
        jpsMsg.poses.push_back(pMsg);
        ROS_INFO("(%.2f,%.2f)", p(0), p(1));
    }

    jpsPub.publish(jpsMsg);

    std::vector<Eigen::MatrixX4d> polys;
    corridor::createCorridorBRS(jpsPath, mapMsg, boundary, polys);
    corridor::visualizePolytope(polys, meshPub, edgePub);
}

void on_shutdown(int sig){
    if (!init){
        ROS_INFO("shutting down not initialized");
        ros::shutdown();
        return;
    }

    ros::shutdown();
    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "BRS_manager");
    ros::NodeHandle nh;

    ros::Subscriber brsSub = nh.subscribe("/brsData", 100, &brscb);
    ros::Subscriber angleSub = nh.subscribe("/brsAngle", 1, &anglecb);

    jpsPub = nh.advertise<nav_msgs::Path>("brsJPS", 0);
    gridPub = nh.advertise<nav_msgs::OccupancyGrid>("brsGrid", 0);
    boundaryPub = nh.advertise<visualization_msgs::Marker>("/brsBoundary", 0);

    meshPub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/meshBRS", 1000);
    edgePub = 
        nh.advertise<visualization_msgs::Marker>("/visualizer/edgeBRS", 1000);

    ros::Timer pubTimer = nh.createTimer(ros::Duration(1), &mapPublisher);

    signal(SIGINT, on_shutdown);

    ros::spin();
    return 0;
}
