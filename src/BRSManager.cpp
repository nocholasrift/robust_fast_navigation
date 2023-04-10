#include <ros/ros.h>
#include <iostream>

#include <std_msgs/Int32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float64MultiArray.h>

bool init = false;
int angle = 0;
ros::Publisher gridPub;
std_msgs::Float64MultiArray brsMsg;

void brscb(const std_msgs::Float64MultiArray::ConstPtr& msg){

    // std::cout << "Size is: " << msg->layout.dim.size() << std::endl;
    // float strideX = msg->layout.dim[0].stride;
    // float sizeX = msg->layout.dim[0].size;
    // float strideY = msg->layout.dim[1].stride;
    // float sizeY = msg->layout.dim[1].size;
    // float strideTheta = msg->layout.dim[2].stride;
    // float sizeTheta = msg->layout.dim[2].size;

    brsMsg = *msg;
    init = true;
}

void anglecb(const std_msgs::Int32::ConstPtr& msg){
    angle = msg->data;
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
        }
    }

    gridPub.publish(mapMsg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "BRS_manager");
    ros::NodeHandle nh;

    ros::Subscriber brsSub = nh.subscribe("/brsData", 1, &brscb);
    ros::Subscriber angleSub = nh.subscribe("/brsAngle", 1, &anglecb);
    gridPub = nh.advertise<nav_msgs::OccupancyGrid>("brsGrid", 0);

    ros::Timer pubTimer = nh.createTimer(ros::Duration(1), &mapPublisher);

    ros::spin();
    return 0;
}
