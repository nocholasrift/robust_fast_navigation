#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <robust_fast_navigation/motion_primitives.h>

// Global variables
nav_msgs::Odometry odom_data;
sensor_msgs::LaserScan lidar_data;
ros::Publisher marker_pub;

// Method declarations
// void displayPrimitiveTreeStates(const MotionPrimitiveNode* root, ros::Publisher& marker_pub);
void displayMotionPrimitiveTree(const MotionPrimitiveNode* node, visualization_msgs::MarkerArray& marker_array, int depth = 0);


void odom_callback(const nav_msgs::Odometry::ConstPtr& data) {
    odom_data = *data;
    tf::Quaternion q(
	    odom_data.pose.pose.orientation.x,
	    odom_data.pose.pose.orientation.y,
	    odom_data.pose.pose.orientation.z,
	    odom_data.pose.pose.orientation.w
	);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

    std::vector<double> state = {odom_data.pose.pose.position.x,
                                 odom_data.pose.pose.position.y,
                                 yaw};

    costmap_2d::Costmap2DROS* costmap;
    MotionPrimitiveTree tree(costmap, .5, .1);

    ROS_INFO("generating tree");
    ros::Time start = ros::Time::now();
    tree.generate_motion_primitive_tree(5, state);
    ROS_INFO("%.4f", (ros::Time::now()-start).toSec());
    ROS_INFO("displaying tree");
    visualization_msgs::MarkerArray msg;
    displayMotionPrimitiveTree(tree.getRoot(), msg);
    marker_pub.publish(msg);
    ROS_INFO("exiting...");

    exit(0);
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& data) {
    lidar_data = *data;
}

void displayMotionPrimitiveTree(const MotionPrimitiveNode* node, visualization_msgs::MarkerArray& marker_array, int depth){
    // Define a unique color for each depth level
    std_msgs::ColorRGBA color;
    color.r = static_cast<float>(depth % 5) / 5.0;  // Adjust the division value to change the color gradient
    color.g = static_cast<float>((depth + 1) % 5) / 5.0;
    color.b = static_cast<float>((depth + 2) % 5) / 5.0;
    color.a = .8;

    // Create a marker for the current node
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "motion_primitives";
    marker.id = marker_array.markers.size();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.color = color;

    // Iterate over the states of the current node and add points to the marker
    for (const std::vector<double>& state : node->states) {
        geometry_msgs::Point point;
        point.x = state[0];
        point.y = state[1];
        point.z = 0.0;
        marker.points.push_back(point);
    }

    // Add the marker to the marker array
    marker_array.markers.push_back(marker);

    // Recursively process the children of the current node
    for (const MotionPrimitiveNode* child : node->children) {
        displayMotionPrimitiveTree(child, marker_array, depth + 1);
    }
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_recovery_node");
    ros::NodeHandle nh;

    // Subscribe to odometry and lidar topics
    ros::Subscriber odom_sub = nh.subscribe("/gmapping/odometry", 1, odom_callback);
    ros::Subscriber lidar_sub = nh.subscribe("/front/scan", 1, lidar_callback);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("primitive_tree_markers", 1);


    ros::spin();

    return 0;
}
