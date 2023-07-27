#ifndef MOTION_PRIMITIVE_H
#define MOTION_PRIMITIVE_H

#include <cmath>
#include <vector>
#include <random>
#include <tf/tf.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PolygonStamped.h>



// const double duration = 2;  // Duration of each motion primitive execution (in seconds)
// const double dt = 0.1;  // Time step for generating motion trajectories

// Define motion primitive tree node class
class MotionPrimitiveNode {
public:
    std::map<std::string, double> motion_primitive;
    MotionPrimitiveNode* parent;
    std::vector<MotionPrimitiveNode*> children;
    std::vector<std::vector<double>> states;  // List to store robot states for each time step
    int depth;
    double duration, dt;

    MotionPrimitiveNode(const std::map<std::string, double>& motion_primitive, MotionPrimitiveNode* parent = nullptr, double duration=2.0, double dt=.1)
        : motion_primitive(motion_primitive),
          parent(parent),
          depth(0),
          duration(duration),
          dt(dt),
          step_count(0) {}

    void addChild(MotionPrimitiveNode* child) {
        children.push_back(child);
    }

    void execute(ros::Publisher& cmd_vel_pub) const{
        int num_steps = static_cast<int>(duration / dt);

        // Create a Twist message to send the velocity commands
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = motion_primitive.at("speed");
        twist_msg.angular.z = motion_primitive.at("angular_velocity");

        // Execute the motion primitive for the intended duration
        for (int i = 0; i < num_steps; ++i) {
            cmd_vel_pub.publish(twist_msg);
            ros::Duration(dt).sleep();
        }
    }

    bool isDone(double currTime) const{
        return duration < currTime;
        // int num_steps = static_cast<int>(duration / dt);
        // std::cout << "step count is " << step_count << " / " << num_steps << std::endl;
        // if (step_count < num_steps){
        //     step_count += 1;
        //     return true;
        // }

        // std::cout << "?" << std::endl;
        // step_count = 0;

        // return false;
    }

    std::string toString() const {
        std::string str = "Depth: " + std::to_string(depth) + ", Motion Primitive: ";
        
        for (const auto& entry : motion_primitive) {
            str += entry.first + ": " + std::to_string(entry.second) + ", ";
        }
        
        // Remove the trailing comma and space
        if (!motion_primitive.empty()) {
            str = str.substr(0, str.length() - 2);
        }
        
        return str;
    }

    friend std::ostream& operator<<(std::ostream& os, const MotionPrimitiveNode& node) {
        os << "Depth: " << node.depth << ", Motion Primitive: ";
        
        for (const std::pair<std::string, double>& entry : node.motion_primitive) {
            os << entry.first << ": " << entry.second << ", ";
        }
        
        // Remove the trailing comma and space
        if (!node.motion_primitive.empty()) {
            os.seekp(-2, std::ios_base::end);  // Move back 2 positions from the end
        }
        
        return os;
    }

private:
    int step_count;
};

// void visualize_motion_primitive(ros::Publisher& marker_pub, const MotionPrimitiveNode* motion_primitive_node, int id);
// std_msgs::ColorRGBA interpolate_color(const std_msgs::ColorRGBA& start_color, const std_msgs::ColorRGBA& end_color, double t);

class MotionPrimitiveTree {
    public:
        MotionPrimitiveTree(const costmap_2d::Costmap2DROS* costmap_ros, double duration, double dt)
        : costmap_ros(costmap_ros), duration(duration), dt(dt), root(nullptr) {}


        void update_costmap(const costmap_2d::Costmap2DROS* costmap){
            costmap_ros = costmap;
        }

        void generate_motion_primitive_tree(int max_depth, const std::vector<double>& initial_state) {
            root = new MotionPrimitiveNode({},nullptr, duration, dt);

            std::vector<std::pair<MotionPrimitiveNode*, std::vector<double>>> stack;
            stack.push_back(std::make_pair(root, initial_state));

            while (!stack.empty()) {
                MotionPrimitiveNode* node = stack.back().first;
                std::vector<double> current_state = stack.back().second;
                stack.pop_back();

                if (node == nullptr || node->depth == max_depth) {
                    continue;
                }

                for (const auto& primitive : motion_primitives) {
                    MotionPrimitiveNode* child_node = new MotionPrimitiveNode(primitive, node, duration, dt);
                    node->addChild(child_node);

                    // Generate the motion trajectory and store the robot's states
                    std::vector<std::vector<double>> motion_trajectory = generate_motion_trajectory(primitive, current_state);
                    child_node->states = motion_trajectory;
                    child_node->depth = node->depth + 1;
                    child_node->duration = duration;

                    stack.push_back(std::make_pair(child_node, motion_trajectory.back()));
                }
            }

            return;
        }

        // Evaluate a backward trajectory and return a score
        double evaluate_backward_trajectory(const std::vector<const MotionPrimitiveNode*>& trajectory) {
            // Implement your evaluation criteria here
            // You can compute a score based on proximity to goal, safety, efficiency, etc.
            // Return the score for the trajectory

            // at beginning of bfs this is possible
            if (trajectory.size() == 0)
                return -1.0;

            // if root is passed into trajectory this is possible
            if (trajectory.back()->states.size() == 0)
                return -1.0;

            const MotionPrimitiveNode* last_node = trajectory.back();
            const std::vector<double>& last_state = last_node->states.back();
            double last_x = last_state[0];
            double last_y = last_state[1];

            // std::cout << "getting costmap info" << std::endl;
            const costmap_2d::Costmap2D& costmap = *costmap_ros->getCostmap();

            // std::cout << "World to map" << std::endl;
            // Convert (x, y) coordinates to map indices
            unsigned int map_x, map_y;
            costmap.worldToMap(last_x, last_y, map_x, map_y);

            // std::cout << "getting cost" << std::endl;

            // Check if the map cell at the last position is free
            unsigned char cost = costmap.getCost(map_x, map_y);
            return cost == costmap_2d::FREE_SPACE;
        }

        std::vector<const MotionPrimitiveNode*> find_best_trajectory() {
            std::vector<const MotionPrimitiveNode*> current_trajectory;
            std::vector<const MotionPrimitiveNode*> best_trajectory;

            dfs_traversal(*root, current_trajectory, best_trajectory);
            best_trajectory.erase(best_trajectory.begin());

            return best_trajectory;
        }

        // Function to generate a random backward motion trajectory
        std::vector<MotionPrimitiveNode*> generate_random_backward_motion() {
            std::vector<MotionPrimitiveNode*> motion_trajectory_nodes;
            motion_trajectory_nodes.push_back(root->children[rand() % motion_primitives.size()]);

            for (int i = 0; i < 4; ++i) {
                motion_trajectory_nodes.push_back(motion_trajectory_nodes.back()->children[rand() % motion_primitives.size()]);
            }

            return motion_trajectory_nodes;
        }

        bool checkCollision(const MotionPrimitiveNode& node){
            const costmap_2d::Costmap2D& costmap = *costmap_ros->getCostmap();

            std::vector<geometry_msgs::Point> raw_footprint = costmap_ros->getUnpaddedRobotFootprint();
            
            for (const std::vector<double>& state : node.states){

                // Transform the raw footprint to match the state
                std::vector<geometry_msgs::Point> oriented_footprint;
                tf::Transform pose_tf;
                // std::cerr << "setting translation" << std::endl;
                tf::Vector3 translation(state[0], state[1], 0.0);
                tf::Quaternion rotation;
                // std::cerr << "setting rotation" << std::endl;
                rotation.setRPY(0.0, 0.0, state[2]);
                // std::cerr << "applying transform" << std::endl;
                pose_tf.setOrigin(translation);
                pose_tf.setRotation(rotation);

                // std::cerr << "transforming footprint" << std::endl;
                for(const geometry_msgs::Point& point : raw_footprint){
                    geometry_msgs::Point transformed_point;
                    tf::Vector3 point_tf(point.x, point.y, point.z);
                    tf::Vector3 transformed_point_tf = pose_tf * point_tf;
                    transformed_point.x = transformed_point_tf.getX();
                    transformed_point.y = transformed_point_tf.getY();
                    transformed_point.z = transformed_point_tf.getZ();
                    oriented_footprint.push_back(transformed_point);
                }

                // std::cerr << "footprint collision" << std::endl;
                // Check if any of the footprint cells are in the inflated obstacle space
                for (const geometry_msgs::Point& point : oriented_footprint)
                {
                    unsigned int cell_x, cell_y;
                    if (costmap.worldToMap(point.x, point.y, cell_x, cell_y)){

                        unsigned char cost = costmap.getCost(cell_x, cell_y);

                        if (cost == costmap_2d::LETHAL_OBSTACLE)// || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                        {
                            std::cout << "point " << point.x << ", " << point.y << " collides" << std::endl;
                            return true; // Collision detected
                        }
                    }
                }   // end footprint iteration

            } // end state iteration
            
            return false; // No collision
        }

        const MotionPrimitiveNode* getRoot(){
            return root;
        }

    private:
        // Define motion primitives (backward motions)
        std::vector<std::map<std::string, double>> motion_primitives = {
            {{"speed", -0.4}, {"angular_velocity", 0.0}},     // Straight backward
            {{"speed", -0.2}, {"angular_velocity", -0.3}},    // Backward and slight left turn
            {{"speed", -0.2}, {"angular_velocity", 0.3}},     // Backward and slight right turn
            {{"speed", -0.3}, {"angular_velocity", -0.15}},   // Backward and moderate left turn
            {{"speed", -0.3}, {"angular_velocity", 0.15}}     // Backward and moderate right turn
        };

        const costmap_2d::Costmap2DROS* costmap_ros;
        double duration, dt;
        MotionPrimitiveNode* root;

        std::vector<std::vector<double>> generate_motion_trajectory(const std::map<std::string, 
                                                                    double>& motion_primitive, 
                                                                    const std::vector<double>& initial_state
        ) {
            double speed = motion_primitive.at("speed");
            double angular_velocity = motion_primitive.at("angular_velocity");

            std::vector<std::vector<double>> trajectory;
            trajectory.push_back(initial_state);

            for (double t = dt; t <= duration; t += dt) {
                // Retrieve the previous state
                std::vector<double> prev_state = trajectory.back();

                // Compute the new state based on the motion primitive
                double new_x = prev_state[0] + speed * std::cos(prev_state[2]) * dt;
                double new_y = prev_state[1] + speed * std::sin(prev_state[2]) * dt;
                double new_theta = prev_state[2] + angular_velocity * dt;

                // Append the new state to the trajectory
                trajectory.push_back({new_x, new_y, new_theta});
            }

            return trajectory;
        }

        // Perform DFS traversal to find the best backward trajectory
        bool dfs_traversal(const MotionPrimitiveNode& node, 
                        std::vector<const MotionPrimitiveNode*>& current_trajectory, 
                        std::vector<const MotionPrimitiveNode*>& best_trajectory
        ){

            // don't pursue this branch at all if the primitive collides with obstacles.
            // std::cout << "checking node collision" << std::endl;
            if (checkCollision(node)){
                // std::cout << "collides!" << std::endl;
                return false;
            }

            current_trajectory.push_back(&node);

            // std::cout << "evaluating trajectory" << std::endl;
            // Check if the current trajectory is better than the best trajectory found so far
            if (evaluate_backward_trajectory(current_trajectory) > evaluate_backward_trajectory(best_trajectory)) {
                best_trajectory = current_trajectory;
                // std::cout << "fount a new best trajectory!" << std::endl;
            }

            // Perform DFS on the child nodes
            bool collision_free_traj_found = false;

            // std::cout << "Looking at children" << std::endl;
            for (MotionPrimitiveNode* child : node.children) {
                // std::cout << "checking " << child->toString() << " now" << std::endl;
                if(dfs_traversal(*child, current_trajectory, best_trajectory)){
                    collision_free_traj_found = true;
                }
            }

            // Remove the current node from the current trajectory (backtrack)
            current_trajectory.pop_back();
            
            return collision_free_traj_found;
        }

};


// std_msgs::ColorRGBA interpolate_color(const std_msgs::ColorRGBA& start_color, const std_msgs::ColorRGBA& end_color, double t) {
//     std_msgs::ColorRGBA color;
//     color.r = start_color.r + (end_color.r - start_color.r) * t;
//     color.g = start_color.g + (end_color.g - start_color.g) * t;
//     color.b = start_color.b + (end_color.b - start_color.b) * t;
//     color.a = start_color.a + (end_color.a - start_color.a) * t;
//     return color;
// }

// void visualize_motion_primitive(ros::Publisher& marker_pub, const MotionPrimitiveNode* motion_primitive_node, int id) {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "odom";  // Replace with the appropriate frame ID
//     marker.type = visualization_msgs::Marker::LINE_STRIP;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.scale.x = 0.1;  // Line width
//     marker.id = id;

//     std_msgs::ColorRGBA start_color;
//     start_color.r = 0.0;
//     start_color.g = 0.0;
//     start_color.b = 1.0;
//     start_color.a = 1.0;

//     std_msgs::ColorRGBA end_color;
//     end_color.r = 1.0;
//     end_color.g = 0.0;
//     end_color.b = 0.0;
//     end_color.a = 1.0;

//     int num_states = motion_primitive_node->states.size();
//     for (int i = 0; i < num_states; ++i) {
//         geometry_msgs::Point point;
//         point.x = motion_primitive_node->states[i][0];
//         point.y = motion_primitive_node->states[i][1];
//         point.z = 0.0;
//         marker.points.push_back(point);

//         double t = static_cast<double>(i) / (num_states - 1);  // Normalized time step between 0.0 and 1.0
//         std_msgs::ColorRGBA color = interpolate_color(start_color, end_color, t);
//         marker.colors.push_back(color);
//     }

//     marker_pub.publish(marker);
// }

#endif
