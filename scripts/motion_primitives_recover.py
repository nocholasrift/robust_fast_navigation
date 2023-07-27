#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Point

from tf.transformations import euler_from_quaternion

import sys
import numpy as np
from random import randrange
import matplotlib.pyplot as plt

# Define motion primitives (backward motions)
motion_primitives = [
    {"speed": -0.4, "angular_velocity": 0.0},  # Straight backward
    {"speed": -0.2, "angular_velocity": -0.3},  # Backward and slight left turn
    {"speed": -0.2, "angular_velocity": 0.3},  # Backward and slight right turn
    {"speed": -0.3, "angular_velocity": -0.15},  # Backward and moderate left turn
    {"speed": -0.3, "angular_velocity": 0.15},  # Backward and moderate right turn
]

# Define time parameter
duration = .8  # Duration of each motion primitive execution (in seconds)

# Global variables
odom_data = None
lidar_data = None


# Define heuristic scoring function
def score_motion_primitive(motion_primitive):
    # Implement your scoring heuristic here
    # Evaluate the motion_primitive and return a score
    return 0  # Placeholder score, replace with your scoring logic

# Define motion primitive tree node class
class MotionPrimitiveNode:
    def __init__(self, motion_primitive, parent=None):
        self.motion_primitive = motion_primitive
        self.parent = parent
        self.score = score_motion_primitive(motion_primitive)
        self.children = []
        self.states = []  # List to store robot states for each time step
        self.depth = 0
        self.duration = 1.0

    def add_child(self, child):
        self.children.append(child)

    def execute(self, cmd_vel_pub, loop_rate):

        num_steps = int(self.duration / loop_rate)

        # Calculate the time interval between each step
        time_interval = self.duration / num_steps

        # Create a Twist message to send the velocity commands
        twist_msg = Twist()
        twist_msg.linear.x = self.motion_primitive['speed']
        twist_msg.angular.z = self.motion_primitive['angular_velocity']

        # Execute the motion primitive for the intended duration
        for _ in range(num_steps):
            cmd_vel_pub.publish(twist_msg)
            rospy.sleep(time_interval)

        
def generate_motion_trajectory(motion_primitive, initial_state, dt):
    # Extract motion parameters from the motion primitive
    speed = motion_primitive["speed"]
    angular_velocity = motion_primitive["angular_velocity"]


    # Initialize the trajectory with the initial state
    trajectory = [initial_state]

    # Generate the motion trajectory
    for t in range(1, int(duration / dt) + 1):
        # Retrieve the previous state
        prev_state = trajectory[t-1]

        # Compute the new state based on the motion primitive
        # print(f"prev_state_x: {prev_state[0]}\tprev_state_y: {prev_state[1]}\tnew_theta: {prev_state[2]}")

        new_x = prev_state[0] + speed * np.cos(prev_state[2]) * dt
        new_y = prev_state[1] + speed * np.sin(prev_state[2]) * dt
        new_theta = prev_state[2] + angular_velocity * dt

        # Append the new state to the trajectory
        trajectory.append([new_x, new_y, new_theta])

    # sys.exit()
    return trajectory


def generate_motion_primitive_tree(max_depth, initial_state, dt):
    global duration
    root = MotionPrimitiveNode(None)

    stack = [(root, initial_state)]
    
    while stack:
        node, current_state = stack.pop()
        # print("node is", node,f"(depth = {init_depth-max_depth})")

        if node is None or node.depth == max_depth:
            continue

        for primitive in motion_primitives:
            child_node = MotionPrimitiveNode(primitive, node)
            node.add_child(child_node)

            # Generate the motion trajectory and store the robot's states
            motion_trajectory = generate_motion_trajectory(primitive, current_state, dt)
            node.children[-1].states = motion_trajectory
            node.children[-1].depth = node.depth+1
            node.children[-1].duration = duration

            stack.append((node.children[-1], motion_trajectory[-1]))

    return root


def interpolate_color(start_color, end_color, t):
    color = ColorRGBA()
    color.r = start_color.r + (end_color.r - start_color.r) * t
    color.g = start_color.g + (end_color.g - start_color.g) * t
    color.b = start_color.b + (end_color.b - start_color.b) * t
    color.a = start_color.a + (end_color.a - start_color.a) * t
    return color

def visualize_motion_primitive(marker_pub, motion_primitive_node, _id):
    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = "odom"  # Replace with the appropriate frame ID
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.1  # Line width
    marker.id = _id

    # Define the start and end colors for the color gradient
    start_color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue
    end_color = ColorRGBA(1.0, 0.0, 0.0, 1.0)    # Red

    # Set the positions and colors of the marker based on the states in the motion primitive node
    num_states = len(motion_primitive_node.states)
    for i, state in enumerate(motion_primitive_node.states):
        point = Point()
        point.x = state[0]
        point.y = state[1]
        point.z = 0.0
        marker.points.append(point)

        # Interpolate the color based on the time step
        t = float(i) / (num_states - 1)  # Normalized time step between 0.0 and 1.0
        color = interpolate_color(start_color, end_color, t)
        marker.colors.append(color)

    # Publish the Marker message
    marker_pub.publish(marker)


# Callback function for odometry data
def odom_callback(data):
    global odom_data
    odom_data = data

    orientation = odom_data.pose.pose.orientation
    # Convert the quaternion to Euler angles (roll, pitch, yaw)
    euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

    # Extract the yaw angle (rotation around the z-axis)
    yaw = euler[2]

    init_state = [odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, yaw]

    dt = .1
    max_depth = 5 # Maximum depth of the tree
    tree_root = generate_motion_primitive_tree(max_depth, init_state, dt)

    # Generate a random backward motion
    motion_trajectory_nodes = generate_random_backward_motion(tree_root)

    marker_pub = rospy.Publisher("/motion_primitive_marker", Marker, queue_size=1)

    # Publish the motion commands
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    for ind, node in enumerate(motion_trajectory_nodes):
        # Create Twist message with motion commands
        visualize_motion_primitive(marker_pub, node, ind)
        node.execute(velocity_pub, dt)

    rospy.signal_shutdown("Execution complete")

# Callback function for lidar data
def lidar_callback(data):
    global lidar_data
    lidar_data = data

def generate_random_backward_motion(tree_root):
    ret = [tree_root.children[randrange(len(motion_primitives))]]
    
    for _ in range(4):
        ret.append(ret[-1].children[randrange(len(motion_primitives))])

    return ret

# ROS node initialization
def motion_recovery_node():
    global odom_data, lidar_data

    rospy.init_node('motion_recovery_node', anonymous=True)

    # Subscribe to odometry and lidar topics
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    rospy.Subscriber('/front/scan', LaserScan, lidar_callback)

    

    rospy.spin()

if __name__ == '__main__':
    motion_recovery_node()
