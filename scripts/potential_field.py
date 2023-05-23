#! /usr/bin/env python

# This ROS node computes a potential field sum based on
# the laser scan readings, and publishes this sum for
# use in other nodes' obstacle avoidance logic

import math
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Twist

from tf.transformations import euler_from_quaternion

#------------------------------------------

class potential_field:
    def __init__(self):
        rospy.init_node("potential_field")

        laser_topic = rospy.get_param("~laser_topic", "/front/scan")
        potential_field_topic = rospy.get_param("~potential_field_topic", "potential_field_sum")

        self.sample_rate = rospy.get_param("~sample_rate", 30)

        self.min_angle = rospy.get_param("~min_angle", -np.pi)
        self.max_angle = rospy.get_param("~max_angle", np.pi)
        self.side_obstacle_force = rospy.get_param("~side_obstacle_force", 5.0)
        self.front_obstacle_force = rospy.get_param("~front_obstacle_force", 5.0)

        self.cmd_sub = rospy.Subscriber("/mpc_vel", Twist, self.handle_cmd)
        self.laser_sub = rospy.Subscriber(laser_topic, LaserScan, self.handle_laser)
        self.odom_sub = rospy.Subscriber("/gmapping/odometry", Odometry, self.handle_odom)

        self.viz_pub = rospy.Publisher("/force", Marker, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.potential_field_pub = rospy.Publisher(potential_field_topic, Point, queue_size=10)

        # laser readings
        self.cmd = None
        self.odom = None
        self.laser = None

#------------------------------------------

    def start(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            self.compute_potential()
            rate.sleep()

#------------------------------------------

    def handle_cmd(self, data):
        self.cmd = data

#------------------------------------------

    def handle_odom(self, data):
        self.odom = data

#------------------------------------------

    def compute_potential(self):
        if self.laser == None or self.odom == None:
            return

        Q = .7
        laser_data = self.laser
        ranges = laser_data.ranges
        angle = laser_data.angle_min
        resolution = laser_data.angle_increment

        vector_sum = np.array([0.0, 0.0])
        count = 0

        for r in ranges:
            if (r < laser_data.range_min or
                    angle > self.max_angle or
                    angle < self.min_angle):
                distance = laser_data.range_max
            else:
                distance = r

            # mag = 1.0 / (distance * distance) #* self.compute_force(angle, laser_data.angle_max)
            mag = -.5*(1.0/distance - 1.0/Q)/(distance*distance) if distance <= Q else 0
            if distance <= Q:
                count += 1

            vector = np.array([np.cos(angle), np.sin(angle)])
            vector *= mag

            vector_sum += vector

            # if distance <=.5:
            #     print("MAG IS ", mag, "\tDistance is", distance)
            
            angle += resolution

        if count != 0:
            # vector_sum /= len(ranges) # no division by zero
            vector_sum /= count
            # print("VECTOR SUM IS ", vector_sum)

        self.publish_sum(vector_sum[0], vector_sum[1])
        
        if self.cmd != None:
            quat = self.odom.pose.pose.orientation
            (yaw_r, _, _)=euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))

            msg = Twist()
            msg.linear.x = self.cmd.linear.x
            msg.angular.z = np.clip(self.cmd.angular.z 
                                    + .15 * (math.atan2(vector_sum[1], vector_sum[0])-yaw_r),
                                    -np.pi, np.pi)
            self.cmd_pub.publish(msg)

#------------------------------------------

    def handle_laser(self, laser_data):
        self.laser = laser_data
        
#------------------------------------------

    # make the force linear in the angle of the reading,
    # with the strongest forces from obstacles in front and
    # weakest from obstacles to the sides
    def compute_force(self, theta, max_theta):
        diff = abs(theta)
        normalized_diff = diff / abs(max_theta)
        slope = self.front_obstacle_force - self.side_obstacle_force
        return (1.0 - normalized_diff) * slope + self.side_obstacle_force

#------------------------------------------

    def publish_sum(self, x, y):
        quat = self.odom.pose.pose.orientation
        (yaw_r, _, _)=euler_from_quaternion((quat.x,quat.y,quat.z,quat.w))
        rot=np.array([[np.cos(yaw_r),-np.sin(yaw_r)],[np.sin(yaw_r), np.cos(yaw_r)]])
        vec = np.matmul(rot,np.array([x,y]))
        # vec = np.array([x,y])
        vector = Point(vec[0], vec[1], 0)
        self.potential_field_pub.publish(vector)
        
        if self.odom != None:
            # rospy.logerr("HELLO????")
            msg = Marker()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            msg.ns = "force"
            msg.id = 597
            msg.type = Marker.ARROW
            msg.action = Marker.ADD
            msg.scale.x = .2
            msg.scale.y = .2
            msg.scale.z = .2
            msg.color.b = 1
            msg.color.a = 1
            
            p1 = Point()
            p1.x = self.odom.pose.pose.position.x
            p1.y = self.odom.pose.pose.position.y
            msg.points.append(p1)

            p2 = Point()
            p2.x = p1.x+vec[0]
            p2.y = p1.y+vec[1]
            msg.points.append(p2)
            self.viz_pub.publish(msg)


#------------------------------------------

if __name__ == "__main__":
    pf = potential_field()
    pf.start()
