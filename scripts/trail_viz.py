#!/usr/bin/env python3

import copy
import rospy
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry 
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

cmap = plt.cm.get_cmap('viridis')

trail_strip = Marker()
trail_mark_arr = MarkerArray()
odom_msg = None
prev_odom = None

trail_pub = None
v = 0
max_vel = 1

ros_timer = None

def odomCB(msg):
    global odom_msg, v

    if odom_msg is not None:
        x1 = odom_msg.pose.pose.position.x
        y1 = odom_msg.pose.pose.position.y

        x2 = msg.pose.pose.position.x
        y2 = msg.pose.pose.position.y

        v = np.linalg.norm([x1-x2,y1-y2])*30

    odom_msg = msg

def updateTrail(event):

    global odom_msg, prev_odom, cmap, v, trail_mark_arr, trail_strip, max_vel, ros_timer

    if odom_msg is None:
        return
    
    if ros_timer is None:
        ros_timer = rospy.Time.now()

    msg = MarkerArray()

    odom_x = odom_msg.pose.pose.position.x
    odom_y = odom_msg.pose.pose.position.y

    deleteMsg = Marker()
    deleteMsg.header.stamp = rospy.Time.now()
    deleteMsg.header.frame_id = "map"
    deleteMsg.ns = "collision"
    deleteMsg.action = Marker.DELETEALL

    msg.markers.append(deleteMsg)

    if odom_msg is None or prev_odom is None:
        if odom_msg is not None:
            prev_odom = odom_msg
        return


    trail_strip.header.frame_id = "map"
    trail_strip.header.stamp = rospy.Time.now()
    trail_strip.id = 0
    trail_strip.ns = 'trail'
    trail_strip.action = Marker.ADD
    trail_strip.type = Marker.LINE_STRIP

    trail_strip.scale.x = .1

    trail_strip.pose.orientation.w = 1

    trail_size = 100

    p1 = Point()
    p1.x = odom_x
    p1.y = odom_y
    p1.z = .1
    
    if len(trail_strip.points) >= trail_size:
        del trail_strip.points[0]
        del trail_strip.colors[0]

    trail_strip.points.append(p1)

    rgba = cmap(v/max_vel)
    color = ColorRGBA()
    color.r = rgba[0]
    color.g = rgba[1]
    color.b = rgba[2]
    color.a = 1

    trail_strip.colors.append(color)

    trail_pub.publish(trail_strip)

    prev_odom = odom_msg

def main():
    global   trail_pub, max_vel

    rospy.init_node("trail_publisher")

    max_vel = rospy.get_param("/trail_viz/max_velocity", "1.0")
    print("max vel is", max_vel)

    odom_sub = rospy.Subscriber("/gmapping/odometry", Odometry, odomCB)
    trail_pub = rospy.Publisher("/trail", Marker, queue_size=1)
    trail_timer = rospy.Timer(rospy.Duration(.05), updateTrail)
    
    rospy.spin()

if __name__ == "__main__":
    main()
