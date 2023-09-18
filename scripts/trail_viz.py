#!/usr/bin/env python3

import copy
import rospy
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry 
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point, PoseStamped

# cmap = plt.cm.get_cmap('viridis')
cmap = matplotlib.colormaps['plasma']

trail_strip = Marker()
trail_mark_arr = MarkerArray()
odom_msg = None
prev_odom = None

trail_pub = None
v = 0
max_vel = 1.2

ros_timer = None
is_recover = False
rec_point = None

def recoveryGoalCB(msg):
    global is_recover, rec_point
    is_recover = True
    rec_point = msg.pose.position

def odomCB(msg):
    global odom_msg, v, is_recover, rec_point

    if odom_msg != None:
        x1 = odom_msg.pose.pose.position.x
        y1 = odom_msg.pose.pose.position.y

        x2 = msg.pose.pose.position.x
        y2 = msg.pose.pose.position.y

        v = np.linalg.norm([x1-x2,y1-y2])*30

        if is_recover and np.linalg.norm([x1-rec_point.x,y1-rec_point.y]) < .2:
            is_recover = False

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

    trail_strip.scale.x = .2

    trail_strip.pose.orientation.w = 1

    trail_size = 100000

    p1 = Point()
    p1.x = odom_x
    p1.y = odom_y
    if is_recover:
        p1.z = 1.
    else:
        p1.z = 0.
    
    if len(trail_strip.points) >= trail_size:
        del trail_strip.points[0]
        del trail_strip.colors[0]

    trail_strip.points.append(p1)

    frac = v / max_vel
    rgba = cmap(frac)

    color = ColorRGBA()
    color.a = 1
    if is_recover:
        color.r = 0
        color.g = 1
        color.b = 0
    else:
        color.r = rgba[0]
        color.g = rgba[1]
        color.b = rgba[2]

    trail_strip.colors.append(color)

    trail_pub.publish(trail_strip)

    prev_odom = odom_msg

def main():
    global   trail_pub, max_vel

    rospy.init_node("trail_publisher")

    odom_sub = rospy.Subscriber("/gmapping/odometry", Odometry, odomCB)
    rec_goal_sub = rospy.Subscriber("/recoveryGoal", PoseStamped, recoveryGoalCB)
    trail_pub = rospy.Publisher("/trail", Marker, queue_size=1)
    trail_timer = rospy.Timer(rospy.Duration(.05), updateTrail)
    
    rospy.spin()

if __name__ == "__main__":
    main()
