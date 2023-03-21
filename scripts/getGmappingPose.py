#! /usr/bin/env python3

import tf
import sys
import rospy
import tf2_ros
# import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

odom = None


def pose_callback(data):
    global odom
    odom = data

def rotate_orientation(ori, q):
    q = (
        q.x,
        q.y,
        q.z,
        q.w,
    )
    rot_mat = tf.transformations.quaternion_matrix(q)
    pose_rot = rot_mat.dot([ori.x, ori.y, ori.z, ori.w])
    ori.x = pose_rot[0]
    ori.y = pose_rot[1]
    ori.z = pose_rot[2]
    ori.w = pose_rot[3]

def translate_position(pos, t):
    pos.x += t.x
    pos.y += t.y
    pos.z += t.z

def main():
    global odom
    rospy.init_node("gmapping_pose_publisher", anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, pose_callback)

    pose_pub = rospy.Publisher("gmapping/odometry", Odometry, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(20)
    i = 0

    transform = None
    while not rospy.is_shutdown():
        rate.sleep()

        if odom == None:
            continue

        tmp = PoseStamped()
        tmp.pose = odom.pose.pose
        tmp.header.frame_id = "/odom"
        tmp.header.stamp = rospy.Time.now()

        try:
            if i % 10 == 0:
                trans = tf_buffer.lookup_transform(
                    "map", "odom", rospy.Time(0), rospy.Duration(1.0)
                )
                i = 0

            translate_position(odom.pose.pose.position, trans.transform.translation)    
            rotate_orientation(odom.pose.pose.orientation, trans.transform.rotation)
            # pose_transformed = tf2_geometry_msgs.do_transform_pose(tmp, transform)
            # pose_transformed = tf_buffer.transformPose('map', tmp.pose)

            # pose_transformed = 

        except Exception as e:
            rospy.loginfo(e)
            continue

        pose_pub.publish(odom)


if __name__ == "__main__":
    main()