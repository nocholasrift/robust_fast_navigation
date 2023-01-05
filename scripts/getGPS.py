#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

initial_pos = None

def gazeboModelCallback(data):
    global gpsMsg, initial_pos

    objList = data.name
    poseList = data.pose
    # rospy.loginfo(data)
    jackal0_GPS = poseList[objList.index("jackal")]

    gpsMsg.header.frame_id = "odom"
    gpsMsg.header.stamp = rospy.Time.now()
    gpsMsg.pose.pose.position.x = jackal0_GPS.position.x
    gpsMsg.pose.pose.position.y = jackal0_GPS.position.y
    gpsMsg.pose.pose.position.z = jackal0_GPS.position.z
    gpsMsg.pose.pose.orientation.x = jackal0_GPS.orientation.x
    gpsMsg.pose.pose.orientation.y = jackal0_GPS.orientation.y
    gpsMsg.pose.pose.orientation.z = jackal0_GPS.orientation.z
    gpsMsg.pose.pose.orientation.w = jackal0_GPS.orientation.w


def main():
    global gpsMsg
    rospy.init_node("global_pos", anonymous=True)

    gpsMsg = Odometry()
    gps_pub = rospy.Publisher("/gmapping/odometry", Odometry, queue_size=10)
    
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazeboModelCallback)
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        gps_pub.publish(gpsMsg)
        rate.sleep()


if __name__ == "__main__":
    main()
