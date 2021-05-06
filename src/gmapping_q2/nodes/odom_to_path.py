#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf

import sys
import time


def callback(data):

    pose = PoseStamped() 

# Set a atributes of the msgs
    
    pose.header.frame_id = "odom"
    pose.pose.position.x = float(data.pose.pose.position.x)
    pose.pose.position.y = float(data.pose.pose.position.y)
    pose.pose.orientation.x = float(data.pose.pose.orientation.x)
    pose.pose.orientation.y = float(data.pose.pose.orientation.y)
    pose.pose.orientation.z = float(data.pose.pose.orientation.z)
    pose.pose.orientation.w = float(data.pose.pose.orientation.w)

    # Set a atributes of the msg
    pose.header.seq = path_GT.header.seq + 1
    path_GT.header.frame_id = "odom"
    path_GT.header.stamp = rospy.Time.now()
    pose.header.stamp = path_GT.header.stamp
    path_GT.poses.append(pose)
    # Published the msg

    pub.publish(path_GT)



if __name__ == '__main__':


    # Node and msg initialization
    rospy.init_node('path_plotter')

    pub = rospy.Publisher('/path_GT', Path, queue_size=50)

    path_GT = Path() 
    msg = Odometry()

    # Subscription to the topic
    msg = rospy.Subscriber('/odom', Odometry, callback)

    rospy.spin()


