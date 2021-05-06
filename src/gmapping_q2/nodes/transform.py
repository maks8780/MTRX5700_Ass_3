#!/usr/bin/env python  
import rospy

import tf2_ros
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped

import sys

def callback(data):

    pose = PoseStamped() 
    # trans = TransformStamped()

# Set a atributes of the msgs
    
    trans = tfBuffer.lookup_transform("map", "odom", rospy.Time())

    pose.pose.position.x =  float(trans.transform.translation.x) + float(data.pose.pose.position.x)
    pose.pose.position.y = float(trans.transform.translation.y) + float(data.pose.pose.position.y)
    pose.pose.orientation.x = float(trans.transform.rotation.x) 
    pose.pose.orientation.y = float(trans.transform.rotation.y) 
    pose.pose.orientation.z = float(trans.transform.rotation.z) 
    pose.pose.orientation.w = float(trans.transform.rotation.w)

    # Set a atributes of the msg
    pose.header.seq = path_GM.header.seq + 1
    path_GM.header.frame_id = "odom"
    path_GM.header.stamp = rospy.Time.now()
    pose.header.stamp = path_GM.header.stamp
    path_GM.poses.append(pose)
    # Published the msg

    pub.publish(path_GM)


if __name__ == '__main__':
    rospy.init_node('transform')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/path_GM', Path, queue_size=50)

    path_GM = Path() 
    msg = Odometry()

    # Subscription to the topic
    msg = rospy.Subscriber('/odom', Odometry, callback)

    rospy.spin()