#!/usr/bin/env python  
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped

import sys

F = open("stamped_traj_estimate.txt", "w") 
def callback(data):

    pose = PoseStamped() 

# Set a atributes of the msgs
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("map", "base_scan", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        

        pose.header.frame_id = "odom"
        pose.pose.position.x =  float(trans.transform.translation.x)
        pose.pose.position.y = float(trans.transform.translation.y)
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


        F.write(str(pose.header.stamp) + " " + str(pose.pose.position.x) + " " + 
        str(pose.pose.position.y) + " " + str(pose.pose.position.z) + " " + 
        str(pose.pose.orientation.x) + " " + str(pose.pose.orientation.y)+ " " + 
        str(pose.pose.orientation.z) + " " + str(pose.pose.orientation.w) + "\n")        
        pub.publish(path_GM)
        break

    

if __name__ == '__main__':
    rospy.init_node('transform')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)

    pub = rospy.Publisher('/path_GM', Path, queue_size=10)

    path_GM = Path() 
    msg = Odometry()

    # Subscription to the topic
    msg = rospy.Subscriber('/odom', Odometry, callback)
    
    while not rospy.is_shutdown():    
        rospy.spin()

    F.close()
