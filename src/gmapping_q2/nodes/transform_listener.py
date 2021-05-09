#!/usr/bin/env python  
import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
import geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped

import sys


if __name__ == '__main__':
    rospy.init_node('transform_listener')
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    mapToBasefootprint = rospy.Publisher('/map2Base_fp', geometry_msgs.msg.Pose, queue_size=1)

    rate = rospy.Rate(26.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("map", "base_footprint", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        
        estimatedPosition = geometry_msgs.msg.Pose()

        estimatedPosition.position.x =  float(trans.transform.translation.x)
        estimatedPosition.position.y = float(trans.transform.translation.y)
        estimatedPosition.position.z = float(trans.transform.translation.z)
        estimatedPosition.orientation.x = float(trans.transform.rotation.x) 
        estimatedPosition.orientation.y = float(trans.transform.rotation.y) 
        estimatedPosition.orientation.z = float(trans.transform.rotation.z) 
        estimatedPosition.orientation.w = float(trans.transform.rotation.w)

        mapToBasefootprint.publish(estimatedPosition)

        rate.sleep()