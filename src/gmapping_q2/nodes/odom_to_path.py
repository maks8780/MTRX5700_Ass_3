#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
import tf2_ros
import copy
import sys
import time


F_ground = open("/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/src/gmapping_q2/errors/stamped_groundtruth.txt", "w") 
F_trans = open("/home/mtrx5700/Documents/University/MTRX5700/MTRX5700_Ass_3/src/gmapping_q2/errors/stamped_traj_estimate.txt", "w") 

currentTime = rospy.Time(0)

def odomCallback(data):
    
    global pose 
    pose = PoseStamped()

    pose.header.frame_id = "odom"
    pose.pose.position.x = data.pose.pose.position.x
    pose.pose.position.y = data.pose.pose.position.y
    pose.pose.position.z = data.pose.pose.position.z
    pose.pose.orientation.x = data.pose.pose.orientation.x
    pose.pose.orientation.y = data.pose.pose.orientation.y
    pose.pose.orientation.z = data.pose.pose.orientation.z
    pose.pose.orientation.w = data.pose.pose.orientation.w

    global timeStamp
    timeStamp = rospy.Time.now()

    # Set attributes of the msg
    path_GT.header.frame_id = "odom"
    path_GT.header.stamp = timeStamp
    pose.header.stamp = path_GT.header.stamp
    path_GT.poses.append(pose)
    # Published the msg

    pub_ground.publish(path_GT)

        
def transCallback(trans):  

    global pose_trans
    global currentTime
    pose_trans = PoseStamped()

    pose_trans.header.frame_id = "odom"
    pose_trans.pose.position.x = trans.position.x
    pose_trans.pose.position.y = trans.position.y
    pose_trans.pose.position.z = trans.position.z
    pose_trans.pose.orientation.x = trans.orientation.x 
    pose_trans.pose.orientation.y = trans.orientation.y 
    pose_trans.pose.orientation.z = trans.orientation.z
    pose_trans.pose.orientation.w = trans.orientation.w

    # Set a atributes of the msg
    path_GM.header.frame_id = "odom"
    path_GM.header.stamp = timeStamp
    pose_trans.header.stamp = path_GM.header.stamp
    path_GM.poses.append(pose_trans)
    # Published the msg

    pub_trans.publish(path_GM)

    if timeStamp - currentTime > rospy.Duration(0.5):
        F_trans.write(str(timeStamp) + " " + str(pose_trans.pose.position.x) + " " + 
        str(pose_trans.pose.position.y) + " " + str(pose_trans.pose.position.z) + " " + 
        str(pose_trans.pose.orientation.x) + " " + str(pose_trans.pose.orientation.y)+ " " + 
        str(pose_trans.pose.orientation.z) + " " + str(pose_trans.pose.orientation.w) + "\n") 

        F_ground.write(str(timeStamp) + " " + str(pose.pose.position.x) + " " + 
        str(pose.pose.position.y) + " " + str(pose.pose.position.z) + " " + 
        str(pose.pose.orientation.x) + " " + str(pose.pose.orientation.y)+ " " + 
        str(pose.pose.orientation.z) + " " + str(pose.pose.orientation.w) + "\n")

        currentTime = copy.deepcopy(timeStamp)
    

if __name__ == '__main__':

    # Node and msg initialization
    rospy.init_node('path_plotter')
 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(50.0)

    pub_ground = rospy.Publisher('/path_GT', Path, queue_size=10)
    pub_trans = rospy.Publisher('/path_GM', Path, queue_size=10)

    path_GT = Path() 
    path_GM = Path() 
    
    # Subscription to the topic
    odomSub = rospy.Subscriber('/odom', Odometry, odomCallback)
    transSub = rospy.Subscriber('/map2Base_fp', Pose, transCallback)

    try:
        while not rospy.is_shutdown():         
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    F_trans.close()
    F_ground.close()
