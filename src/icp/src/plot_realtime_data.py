import matplotlib.pyplot as plt
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation
from threading import RLock


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        # self.ln, = plt.plot([], [], 'ro')
        self.gt_odom_line, = self.ax.plot([], [], 'ro')
        self.icp_odom_line, = self.ax.plot([], [], 'go')
        self.x_gt_data, self.y_gt_data = [] , []
        self.x_icp_data, self.y_icp_data = [] , []
        self.lock = RLock()

    def plot_init(self):
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        # return self.ln

    def getYaw(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z,
                pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] 
        return yaw   

    def odom_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        self.y_gt_data.append(msg.pose.pose.position.y)
        # x_index = len(self.x_data)
        self.x_gt_data.append(msg.pose.pose.position.x)

    def icp_odom_callback(self, msg):
        # yaw_angle = self.getYaw(msg.pose.pose)
        self.y_icp_data.append(msg.pose.pose.position.y)
        # x_index = len(self.x_data)
        self.x_icp_data.append(msg.pose.pose.position.x)

    def update_plot(self, frame):
        self.lock.acquire()
        self.gt_odom_line.set_data(self.x_gt_data, self.y_gt_data)
        self.icp_odom_line.set_data(self.x_icp_data, self.y_icp_data)
        self.lock.release()
        # return self.ln


rospy.init_node('lidar_visual_node')
vis = Visualiser()
sub = rospy.Subscriber('/icp_odom', Odometry, vis.icp_odom_callback)
sub = rospy.Subscriber('/odom', Odometry, vis.odom_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init)
plt.show(block=True)