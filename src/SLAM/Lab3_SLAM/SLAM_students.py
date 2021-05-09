#!/usr/bin/env python

import rospy
import numpy as np
#Message types
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from landmarks_msg.msg import Landmarks_Msg
#Functions
from Relative2AbsolutePose import Relative2AbsolutePose
from Relative2AbsoluteXY import Relative2AbsoluteXY
from Absolute2RelativeXY import Absolute2RelativeXY
from pi2pi import pi2pi

from visualization_msgs.msg import Marker, MarkerArray
from threading import RLock


from scipy.spatial.transform import Rotation as R
# from tf_conversions import transformations
from std_msgs.msg import Float32
import sys

# standar deviation for simulated odometry
std_dev_linear_vel =  0.1
std_dev_angular_vel =  (5 * np.pi) / 180

# landmarks' most recent absolute coordinate
seenLandmarks_ = []

# State Transition Model
F_ = []
# Control-Input Model
W_ = []
# dimension of robot pose
dimR_ = 3
# Sequence counter
seq_ = 0


class Robot ():
    """The Robot Class."""

    def __init__(self, pose, pose_cov, sensor_type):
        """Initialize the class.

        :param pose: Pose of the robot. It is a 3x1 column array.
        :type pose: np.array
        :param pos_Cov: Covariance of the pose. It is a 3x3 array.
        :type pos_Cov: np.array
        :param sense_Type: Type of the sensor.
        :type sense_Type: str
        """
        self.x = pose[0][0]                 # x coordinate of the pose
        self.y = pose[1][0]                 # y coordinate of the pose
        self.theta = pose[2][0]             # orientation
        self.pose_covariance = pose_cov     # covariance of the pose
        self.sensor_type = sensor_type      # sensor type - 'Vision'

    def setPose(self, new_pose):
        """Set the pose of the robot with the value specified.

        :param new_pose: The new pose of the robot.
        :type new_pose: np.array
        """
        self.x = new_pose[0][0]
        self.y = new_pose[1][0]
        self.theta = new_pose[2][0]

    def getPose(self):
        """Get the current pose of the robot.

        :return: The current pose of the robot
        :rtype: np.array
        """
        return [[self.x], [self.y], [self.theta]]

    def setCovariance(self, new_cov):
        """Set the covariance of the pose with the value specified.

        :param new_cov: The new covariance of the pose. It is a 3x3 array
        :type new_cov: np.array
        """
        self.pose_covariance = new_cov

    def getCovariance(self):
        """Get the covariance of the pose.

        :return: The covariance of the pose. It is a 3x3 array
        :rtype: np.array
        """
        return self.pose_covariance

    def move(self, current_robot_pose_abs, u_rel):
        """Execute a move step for the robot.

        :param current_robot_pose_abs: The current pose of the robot in the
        absolute frame of reference. 3x1 array
        :type current_robot_pose_abs: np.array
        :param u_rel: the motion command [dx, dy, dtheta]
        in the robot's frame of reference. 3x1 array
        :type u_rel: np.array
        :return: The new pose of the robot in the absolute frame and Jacobians
        of the transition matrix wrt the previous pose and the motion command.
        :rtype: list
        """
        # compute the new pose using the transition model and the motion
        [next_robot_pose_abs, H1, H2] = Relative2AbsolutePose(
                                                    current_robot_pose_abs,
                                                    u_rel)
        # update the pose of the robot
        self.setPose(next_robot_pose_abs)
        # return the new pose of the robot and the Jacobians
        return next_robot_pose_abs, H1, H2

    def sense(self, current_robot_pose_abs, landmark_position_abs):
        """Execute a sense step, i.e. given the current pose of the robot and
        the position of a landmark, both in the absolute frame, computes the
        position of the landmark in the robot's frame of reference
        :param current_robot_pose_abs: current pose of the robot in the
        absolute frame of reference
        :type current_robot_pose_abs: np.array
        :param landmark_position_abs: position of a landmark in the absolute
        frame of reference
        :type landmark_position_abs: np.array
        :return: The position of the landmark in the robot's frame of
        reference and the Jacobians
        :rtype: list
        """
        if self.sensor_type == 'Vision':
            [measurement, H1, H2] = Absolute2RelativeXY(current_robot_pose_abs,
                                                        landmark_position_abs)
        else:
            raise ValueError('Unknown Measurement Type')

        return measurement, H1, H2

    def inverseSense(self, robotCurrentAbs, measurement):
        """
        Executes an inverse sense step, i.e. given the current pose of the robot in the absolute frame
        of reference and the position of a landmark in the robot's frame of reference, computes the position of the
        landmark in the absolute frame of reference
        :param robotCurrentAbs: current pose of the robot in the absolute frame of reference
        :param measurement: position of a landmark in the robot's frame of reference
        :return : The position of the landmark in the absolute frame of reference and the Jacobians
        """
        if self.sensor_type == 'Vision':
            [landmark_abs, H1, H2] = Relative2AbsoluteXY(robotCurrentAbs,
                                                         measurement)
        else:
            raise ValueError('Unknown Measurement Type')

        return landmark_abs, H1, H2


class KalmanFilter():
    """
    This is the class that implements the Landmark SLAM Kalman Filter.
    """
    def __init__(self, mean, covariance, robot):
        """
        Initialize the object.

        :param mean: mean of the initial robot pose
        :type mean: np.array
        :param covariance: covariance of the initial robot pose
        :type covariance: [type]
        :param robot: the robot model
        :type robot: Robot
        """
        self.state_mean = mean
        self.state_covariance = covariance
        self.robot = robot
        self.landmarks = []
        self.taglist = []


    @property
    def number_landmarks(self):
        return len(self.landmarks)

    def setStateMean(self, mean):
        """
        Sets the mean of the state to the input mean values.

        :param mean: the values to which the mean of the state has to be set.
        :type mean: np.array
        """
        self.state_mean = mean

    def getStateMean(self):
        """
        Returns the mean of the state.
        :return : the mean of the state

        """
        return self.state_mean

    def setStateCovariance(self, covariance):
        """
        Sets the covariance of the state to the input covariance values
        :param covariance: the values to which the covariance of the state has
        to be set
        :type covariance: np.array
        """
        self.state_covariance = covariance

    def getStateCovariance(self):
        """
        Returns the covariance of the state
        :return : the covariance of the state
        """
        return self.state_covariance

    def predict(self, motion, motion_covariance):
        """
        Predicts the next state of the system using linear-gaussian update for
        state and covariance.
        :param motion: a vector of the form [dx, dy, dtheta] corresponding to motion in the robot's frame of reference.
        :param motion_cov: should be the 3 x 3 covariance matrix for the motion vector
        """
        # motion = np.array(motion)
        # motion_covariance = np.array(motion_covariance)
        prior_robot_pose, F, W = self.robot.move(self.robot.getPose(), np.array(motion))
        W = np.array(W)
        F = np.array(F)

        # get the prior covariance of the entire state incl landmarks
        state_covariance = self.getStateCovariance()
        robot_state_covariance = state_covariance[0:3, 0:3]

        print("cov\n{}".format(state_covariance))
        print("robot cov\n{}".format(robot_state_covariance))

        # F_scattered = np.identity(robot_pose_covariance.shape[0])
        # F_scattered[0:3, 0:3] = F

        # Create the new covariance using the previous calculations, then update both covariances
        print(W @ motion_covariance @ W.T)
        robot_state_covariance = F @ robot_state_covariance @ F.T + W @ motion_covariance @ W.T
        
        state_covariance[0:3, 0:3] = robot_state_covariance
        self.setStateCovariance(state_covariance)

        state_mean = self.getStateMean()
        state_mean[0:3] = prior_robot_pose
        state_mean = state_mean.reshape(state_mean.shape[0], 1)


        self.setStateMean(state_mean)
        
    def update(self, measurement, measurementCovariance, new):
        """
        Updates the posterior mean and covariance of the state given a new measurement.
        :measurement: measurement of the form (dx, dy, label) 
        :param measurementCovariance: covariance of the measurement
        :return : posterior state mean and posterior state covariance
        """
        global seenLandmarks_
        # measurement = np.array(measurement)
        # measurement = measurement.reshape(measurement.shape[0], 1)

        # measurementCovariance = np.array(measurementCovariance)
        
        measurement_absolute, G1, G2 = self.robot.inverseSense(self.robot.getPose(), measurement)
        label = measurement[2]

        measurement_absolute = np.array(measurement_absolute)
        G1 = np.array(G1, dtype=np.int16)
        G2 = np.array(G2)
        

        state_covariance_matrix = self.getStateCovariance()
        print("init sate cov {}".format(state_covariance_matrix))
        # state_covariance_matrix = np.array(state_covariance_matrix)

        state_vector = self.getStateMean()
        print("init state mean {}".format(state_vector))
        # state_vector = np.array(state_vector)

        #Augment the state vector if the landmark is new
        #TODO: should this be the relative landmark position or the absolute xy coordinate system?
        #TODO: need to initalize with the inverse measurement function?
        if new == 1:
            #add new measurements to end of state
            #do not include landmark label in measurement
            # assert(measurement_absolute.shape == (2,1))
            state_vector = np.append(state_vector, measurement_absolute)
            state_vector = state_vector.reshape((state_vector.shape[0], 1))

            self.setStateMean(state_vector)
            rospy.loginfo("state vec\n {}".format(state_vector))
            rospy.loginfo("state vec shape\n {}".format(state_vector.shape))


            # Augment state covariance if the landmark is new 
            # add new 3 col and 3 row and fix measurement covariance at end 

            #get values of first row of covariance matrix

            # print(G1)
            print(state_covariance_matrix)
            # print(state_covariance_matrix[0:3,:])
            # print("here")

            new_row_covariance = G1 @ state_covariance_matrix[0:3, :]
            new_col_covariance = new_row_covariance.T


            # rospy.loginfo("new row cov\n {}".format(new_row_covariance))
            rospy.loginfo("new col cov\n {}".format(new_col_covariance))
            # rospy.loginfo("state cov\n {}".format(state_covariance_matrix))


            #construct covariance matrix with new landmark covariance in them
            state_covariance_matrix= np.concatenate((state_covariance_matrix,new_col_covariance),axis=1)
            new_row_covariance = np.concatenate((new_row_covariance, measurementCovariance), axis=1)
            print(state_covariance_matrix)
            print(new_row_covariance)

            state_covariance_matrix = np.concatenate((state_covariance_matrix,new_row_covariance))
            rospy.loginfo("state cov\n {}".format(state_covariance_matrix))
            self.setStateCovariance(state_covariance_matrix)

            # if new == 1:
            #     rospy.signal_shutdown("yes")
            #     sys.exit(0)


        # TODO Calculate expected measurement
        # assert(label in seenLandmarks_)
        print("Seen landmakrs {}".format(seenLandmarks_))
        robot_pose = np.array(self.robot.getPose()[0:2])
        label_index = seenLandmarks_.index(label)
        start_index_state_vector = (label_index)*2 + 3
        end_index_stat_vector = (label_index)*2 + 5

        landmark_location = self.getStateMean()[start_index_state_vector : end_index_stat_vector]
        # estimated_landmark_position = estimated_landmark_position.reshape(estimated_landmark_position.shape[0], 1)

        relative_estimated_landmark_position =  np.array(landmark_location) - robot_pose
        print("relative {}".format(relative_estimated_landmark_position))

        # TODO Calculate the innovation
        innovation = np.atleast_2d(np.array(measurement[0:2])).T - relative_estimated_landmark_position
        N = state_covariance_matrix.shape[0]
        print("N ", N)
        C = np.zeros((2, N))
        rospy.loginfo("innovation {}".format(innovation))



        # if new == 1:
        #     rospy.signal_shutdown("yes")
        #     sys.exit(0)


        C[:, 0:3] = G1
        C[:, start_index_state_vector:end_index_stat_vector] = G2

        rospy.loginfo("C shape\n {}".format(C.shape))
        rospy.loginfo("cov shape\n {}".format(state_covariance_matrix.shape))
        rospy.loginfo("measur shape\n {}".format(measurementCovariance.shape))
        rospy.loginfo("C \n {}".format(C.dtype))

        rospy.loginfo("state_type \n {}".format(state_covariance_matrix))


        # TODO Construct measurement Jacobian
        K = np.array(state_covariance_matrix @  C.T @ np.linalg.inv((C @ state_covariance_matrix @ C.T + measurementCovariance)))
        # TODO Calculate the Innovation matrix

        rospy.loginfo("K {}".format(K.shape))

        # TODO Calculate the Sensor state

        # TODO Calculate the Kalman gain
        
        # TODO Update posterior mean and cov
        state_vector = self.getStateMean()
        # state_vector = state_vector.reshape((state_vector.shape[0], 1))
        # update_state = (K @ innovation)
        # print(update_state)
        # print(update_state.shape)

        rospy.loginfo("State vector {}".format(state_vector))
        print("State veoctor size {}".format(state_vector.shape))
        # print(state_vector.shape)
        # state_vector = np.add(state_vector, update_state)
        state_vector += K @ innovation


        state_covariance_matrix = ( np.eye(N) - K @ C) @ state_covariance_matrix
        # print("Final cov shape: {}".format(state_covariance_matrix.shape))
        self.setStateCovariance(state_covariance_matrix)

        state_vector = state_vector.reshape((state_vector.shape[0], 1))
        self.setStateMean(state_vector)


        # return posterior_state_mean, posterior_state_covariance
        
class SLAM(KalmanFilter):

    def callbackOdometryMotion(self, msg): # imu):
        """
        This will be called every time we receive an odom reading
        """
        # Compute dtrobot_pose = duration of the sensor measurement
        if self.time is None:
            # handle the first measurement
            self.time = msg.header.stamp
            return
        dt = (msg.header.stamp.secs - self.time.secs) + (msg.header.stamp.nsecs - self.time.nsecs) * 10**(-9)
        self.time = msg.header.stamp
 
        # read linear and angular velocities from teh ROS message
        linear_vel = msg.twist.twist.linear.x    # velocity of the robot in its instantaneous frame. X is the forward direction
        angular_vel = msg.twist.twist.angular.z   # angular velocity about the Z axis of the robot's instantaneous frame

        # when there is no motion do not perform a prediction
        if abs(linear_vel) < 0.009 and abs(angular_vel) < 0.09:
            return

        # Simulation: we have set s_linear_vel and s_angular_vel to be proportional to the distance or angle moved
        # In real robots this can be read directly from the odometry message if it is provided
        s_linear_vel_x = std_dev_linear_vel * linear_vel * dt #  5 cm / seg 
        s_linear_vel_y = 0.000000001 # just a small value as there is no motion along y of the robot
        s_angular_vel = std_dev_angular_vel * angular_vel * dt  # 2 deg / seg

        # compute the motion command [dx, dy, dtheta] 
        # Simulation: here we add the random gaussian noise proportional to the distance travelled and angle rotated.
        # Note: this is an approximation but works as time steps are small          
        dx = linear_vel * dt + s_linear_vel_x * np.random.standard_normal()
        dy = 0.     # there is no motion along y of the robot
        dtheta = angular_vel * dt + s_angular_vel * np.random.standard_normal()

        # Calculate motion command and set it
        u = np.array([[dx], [dy], [dtheta]])
        self.motion_command = u

        # Simulation: constructing the covariance matrix from s_vel and s_omega set for the simulation
        self.motion_covariance = np.array([[(s_linear_vel_x)**2, 0.0, 0.0],
                                           [0.0, (s_linear_vel_y)**2, 0.0],
                                           [0.0, 0.0, (s_angular_vel)**2]])

        self.lock.acquire()
        self.KF.predict(self.motion_command, self.motion_covariance)
        self.lock.release()
        
        det_msg = Float32()
        det_msg.data = np.linalg.det(self.KF.getStateCovariance())
        self.pub_det.publish(det_msg)

        
    def callbackLandmarkMeasurement(self, data):
        global seenLandmarks_
        
        # For your cylinder detector, we'd suggest to use the message Landmarks_Msg, it contains an array of Landmark_Msg, 
        # each Landmark_Msg contains label, x (position), y (position), s_x (variance) and s_y (variance)

        print("received {} landmark".format(len(data.landmarks)))

        for landmark in data.landmarks:
            dx = landmark.x
            dy = landmark.y
            label = landmark.label

            new = 0
            # if seenLandmarks_ is empty
            if not seenLandmarks_:
                new = 1
                seenLandmarks_.append(label)
            # if landmark was not seen previously
            elif label not in seenLandmarks_:
                new = 1
                seenLandmarks_.append(label)      
            measurement = []
            measurement.append(dx)
            measurement.append(dy)
            measurement.append(label) 
            
            # get covariance from data received
            # Simulation: we create the covariance matrix from the s_landmark values set for simulation
            self.measurement_covariance = np.array([[landmark.s_x, 0.0],
                                           [0.0, landmark.s_y]] )       
            
            # Deal with measurement covariance close to zero
            if landmark.s_x < 10**(-4) and landmark.s_y < 10**(-4):
               print("Measurement covariance is close to zero.")
               self.measurement_covariance = np.array([[0.01,0.0], [0.0, 0.01]])   # 10 cm ??     
                
            # call KF to execute an update
            # try:
            self.lock.acquire()
            self.KF.update(np.array(measurement), self.measurement_covariance , new)
            self.lock.release()
            # except ValueError as e:
                # rospy.logwarn(e)
        

    def publish_traj(self):
        if self.time is None:
            return
        # accumulate the poses in the robot's state
        robot_pose = self.KF.getStateMean()[0:3]
        print(robot_pose.shape)
        print("Publish pose {}".format(robot_pose))
        self.traj_msg.header.seq = self.seq       # sequence ID
        self.traj_msg.header.stamp = self.time    # time stamp
        pose = PoseStamped()
        pose.pose.position.x = robot_pose[0]
        pose.pose.position.y = robot_pose[1]
        pose.pose.position.z = 0.0
        # quat = transformations.quaternion_from_euler(0.0, 0.0, robot_pose[2])
        r = R.from_euler('xyz', [0, 0, robot_pose[2]])
        quat = r.as_quat()
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.traj_msg.poses.append(pose)
        self.pose_count += 1
        if self.pose_count > 5:
            self.traj_msg.poses.pop(0)

        self.pub_traj.publish(self.traj_msg)

    def publishState(self):
        msg = Odometry()
        if self.time is None:
            return
        # Construct header
        msg.header.seq = self.seq       # sequence ID
        self.seq += 1                   # increment sequence ID for next frame
        msg.header.stamp = self.time    # time stamp
        msg.header.frame_id = 'odom'    # reference frame
        msg.child_frame_id = 'base_link'

        # Construct content
        current_pose =  np.array(self.KF.getStateMean()[0:dimR_])     # latest (x, y, yaw)
        rospy.loginfo("Current pose {}".format(current_pose))
        rospy.loginfo("Current pose z {}".format(current_pose[2]))
        msg.pose.pose.position.x = current_pose[0]
        msg.pose.pose.position.y = current_pose[1]
        msg.pose.pose.position.z = 0.0
        # quat = transformations.quaternion_from_euler(0.0, 0.0, current_pose[2])
        r = R.from_euler('xyz',[0, 0, current_pose[2]])
        quat = r.as_quat()
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        current_covariance = np.array(self.KF.getStateCovariance()[0:dimR_, 0:dimR_])   # latest 3x3 covariance
        cov = np.zeros((36, 1), dtype=np.float)
        cov[0] = current_covariance[0, 0]
        cov[1] = current_covariance[0, 1]
        cov[5] = current_covariance[0, 2]
        cov[6] = current_covariance[1, 0]
        cov[7] = current_covariance[1, 1]
        cov[11] = current_covariance[1, 2]
        cov[30] = current_covariance[2, 0]
        cov[31] = current_covariance[2, 1]
        cov[35] = current_covariance[2, 2]
        msg.pose.covariance = np.ravel(cov)

        self.pub_odom.publish(msg)

        rospy.loginfo("State mean {}".format(self.KF.getStateMean()))
        rospy.loginfo("seen landmarks {}".format(seenLandmarks_))
        # state_mean = np.array()
        Landmark_poses = self.KF.getStateMean()[dimR_:]
        print('landmark poses ',Landmark_poses)
        marker_array_msg = MarkerArray()
        for i in range(int(len(Landmark_poses)/2)): 
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.id = seenLandmarks_[i]
            marker.type = 3
            marker.action = 0
            marker.pose.position.x = Landmark_poses[2*i]
            marker.pose.position.y = Landmark_poses[2*i+1]
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.frame_locked = False
            marker.lifetime = rospy.Duration(0.1)
            marker_array_msg.markers.append(marker)

        self.pub_landmarks.publish(marker_array_msg)

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.lock = RLock()

        # Initialise robot
        self.time = None    # variable to keep current time
        self.seq = 0

        # Initialise a robot pose and covariance
        robot_pose = np.array([ [0.],[0.],[0.] ])
        robot_pose = np.reshape(robot_pose, (3,1))
        robot_covariance = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])*10**(-3)
        sense_Type = 'Vision'       
        self.robot = Robot(robot_pose, robot_covariance, sense_Type)
        
        # Initialise motion
        self.motion_command = np.array([[0.],[0.],[0.]])
        self.motion_covariance = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])     # Just initialization
        # Initialise landmark measurement
        self.measurement_covariance = np.array([[1.0,0.0],[0.0, 1.0]])*10**(-3)
        
        # Initialise kalman filter

        # Initialise a state mean and covariance
        state_mean = robot_pose
        state_covariance = robot_covariance
        
        # initial state contains initial robot pose and covariance
        self.KF = KalmanFilter(state_mean, state_covariance, self.robot)

        # Subscribe to the cylinder node output
        rospy.Subscriber('/landmarks', Landmarks_Msg, self.callbackLandmarkMeasurement, queue_size=1, buff_size=2)

        # Subscribe to odometry
        rospy.Subscriber('/odom', Odometry, self.callbackOdometryMotion)

        # Initialise a state and covariance publisher
        self.pub_odom = rospy.Publisher('/robot_slam_odometry', Odometry, queue_size=5)
        self.pub_traj = rospy.Publisher('/robot_trajectory', Path, queue_size=5)
        self.pub_landmarks = rospy.Publisher('/landmarks_state', MarkerArray, queue_size=5)
        self.pub_det = rospy.Publisher('/det', Float32, queue_size=5)

        # Initialise the trajectory message
        self.pose_count = 0 
        self.traj_msg = Path()
        self.traj_msg.header.frame_id = "odom"

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publishState()
            self.publish_traj()
            r.sleep()
        
if __name__ == '__main__':
    print('Landmark SLAM Started...')
    # Initialize the node and name it.
    rospy.init_node('listener')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        slam = SLAM()
    except rospy.ROSInterruptException: pass
