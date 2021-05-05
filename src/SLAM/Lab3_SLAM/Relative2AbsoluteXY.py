# -*- coding: utf-8 -*-
"""
Created on Thu Aug 11 10:53:05 2016
@author: admin-u5941570
Updated on Tue Apr 07 2020
@maintainers: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams, Tejaswi Digumarti
"""

import numpy as np


def Relative2AbsoluteXY(robot_pose_abs, landmark_position_rel):
    """
    Convert's a landmark's position from the robot's frame of reference to the absolute frame of reference
    :param robot_pose_abs: pose of the robot in the the absolute frame of reference [x, y, theta]
    :param landmark_position_rel: position of the landmark in the robot's frame of reference [x, y]
    :return : [position of the landmark in the absolute frame of reference [x, y], H1, H2]
    """
    x1 = robot_pose_abs[0][0]
    y1 = robot_pose_abs[1][0]
    theta1 = robot_pose_abs[2][0]
    x2 = landmark_position_rel[0]
    y2 = landmark_position_rel[1]

    
    landmark_position_rel_vec = [[x2], [y2], [1]]
    
    # R is the transition matrix to robot frame
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]
         
    # Calculate Jacobian H1 with respect to X1
    H1 = [[1, 0, -x2*np.sin(theta1)-y2*np.cos(theta1)],
          [0, 1,  x2*np.cos(theta1)-y2*np.sin(theta1)]]
         
    # Calculate Jacobian H2 with respect to X2
    H2 = [[np.cos(theta1), -np.sin(theta1)],
          [np.sin(theta1),  np.cos(theta1)]]
         
    landmark_abs = np.array(np.dot(R, landmark_position_rel_vec)) + np.array(robot_pose_abs)
    
    return [landmark_abs[0][0], landmark_abs[1][0]], H1, H2


if __name__ == "__main__":
    """
    For self testing only
    """
    in_robot_pose_abs = [[2], [1], [np.pi / 4.0]]
    in_landmark_positions_rel = [1.4142, 1.4142]
    out_landmark_position_abs, jac_h1, jac_h2 = Relative2AbsoluteXY(in_robot_pose_abs, in_landmark_positions_rel)
    print(out_landmark_position_abs)
