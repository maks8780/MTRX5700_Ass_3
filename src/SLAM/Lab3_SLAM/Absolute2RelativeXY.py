# -*- coding: utf-8 -*-
"""
Created on Tue Jan 12 15:08:15 2016
@author: admin-u5072689
Updated on Tue Apr 15 2021
@contributors: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams,
Stephany Berrio Perez, Tejaswi Digumarti
"""

import numpy as np


def Absolute2RelativeXY(robot_pose_abs, landmark_position_abs):
    """Converts a landmark's position from the absolute frame of reference to
    the robot's frame of reference, i.e the position of the landmarks as
    measured by the robot.

    :param robot_pose_abs: pose of the robot in the absolute frame of
    reference [x, y, theta]. 3x1 array
    :type robot_pose_abs: np.array
    :param landmark_position_abs: position of the landmark in the absolute 
    frame of reference [x, y]. 2x1 array
    :type landmark_position_abs: np.array
    :return: position of the landmark in the to the robot's frame of reference
    [x, y], and the Jacobians of
    :rtype: tuple
    """
    x1 = robot_pose_abs[0][0]
    y1 = robot_pose_abs[1][0]
    theta1 = robot_pose_abs[2][0]
    x2 = landmark_position_abs[0]
    y2 = landmark_position_abs[1]

    # Calculate the difference with respect to world frame
    diff = [[x2-x1],
            [y2-y1],
            [1]]

    # R is the transition matrix to robot frame
    R = [[np.cos(-theta1), -np.sin(-theta1), 0],
         [np.sin(-theta1), np.cos(-theta1), 0],
         [0, 0, 1]]

    landmark_position_rel = np.dot(R, diff)

    # Calculate Jacobian of the relative landmark position wrt. the robot pose,
    # i.e. [x1, y1, theta1]
    H1 = [[-np.cos(theta1), -np.sin(theta1), -(x2-x1)*np.sin(theta1)+(y2-y1)*np.cos(theta1)],
          [np.sin(theta1), -np.cos(theta1), -(x2-x1)*np.cos(theta1)-(y2-y1)*np.sin(theta1)]]

    # Calculate Jacobian of the relative landmark position wrt. the absolute
    # landmark pose. i.e. [x2, y2]
    H2 = [[np.cos(theta1), np.sin(theta1)],
          [-np.sin(theta1), np.cos(theta1)]]

    return [[landmark_position_rel[0][0]], [landmark_position_rel[1][0]]], H1, H2


if __name__ == "__main__":
    """
    For testing only
    """

    in_robot_pose_abs = [2, 1, np.pi / 4]
    in_landmark_position_abs = [2, 3]
    out_landmark_position_rel, jac_h1, jac_h2 = Absolute2RelativeXY(in_robot_pose_abs, in_landmark_position_abs)
    print(out_landmark_position_rel)
