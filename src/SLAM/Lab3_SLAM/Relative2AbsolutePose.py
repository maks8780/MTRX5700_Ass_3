# -*- coding: utf-8 -*-
"""
Created on Tue Aug 9 16:12:08 2016
Calculate the robot pose given the robot previous pose and motion of the robot
with respect to that pose
@author: admin-u5941570
Updated on Tue Apr 07 2020
@maintainers: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams,
Stephany Berrio Perez, Tejaswi Digumarti
"""

import numpy as np
from pi2pi import pi2pi


def Relative2AbsolutePose(robot_pose_abs, u_rel):
    """
    Calculates the new pose of the robot given its current pose in the
    absolute coordinate frame and a motion input.

    :param robot_pose_abs: current pose of the robot in the absolute reference
    frame [x, y, theta]
    :type robot_pose_abs: [type]
    :param u_rel: motion command in the robot's frame of reference
    [dx, dy, dtheta]
    :type u_rel: [type]
    :return: pose of the robot in the absolute reference frame after applying
    the motion and the Jacobians of the new pose wrt, the current pose
    and the motion command respectively
    :rtype: tuple
    """
    x1 = robot_pose_abs[0][0]
    y1 = robot_pose_abs[1][0]
    theta1 = robot_pose_abs[2][0]
    dx = u_rel[0][0]
    dy = u_rel[1][0]
    dtheta = u_rel[2][0]

    # R is the transition matrix of robot frame
    # i.e. X_t+1 = X_t + R(theta_t) * u
    R = [[np.cos(theta1), -np.sin(theta1), 0],
         [np.sin(theta1), np.cos(theta1), 0],
         [0, 0, 1]]

    next_robot_pose_abs = np.dot(R, u_rel) + robot_pose_abs
    next_robot_pose_abs[2][0] = pi2pi(next_robot_pose_abs[2][0])

    # Calculate Jacobian of X_t+1 with respect to the current robot pose X_t
    H1 = [[1, 0, -dx*np.sin(theta1)-dy*np.cos(theta1)],
          [0, 1,  dx*np.cos(theta1)-dy*np.sin(theta1)],
          [0, 0, 1]]

    # Calculate Jacobian of X_t+1 with respect to motion command u
    H2 = [[np.cos(theta1), -np.sin(theta1), 0],
          [np.sin(theta1), np.cos(theta1), 0],
          [0, 0, 1]]

    assert(next_robot_pose_abs.shape == (3,1))

    return next_robot_pose_abs, H1, H2


if __name__ == "__main__":
    """
    For self testing only
    """
    input_robot_pose_abs = [2, 1, np.pi / 4.0]
    motion_rel = [0, 2, 0]
    (output_next_robot_abs,
        jac_h1, jac_h2) = Relative2AbsolutePose(input_robot_pose_abs,
                                                motion_rel)
    print(jac_h1)
