"""
Created on Mon Aug 29 13:36:08 2016
@author: admin-u5941570
Updated on Tue Apr 07 2020
@maintainers: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams, Tejaswi Digumarti
"""

import numpy as np


def RelativeLandmarkPositions(landmark_position_abs, next_landmark_position_abs):
    """
    Given two input landmark positions in the absolute frame of reference, computes the relative position of the
    next landmark with respect to the current landmark
    :param landmark_position_abs: position of the current landmark in the absolute reference frame [x, y]
    :param next_landmark_position_abs: position of the next landmark in the absolute reference frame [x, y]
    :return : relative position of the next landmark with respect to the current landmark's position [dx, dy]
    """
    # label is in position [0], hence use positions [1] and [2]
    x1 = float(landmark_position_abs[1])
    y1 = float(landmark_position_abs[2])
    x2 = float(next_landmark_position_abs[1])
    y2 = float(next_landmark_position_abs[2])
    
    # Calculate the difference of position in world frame
    diff = [x2-x1, y2-y1]
    
    return diff
