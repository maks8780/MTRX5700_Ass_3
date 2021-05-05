# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 12:35:08 2016
@author: admin-u5941570
Updated on Tue Apr 07 2020
@maintainers: Viorela Ila, Max Revay, Jing Cheng, Stefan Williams, Tejaswi Digumarti
"""

import numpy as np


def pi2pi(angle):
    """
    Maps angle to the range of [-pi, pi]
    :param angle: then angle that needs to be mapped to the range [-pi, pi]
    :return : angle in the range [-pi, pi]
    """
    dp = 2*np.pi
    if angle <= -dp or angle >= dp:
        angle = angle % dp
    if angle >= np.pi:
        angle = angle - dp
    if angle <= -np.pi:
        angle = angle + dp
    return angle


if __name__ == "__main__":
    """
    For self testing only
    """
    ang = 5.13
    ang_pi2pi = pi2pi(ang)
    print(ang_pi2pi)
