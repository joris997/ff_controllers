#!/usr/bin/env python3

import math
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_matrix

class HJI_controller():
    def __init__(self):
        self.x0 = np.zeros((13,1))

    def solve(self, x, x_des, verbose=False):
        # Use the BRT to compute the control based on the current state and the value function
        return np.array([0,0,0]), np.array([0,0,0])
        
    def prepare_controller(self, x, x_des):
        # Now we need to compute the backwards reachable tube for the HJI controller
        return True