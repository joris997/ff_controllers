#!/usr/bin/env python3

import math
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_matrix

class PID_controller():
    def __init__(self):
        self.x0 = np.zeros((13,1))

        self.Px = np.diag([.1, .1, .1])
        self.Dx = np.diag([1, 1, 1])

        self.Pq = np.diag([.1, .1, .1])
        self.Dq = np.diag([1, 1, 1])

    def solve(self, x, xdes, verbose=False):
        # PD terms
        xdiff = x[0:3] - xdes[0:3]
        xdiffdot = x[3:6] - xdes[3:6]

        qdiff = euler_from_quaternion(x[6:10]) - euler_from_quaternion(xdes[6:10])
        qdiffdot = x[10:13] - xdes[10:13]

        force_world  = -self.Px@(xdiff) - self.Dx@(xdiffdot)
        torque_world = -self.Pq@(qdiff) - self.Dq@(qdiffdot)
        if verbose:
            print("\n---")
            print(f"x error: {x - x_des}")
            print(f"dx error: {dx - dx_des}")
            print("---")
            print(f"q error: {q - q_des}")
            print(f"dq error: {dq - dq_des}")

        force_body, torque_body = self.convert_to_body_frame(force_world,torque_world)
        return force_body, torque_body

    def prepare_controller(self, x, x_des):
        # We have nothing to prepare for now, so we just return True
        return True
        
    def convert_to_body_frame(self, force, torque):
        # Convert force and torque to body frame
        t = self.tf_buffer.lookup_transform('snap', 'map', rclpy.time.Time())

        translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        quaternion_matrix = quaternion_matrix([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
        rotation_matrix = quaternion_matrix[:3,:3]

        t_force = np.dot(rotation_matrix, force)
        t_torque = np.dot(rotation_matrix, torque)

        return t_force, t_torque
