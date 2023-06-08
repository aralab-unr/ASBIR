#!/usr/bin/env python3
import math
import numpy as np
from geometry_msgs.msg import Quaternion

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_multiply(quaternion1, quaternion0):

    (x0, y0, z0, w0) = (quaternion0.x,quaternion0.y,quaternion0.z,quaternion0.w)
    (x1, y1, z1, w1) = (quaternion1.x,quaternion1.y,quaternion1.z,quaternion1.w)
    return Quaternion(
        x=(x1*w0 + y1*z0 - z1*y0 + w1*x0),
        y=(-x1*z0 + y1*w0 + z1*x0 + w1*y0),
        z=(x1*y0 - y1*x0 + z1*w0 + w1*z0),
        w=(-x1*x0 - y1*y0 - z1*z0 + w1*w0))