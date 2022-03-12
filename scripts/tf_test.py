#!/usr/bin/env python
from __future__ import print_function


import rospy
# from roport.srv import *
import numpy as np

from utils import ros_utils
from utils.transform import Rotation, Transform
import time

def main():
    rospy.init_node("nachi_arm_grasp")
    tf = ros_utils.TransformTree()
    ori = Rotation.from_euler('XYZ', [180, 30, 60], degrees=True)
    rot_trans = Rotation.from_euler('X', 20, degrees=True)
    ori = ori * rot_trans
    ori = ori.as_dcm()
    z_vector = ori[:, 2]
    angle = np.pi/3
    x_vector = compute_rotation(angle, z_vector)

    y_vector = np.cross(z_vector, x_vector)
    print('result is', np.inner(x_vector, z_vector), np.inner(x_vector, y_vector))
    grasp_ori = np.zeros((3, 3))
    grasp_ori[:, 0] = x_vector
    grasp_ori[:, 1] = y_vector
    grasp_ori[:, 2] = z_vector
    print('ori is', ori, 'grasp ori is', grasp_ori)
    grasp_rot = Rotation.from_dcm(grasp_ori)
    x_vector = grasp_rot.as_dcm()[:, 0]
    z_vector = grasp_rot.as_dcm()[:, 2]
    y_vector = grasp_rot.as_dcm()[:, 1]
    print('result is', np.inner(x_vector, z_vector), np.inner(x_vector, y_vector))
    T1 = Transform(grasp_rot, [1, 1, 1])
    T2 = Transform(Rotation.from_dcm(ori), [1, 1.2, 1])

    pose1 = Transform.identity()

    pose2 = Transform.from_list([0.2, 0.2, 0.1, 0, 1, 0, 0])

    while not rospy.is_shutdown():
        # tf.broadcast_static(T1, 'base', 'grasp')
        # tf.broadcast_static(T2, 'base', 'truth_grasp')
        tf.broadcast_static(pose1, 'base', 'pose1')
        tf.broadcast_static(pose2, 'base', 'pose2')
        rospy.sleep(1)

# def compute_rotation(angle, z_vector):
#     # angle is not pi/2
#     if not np.tan(angle) is np.inf:
#         x_vector = np.zeros(3)
#         if angle > 0:
#             factor = 1 / np.cos(angle) ** 2 + (z_vector[0] + np.tan(angle) * z_vector[1]) ** 2 / z_vector[2] ** 2
#             x_vector[0] = np.sqrt(1 / factor)
#             x_vector[1] = -x_vector[0]*np.tan(angle)
#             # x_vector[2] = np.sqrt(1-(x_vector[0]/np.cos(angle))**2)
#             x_vector[2] = -x_vector[0] * (z_vector[0] - np.tan(angle) * z_vector[1]) / z_vector[2]
#         else:
#
#             factor = 1 / np.cos(angle) ** 2 + (z_vector[0] - np.tan(angle) * z_vector[1]) ** 2 / z_vector[2] ** 2
#             x_vector[0] = np.sqrt(1 / factor)
#             x_vector[1] = x_vector[0] * np.tan(angle)
#             # x_vector[2] = np.sqrt(1-(x_vector[0]/np.cos(angle))**2)
#             x_vector[2] = -x_vector[0] * (z_vector[0] + np.tan(angle) * z_vector[1]) / z_vector[2]
#
#         return x_vector
#     else:
#         raise RuntimeError

def compute_rotation(angle, z_vector):
    # angle is not pi/2
    if not np.tan(angle) is np.inf:
        x_vector = np.zeros(3)
        angle_mid = np.arctan(z_vector[0]/z_vector[2])
        # angle_mid = np.arctan2(z_vector[0], z_vector[2])
        if angle > 0:
            x_vector[1] = -np.sin(angle)
            theta2 = np.arcsin(np.tan(angle) * z_vector[1]/np.sqrt(z_vector[0]**2+z_vector[2]**2)) - angle_mid
            # if np.pi / 2 < theta2 <= np.pi:
            #     theta2 = theta2 - np.pi
            # elif -np.pi <= theta2 <= -np.pi / 2:
            #     theta2 = theta2 + np.pi
            print('angle_mid is', angle_mid, 'theta2 is', theta2/np.pi*180)
            # theta2 = np.arccos(np.inner(z_vector, [0, 0, -1]))
            x_vector[0] = np.cos(angle) * np.cos(theta2)
            x_vector[2] = np.cos(angle) * np.sin(theta2)

        else:
            x_vector[1] = -np.sin(angle)
            theta2 = np.arcsin(np.tan(angle)*z_vector[1]/np.sqrt(z_vector[0]**2+z_vector[2]**2))-angle_mid
            # if np.pi / 2 < theta2 <= np.pi:
            #     theta2 = theta2 - np.pi
            # elif -np.pi <= theta2 <= -np.pi / 2:
            #     theta2 = theta2 + np.pi
            print('angle_mid is', angle_mid, 'theta2 is', theta2/np.pi*180)
            x_vector[0] = np.cos(angle)*np.cos(theta2)
            x_vector[2] = np.cos(angle) * np.sin(theta2)

        return x_vector
    else:
        raise RuntimeError

if __name__ == "__main__":
    main()