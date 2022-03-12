#!/usr/bin/env python
from __future__ import print_function
import rospy
from roport.srv import *
import numpy as np
from utils import ros_utils
from utils.dual_arm_control import DualArmCommander

def main():
    rospy.init_node("nachi_dual_arm_grasp")
    print('dual arms is starting')
    mc = DualArmCommander()
    cnt = 1
    while not rospy.is_shutdown() and cnt > 0:
        # mc.robotiq_2f_gripper_control(0.05)
        #
        # mc.robotiq_2f_gripper_control(0.0)

        mc.robotiq_vacuum_gripper_control(1)
        rospy.sleep(3)
        mc.robotiq_vacuum_gripper_control(0)
        rospy.sleep(3)
        cnt -= 1

if __name__ == "__main__":
    main()