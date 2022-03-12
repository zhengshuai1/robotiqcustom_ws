#!/usr/bin/env python
from __future__ import print_function

import rospy
from utils.gripper_control import GripperController


def main():
    arm_name = 'left_arm'
    rospy.init_node(arm_name + "_gripper_node")
    gripper_controller = GripperController(arm_name)
    rospy.loginfo(arm_name + ": start gripper control loop")
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        gripper_controller.start_control()
        r.sleep()


if __name__ == "__main__":
    main()