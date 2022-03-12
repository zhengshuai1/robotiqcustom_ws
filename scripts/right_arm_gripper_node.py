#!/usr/bin/env python
from __future__ import print_function

import rospy
from utils.gripper_control import GripperController
import time

def main():
    arm_name = 'right_arm'
    rospy.init_node(arm_name + "_gripper_node")
    gripper_controller = GripperController(arm_name)
    # raw_input('please press enter to start gripper control')
    rospy.loginfo(arm_name + ": start gripper control loop")
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        gripper_controller.start_control()
        r.sleep()

# def main():
#     rospy.init_node("gripper_node")
#     gripper_controller = GripperController('right_arm')
#     raw_input('please press enter to start gripper control')
#     rospy.loginfo("start gripper control loop")
#     rospy.spin()

# def main():
#     rospy.init_node("gripper_node")
#     gripper_controller = GripperController('right_arm')
#     tic = time.time()
#     gripper_controller.activate_gripper(gripper_controller.group_name)
#     print('time is', time.time()-tic)
#     rospy.sleep(2)
#     tic = time.time()
#     gripper_controller.release_gripper(gripper_controller.group_name)
#     print('time is', time.time() - tic)



if __name__ == "__main__":
    main()