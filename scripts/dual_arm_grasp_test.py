#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from utils import ros_utils
from utils.dual_arm_control import DualArmCommander
from utils.transform import Rotation, Transform

from webots_ros.srv import set_int

class NachiGraspController(object):
    """Nachi robot grasp Control"""

    def __init__(self):
        super(NachiGraspController, self).__init__()
        # self.tf_tree = ros_utils.TransformTree()
        self.mc = DualArmCommander()
        rospy.loginfo("Ready to take action")

    def execute_trajectory(self, group_name, trajectory):
        rospy.loginfo("execute trajectory")
        self.mc.goto_many_poses(group_name, trajectory)
        rospy.loginfo("finish trajectory")

    def control_gripper(self, cmd):
        group_name = 'right_arm'
        service_name = "/simulation/gripper/run"
        try:
            rospy.wait_for_service(service_name, 1)
            gripper_client = rospy.ServiceProxy(service_name, set_int)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        value = 3 if cmd == 'close' else 4
        resp = gripper_client(value)
        if not resp.success:
            rospy.logerr('execute gripper failed')
        return resp.success

    def init_pose(self):
        rospy.loginfo("go to init pose")
        r_goal = Transform.from_list([0.2, 0.2, 0.22, 1, 0, 0, 0])
        self.mc.goto_joint_pose('right_arm', r_goal)
        l_goal = Transform.from_list([0.2, -0.2, 0.22, 1, 0, 0, 0])
        self.mc.goto_joint_pose('left_arm', l_goal)
        rospy.loginfo("finish init pose")

    def run_cartesian(self):
        rospy.loginfo("start grasp loop")
        self.init_pose()
        cnt = 1

        while not rospy.is_shutdown() and cnt:
            group_name, grasp, cls = self.get_sim_grasp()
            trajectory1, trajectory2 = self.plan_trajectories(group_name, grasp, cls)
            self.execute_trajectory(group_name, trajectory1)
            self.mc.move_gripper(width=0, force=20)
            self.execute_trajectory(group_name, trajectory2)
            cnt -=1
        rospy.loginfo("finish grasp loop")

    # def simple_test(self):
    #     pose0 = Transform.from_list([0.33, 0, 0.25, 0, 1, 0, 0])
    #     pose1 = Transform.from_list([0.33, 0, 0.0, 0.70710678, -0.70710678,  0., 0.])
    #     # pose2 = Transform.from_list([0.3, -0.05, 0.03, 0, 1, 0, 0])
    #
    #     group_name = 'right_arm'
    #     trajectory2 = [pose0, pose1]
    #     self.execute_trajectory(group_name, trajectory2)
    #     self.control_gripper('close')
    #     # rospy.sleep(1)
    #     # self.control_gripper('open')
    #     pose3 = Transform.from_list([0.33, 0, 0.2, 0.70710678, -0.70710678,  0., 0.])
    #     trajectory = [pose3]
    #     self.execute_trajectory(group_name, trajectory)
    #     self.control_gripper('open')
    def simple_test(self):
        pose0 = Transform.from_list([0.33, -0.05, 0.25, 0, 1, 0, 0])
        pose1 = Transform.from_list([0.33, -0.05, 0.03, 0, 1,  0., 0.])
        # pose2 = Transform.from_list([0.3, -0.05, 0.03, 0, 1, 0, 0])

        group_name = 'right_arm'
        trajectory2 = [pose0, pose1]
        self.execute_trajectory(group_name, trajectory2)
        self.control_gripper('close')
        # rospy.sleep(1)
        # self.control_gripper('open')
        pose3 = Transform.from_list([0.33, -0.05, 0.2, 0, 1,  0., 0.])
        trajectory = [pose3]
        self.execute_trajectory(group_name, trajectory)
        self.control_gripper('open')


def main():
    rospy.init_node("nachi_dual_arm_grasp")
    print('dual arms is starting')
    arm_grasp = NachiGraspController()
    raw_input('please press enter to start grasp loop')
    # arm_grasp.run_cartesian()
    arm_grasp.simple_test()


if __name__ == "__main__":
    main()
