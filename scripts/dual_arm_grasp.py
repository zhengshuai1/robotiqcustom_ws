#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from geometry_msgs.msg import PoseArray
from utils import ros_utils
from utils.transform import Rotation, Transform


class DualArmController(object):
    """DoublePandaControl"""

    def __init__(self):
        super(DualArmController, self).__init__()

    def goto_joints(self, group_name, goals, tolerance):
        service_name = "execute_group_joint_states"
        try:
            rospy.wait_for_service(service_name, 1)
            self.joints_client = rospy.ServiceProxy(service_name, ExecuteGroupJointStates)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        joints_req = ExecuteGroupJointStatesRequest()
        joints_req.group_name = group_name
        joints_req.goal = goals
        joints_req.tolerance = tolerance
        joints_resp = self.joints_client(joints_req)
        if joints_resp.result_status == joints_resp.FAILED:
            rospy.logerr('joints failed')
        return joints_resp.result_status

    def goto_all_joint_poses(self, group_name, left_pose, right_pose):
        service_name = "execute_all_joint_poses"
        try:
            rospy.wait_for_service(service_name, 1)
            self.joints_client = rospy.ServiceProxy(service_name, ExecuteAllJointPoses)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        all_joints_req = ExecuteAllJointPosesRequest()
        all_joints_req.group_name = group_name
        left_pose = Transform.from_list(left_pose)
        left_pose = ros_utils.to_pose_msg(left_pose)
        right_pose = Transform.from_list(right_pose)
        right_pose = ros_utils.to_pose_msg(right_pose)
        all_joints_req.goals.poses = [left_pose, right_pose]
        joints_resp = self.joints_client(all_joints_req)
        if joints_resp.result_status == joints_resp.FAILED:
            rospy.logerr('joints failed')
            return False
        return True

    def goto_all_poses(self, group_names, left_pose, right_pose):
        service_name = "execute_all_poses"
        # service_name = "execute_all_poses_py"
        try:
            rospy.wait_for_service(service_name, 1)
            self.all_poses_client = rospy.ServiceProxy(service_name, ExecuteAllPoses)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        all_joints_req = ExecuteAllPosesRequest()
        all_joints_req.group_names = [group_names[0], group_names[1]]
        left_pose = Transform.from_list(left_pose)
        left_pose = ros_utils.to_pose_msg(left_pose)
        right_pose = Transform.from_list(right_pose)
        right_pose = ros_utils.to_pose_msg(right_pose)
        all_joints_req.goals.poses = [left_pose, right_pose]
        all_joints_req.stamps = [4, 4]
        joints_resp = self.all_poses_client(all_joints_req)
        if joints_resp.result_status == joints_resp.FAILED:
            rospy.logerr('joints failed')
            return False
        return True

    def run(self):
        rospy.loginfo("start")

        goals = np.pi / 180. * np.array([0, 30, -20, 0, -90, 0])
        # self.goto_joints('left_arm', goals, 0.01)
        # self.goto_joints('right_arm', goals, 0.01)
        all_goals = np.pi / 180. * np.array([0, 30, -20, 0, -90, 0, 0, 30, -20, 0, -90, 0])
        self.goto_joints('dual_arm', all_goals, 0.01)
        # rospy.loginfo("two groups")
        # self.goto_all_poses(['left_arm', 'right_arm'], [0.1, -0.1, 0.1, 0, 1, 0, 0], [-0.1, 0.1, 0.1, 0, 1, 0, 0])
        # self.goto_all_poses(["left_arm", "right_arm"], [0.1, -0.1, 0.05, 0, 1, 0, 0], [-0.1, 0.1, 0.05, 0, 1, 0, 0])
        # self.goto_joints('dual_arm', all_goals, 0.01)
        rospy.loginfo("one group")
        self.goto_all_joint_poses("dual_arm", [0.1, -0.1, 0.1, 0, 1, 0, 0], [-0.1, 0.1, 0.1, 0, 1, 0, 0])
        self.goto_all_joint_poses("dual_arm", [0.1, -0.1, 0.05, 0, 1, 0, 0], [-0.1, 0.1, 0.05, 0, 1, 0, 0])
        self.goto_joints('dual_arm', all_goals, 0.01)


        rospy.loginfo("end")


def main():

    rospy.init_node("dual_arm_grasp")
    print('dual arm is starting')
    dualarm_grasp = DualArmController()
    dualarm_grasp.run()


if __name__ == "__main__":
    main()
