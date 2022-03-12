import rospy
import numpy as np
from roport.srv import *
from utils import ros_utils


class Mz04Commander(object):
    def __init__(self):
        rospy.loginfo("Mz04Commander ready")

    def home(self):
        goals = np.pi / 180. * np.array([0, 30, -30, 0, -90, 0])
        self.goto_joints(goals)
        rospy.loginfo("finish home")

    def goto_joints(self, goals, tolerance=0.01):
        service_name = "execute_group_joint_states"
        group_name = 'arm'
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
            rospy.logerr('execute joints failed')
        return joints_resp.result_status

    def goto_joint_pose(self, goal, goal_type=0, tolerance=0.05):
        service_name = "execute_group_pose"
        group_name = 'arm'
        try:
            rospy.wait_for_service(service_name, 1)
            jp_client = rospy.ServiceProxy(service_name, ExecuteGroupPose)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        jp_req = ExecuteGroupPoseRequest()
        jp_req.group_name = group_name
        goal_pose = ros_utils.to_pose_msg(goal)
        jp_req.goal = goal_pose
        jp_req.goal_type = goal_type
        jp_req.tolerance = tolerance
        jp_resp = jp_client(jp_req)
        if jp_resp.result_status == jp_resp.FAILED:
            rospy.logerr('execute pose failed')
        return jp_resp.result_status

    def goto_many_poses(self, goals, goal_type=0, eef_step=0.09):
        service_name = "execute_group_many_poses"
        group_name = 'arm'
        try:
            rospy.wait_for_service(service_name, 1)
            self.poses_client = rospy.ServiceProxy(service_name, ExecuteGroupManyPoses)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        poses_req = ExecuteGroupManyPosesRequest()
        poses_req.group_name = group_name
        goal_pose = ros_utils.to_posearray_msg(goals)
        poses_req.goals = goal_pose
        poses_req.goal_type = goal_type
        poses_req.eef_step = eef_step
        poses_resp = self.poses_client(poses_req)
        if poses_resp.result_status == poses_resp.FAILED:
            rospy.logerr('execute many cartesian poses failed')
        return poses_resp.result_status

    def move_gripper(self, width=0.0, force=20.0):
        rospy.loginfo("gripper start")
        rospy.sleep(0.3)
        rospy.loginfo("gripper finish")