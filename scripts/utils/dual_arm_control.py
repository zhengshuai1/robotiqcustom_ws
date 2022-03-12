import actionlib
import control_msgs.msg
import rospy
import numpy as np
from roport.srv import *
from utils import ros_utils
from robotiq_msgs.msg import CModelCommandAction, CModelCommandGoal


class DualArmCommander(object):
    def __init__(self):

        rospy.loginfo("DualArmCommander ready")

    def home(self, group_name):
        goals = np.pi / 180. * np.array([0, 30, -30, 0, -90, 0])
        self.goto_joints(group_name, goals)
        rospy.loginfo("finish home")

    def goto_joints(self, group_name, goals, tolerance=0.01):
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
            rospy.logerr('execute joints failed')
        return joints_resp.result_status

    def goto_joint_pose(self, group_name, goal, goal_type=0, tolerance=0.01):
        service_name = "execute_group_pose"
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

    def goto_many_poses(self, group_name, goals, goal_type=0, eef_step=0.06):
        service_name = "execute_group_many_poses"
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
        poses_req.tolerance = 0.1
        poses_req.goal_type = goal_type
        poses_req.eef_step = eef_step
        poses_resp = self.poses_client(poses_req)
        if poses_resp.result_status == poses_resp.FAILED:
            rospy.logerr('execute many cartesian poses failed')
        return poses_resp.result_status

    def goto_all_poses_cpp(self, group_names, left_pose, right_pose):
        service_name = "execute_all_poses"
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

    def goto_all_joint_poses(self, goals, goal_type=0):
        """goals =[left arm, right arm]"""
        service_name = "execute_all_joint_poses"
        group_name = 'dual_arm'
        try:
            rospy.wait_for_service(service_name, 1)
            client = rospy.ServiceProxy(service_name, ExecuteAllJointPoses)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        req = ExecuteAllJointPosesRequest()
        req.group_name = group_name
        req.goals = ros_utils.to_posearray_msg(goals)
        req.goal_type = goal_type
        resp = client(req)
        if resp.result_status == resp.FAILED:
            rospy.logerr('execute both joint pose failed')
            return False
        return True

    def add_collision_box(self, group_name, box_name, box_pose, box_size, is_absolute=True):
        service_name = "execute_add_collision_box"
        try:
            rospy.wait_for_service(service_name, 1)
            acb_client = rospy.ServiceProxy(service_name, ExecuteAddCollisionBox)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        req = ExecuteAddCollisionBoxRequest()
        req.group_name = group_name
        req.box_name = box_name
        req.box_pose = ros_utils.to_pose_msg(box_pose)
        req.box_size = ros_utils.to_point_msg(box_size)
        req.is_absolute = is_absolute
        resp = acb_client(req)
        if resp.result_status == resp.FAILED:
            rospy.logerr('add collision box failed')
        return resp.result_status

    def add_collision_plane(self, group_name, plane_name, plane_pose, plane_normal):
        service_name = "execute_add_collision_plane"
        try:
            rospy.wait_for_service(service_name, 1)
            client = rospy.ServiceProxy(service_name, ExecuteAddCollisionBox)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        req = ExecuteAddCollisionPlaneRequest()
        req.group_name = group_name
        req.plane_name = plane_name
        req.plane_pose = ros_utils.to_pose_msg(plane_pose)
        req.plane_normal = ros_utils.to_point_msg(plane_normal)
        resp = client(req)
        if resp.result_status == resp.FAILED:
            rospy.logerr('add collision plane failed')
        return resp.result_status

    def move_gripper(self, width=0.0, force=20.0):
        rospy.loginfo("gripper start")
        rospy.sleep(0.3)
        rospy.loginfo("gripper finish")

    def move_group_gripper(self, group_name, width=0.0, force=20.0):
        rospy.loginfo("gripper start")
        rospy.sleep(0.3)
        rospy.loginfo("gripper finish")





