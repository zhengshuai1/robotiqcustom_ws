import actionlib
import control_msgs.msg
import rospy
import numpy as np
from roport.srv import *
from utils import ros_utils
from robotiq_msgs.msg import CModelCommandAction, CModelCommandGoal
####
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from std_msgs.msg import Header


class DualArmCommander(object):
    def __init__(self):

        rospy.loginfo("DualArmCommander ready")

    def home(self, group_name):
        goals = np.pi / 180. * np.array([0, 30, -30, 0, -90, 0])
        self.goto_joints(group_name, goals)
        rospy.loginfo("finish home")

    def goto_joints(self, group_name, goals, tolerance=0.01):
        if group_name == 'left_arm':
            service_name = "/left_arm/execute_group_joint_states"
        else:
            service_name = "/right_arm/execute_group_joint_states"
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
        if group_name == 'left_arm':
            service_name = "/left_arm/execute_group_pose"
        else:
            service_name = "/right_arm/execute_group_pose"
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

    def goto_many_poses(self, group_name, goals, goal_type=0, eef_step=0.1):
        if group_name == 'left_arm':
            service_name = "/left_arm/execute_group_many_poses"
        else:
            service_name = "/right_arm/execute_group_many_poses"
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

    def build_cart_path(self, group_name, goals, goal_type=0, eef_step=1):
        if group_name == 'left_arm':
            service_name = "/left_arm/build_cart_path"
        else:
            service_name = "/right_arm/build_cart_path"
        try:
            rospy.wait_for_service(service_name, 1)
            self.poses_client = rospy.ServiceProxy(service_name, BuildCartPath)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        poses_req = BuildCartPathRequest()
        poses_req.group_name = group_name
        goal_pose = ros_utils.to_posearray_msg(goals)
        poses_req.goals = goal_pose
        poses_req.tolerance = 0.1
        poses_req.goal_type = goal_type
        poses_req.eef_step = eef_step
        poses_resp = self.poses_client(poses_req)
        if poses_resp.result_status == poses_resp.FAILED:
            rospy.logerr('execute many cartesian poses failed')
        return poses_resp.plan

    def add_collision_box(self, group_name, box_name, box_pose, box_size, is_absolute=True):
        if group_name == 'left_arm':
            service_name = "/left_arm/execute_add_collision_box"
        else:
            service_name = "/right_arm/execute_add_collision_box"
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

    def moveit_fk(self, group_name, joint_positions):

        rospy.wait_for_service('/left_arm/compute_fk', 1)
        try:
            moveit_fk = rospy.ServiceProxy('/left_arm/compute_fk', GetPositionFK)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s" % e)
        fk_link_names = ['hand_L_ee_link']
        joint_names = []
        for i in range(6):
            joint_names.append('arm_L_joint' + str(i+1))  # your names may vary
        header = Header(0, rospy.Time.now(), "arm_L_link0")
        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = joint_positions
        resp = moveit_fk(header, fk_link_names, rs)
        pose = resp.pose_stamped[0].pose
        # rospy.loginfo(["FK LOOKUP:", pose])  # Lookup the pose
        return pose

    def robotiq_2f_gripper_control(self, width, max_effort=20.0):
        self.robotiq_2f_gripper_client = actionlib.SimpleActionClient(
            "/robotiq_2f_gripper/gripper_action", control_msgs.msg.GripperCommandAction)
        self.robotiq_2f_gripper_client.wait_for_server()
        # rospy.loginfo("Connected to 2f_gripper action server")
        cmd = control_msgs.msg.GripperCommand(width, max_effort)
        goal = control_msgs.msg.GripperCommandGoal(cmd)
        self.robotiq_2f_gripper_client.send_goal(goal)
        return self.robotiq_2f_gripper_client.wait_for_result(rospy.Duration(0.4))

    def robotiq_vacuum_gripper_control(self, grasp=0, mod=0):
        # grasp: 1 release: 0
        self.robotiq_vacuum_gripper_client = actionlib.SimpleActionClient(
            "/robotiq_vacuum_gripper/gripper_action", CModelCommandAction)
        self.robotiq_vacuum_gripper_client.wait_for_server()
        # rospy.loginfo("Connected to vacuum_gripper action server")
        if grasp == 0:
            pos = 255
        elif grasp == 1:
            pos = 0
        else:
            rospy.logerr('unkown cmd for vacuum gripper')
            return
        goal = CModelCommandGoal(mod, pos, 0, 0)
        self.robotiq_vacuum_gripper_client.send_goal(goal)
        return self.robotiq_vacuum_gripper_client.wait_for_result(rospy.Duration(0.25))

    def robot_go(self, group_name, trajectory_lengths, points):
        if group_name == 'left_arm':
            service_name = "/left_arm/nachi_follow_joint_trajectory"
        else:
            service_name = "/right_arm/nachi_follow_joint_trajectory"
        try:
            rospy.wait_for_service(service_name, 1)
            client = rospy.ServiceProxy(service_name, nachiFollowJointTrajectory)
        except rospy.ROSException:
            rospy.logwarn('Service ' + service_name + ' not available')
            return None
        req = nachiFollowJointTrajectoryRequest()
        req.group_name = group_name
        req.trajectory_lengths = trajectory_lengths
        req.points = points
        resp = client(req)
        if resp.result_status == resp.FAILED:
            rospy.logerr('nachi_follow_joint_trajectory failed')
        return resp.result_status




