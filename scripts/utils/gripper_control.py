#!/usr/bin/env python

import actionlib
import control_msgs.msg
import rospy
from robotiq_msgs.msg import CModelCommandAction, CModelCommandGoal
from std_srvs.srv import SetBool, SetBoolResponse
import time


class GripperController(object):
    def __init__(self, group_name):
        super(GripperController, self).__init__()
        self.gripper_running_state = 'idle'
        self.group_name = group_name
        self.srv_id = '/' + self.group_name + '/gripper_control'
        self.srv_gripper_control = rospy.Service(self.srv_id, SetBool, self.gripper_control_handle)
        self.gripper_width = 0.035
        if self.group_name == 'right_arm':
            self.robotiq_2f_gripper_client = actionlib.SimpleActionClient(
                "/robotiq_2f_gripper/gripper_action", control_msgs.msg.GripperCommandAction)
            self.robotiq_2f_gripper_client.wait_for_server()
            rospy.loginfo("Connected to 2f_gripper action server")
        else:
            self.robotiq_vacuum_gripper_client = actionlib.SimpleActionClient(
                "/robotiq_vacuum_gripper/gripper_action", CModelCommandAction)
            self.robotiq_vacuum_gripper_client.wait_for_server()
            rospy.loginfo("Connected to vacuum_gripper action server")

    def gripper_control_handle(self, req):
        resp = SetBoolResponse()
        if req.data:
            self.gripper_running_state = 'run'
            # self.activate_gripper(self.group_name)
            # self.gripper_running_state = 'idle'
        else:
            self.gripper_running_state = 'stop'
            # self.release_gripper(self.group_name)
            # self.gripper_running_state = 'idle'
        self.tic = time.time()
        resp.success = True
        resp.message = 'success'
        return resp

    def activate_gripper(self, group_name):
        if group_name == 'right_arm':
            self.robotiq_2f_gripper_control(0, max_effort=20)  # grasp
        else:
            self.robotiq_vacuum_gripper_control(1, mod=1)  # grasp

    def release_gripper(self, group_name):
        if group_name == 'right_arm':
            self.robotiq_2f_gripper_control(0.035, max_effort=20)  # half width
        else:
            self.robotiq_vacuum_gripper_control(0)

    def robotiq_2f_gripper_control(self, width, max_effort=20.0):
        cmd = control_msgs.msg.GripperCommand(width, max_effort)
        goal = control_msgs.msg.GripperCommandGoal(cmd)
        self.robotiq_2f_gripper_client.send_goal(goal)
        return self.robotiq_2f_gripper_client.wait_for_result(rospy.Duration(0.4))

    def robotiq_vacuum_gripper_control(self, grasp=0, mod=0):
        # grasp: 1 release: 0
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

    def start_control(self):
        if self.gripper_running_state == 'run':
            self.activate_gripper(self.group_name)
            # print('time is', time.time() - self.tic)
            self.gripper_running_state = 'idle'
        if self.gripper_running_state == 'stop':
            self.release_gripper(self.group_name)
            # print('time is', time.time() - self.tic)
            self.gripper_running_state = 'idle'
