#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from utils import ros_utils
from utils.dual_arm_control import DualArmCommander
from utils.transform import Rotation, Transform
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
from segmentation_srv.srv import Segmentation, TopkSegmentation
from geometry_msgs.msg import PoseStamped, PoseArray
import time
from segmentation_srv.msg import TopkSeg


class SegmentationPipe(object):
    def __init__(self):
        super(SegmentationPipe, self).__init__()
        print('wait for seg')
        rospy.wait_for_service('Segmentation')
        print('wait for seg end')
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_listener,)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_listener)
        self.service = rospy.ServiceProxy('Segmentation', TopkSegmentation)
        self.pub = rospy.Publisher('/detected_objects_in_image', PoseStamped, queue_size=10)
        self.topk_pub = rospy.Publisher('/topk_detected_objects_in_image', PoseArray, queue_size=2)
        self.state = False
        self.depth = None
        self.color = None

    def segmentation_client(self):
        try:
            pose = self.service(self.color, self.depth)
            return pose
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def color_listener(self, data):
        if not self.state:
            return
        self.color = data

    def depth_listener(self, data):
        if not self.state:
            return
        self.depth = data

    def do_segmentation(self):
        self.state = True
        rospy.sleep(0.08)
        while not (self.color and self.depth):
            rospy.logerr('wait loop')
            rospy.sleep(0.025)
        if self.color and self.depth:
            t1 = time.time()
            resp = self.segmentation_client()
            t2 = time.time()
            print('time is ', t2-t1)
            # pose_s = PoseStamped()
            # pose_s.header.frame_id = "camera_color_optical_frame"
            # pose_s.header.stamp = rospy.Time(0)
            # pose_s.pose = resp.topk_grasps.poses[0]
            # self.pub.publish(pose_s)
            resp.topk_grasps.header.frame_id = "camera_color_optical_frame"
            self.topk_pub.publish(resp.topk_grasps)
            rospy.logerr('end sense')
            self.state = False

            return resp

    def start(self):
        # rospy.Subscriber("/camera/color/image_raw", Image, self.color_listener)
        # rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_listener)
        rospy.logerr('start sense')

        # self.color = rospy.wait_for_message("/camera/color/image_raw", Image)
        # self.depth = rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
        # rospy.logerr('finish capture color and depth')
        seg_resp = self.do_segmentation()
        return seg_resp

class NachiGraspController(object):
    """Nachi robot grasp Control"""

    def __init__(self):
        super(NachiGraspController, self).__init__()
        # self.tf_tree = ros_utils.TransformTree()
        self.mc = DualArmCommander()

        self.T_rbase_camera = Transform.from_list(
            [0.440856, -0.0448193, 0.794483, -0.699114, 0.714996, -0.00406471, -0.00212906])
        d = 0.9306831
        self.T_lbase_rbase = Transform.from_euler_list([d, 0, 0, np.pi, 0, 0])  # zyx
        self.T_lbase_camera = self.T_lbase_rbase * self.T_rbase_camera
        self.T_base_rbase = Transform.from_euler_list([0, d/2, 0, -np.pi/2, 0, 0])  # zyx
        # self.T_base_lbase = Transform.from_euler_list([0, -d/2, 0, np.pi / 2, 0, 0])  # zyx
        self.T_base_camera = self.T_base_rbase * self.T_rbase_camera
        # print('T_base_camera is', self.T_base_camera.as_matrix())
        self.T_rbase_camera = self.T_rbase_camera.as_matrix()
        self.T_lbase_camera = self.T_lbase_camera.as_matrix()
        # assert 1 < 0, print('T_lbase_camera is', self.T_lbase_camera)
        self.T_base_camera = self.T_base_camera.as_matrix()

        self.left_init = Transform.from_list([0.2, 0, 0.3, 0, 1, 0, 0])
        self.right_init = Transform.from_list([0.2, 0, 0.3, -0.7071068, 0.7071068, 0, 0])
        self.T_hande_tool_inv = Transform(Rotation.from_euler('z', -np.pi/4), [0, 0, 0.1895]).inverse()
        self.segmentation = SegmentationPipe()
        # self.best_grasp_pub = rospy.Publisher('/bestg_rasp', PoseStamped, queue_size=1)
        # self.topk_pub = rospy.Publisher('/topk_detected_objects_in_image', PoseArray, queue_size=2)
        # rospy.Subscriber("/topk_poses", TopkSeg, self.top_listener, )
        rospy.loginfo("Ready to take action")

    # def get_topk_poses(self):
    #     # TopkSeg_msg
    #     resp = rospy.wait_for_message('/topk_poses', TopkSeg)
    #     resp.topk_grasps.header.frame_id = "camera_color_optical_frame"
    #     self.topk_pub.publish(resp.topk_grasps)
    #     rospy.loginfo('finish get topk pose')
    #     return resp

    def get_grasp(self):
        """get grasp pose from segmentation srv.

        Args:

        Returns:
       """
        # mask-rcnn pose
        cnt = 0
        grasp_candidates = []
        cls_candidates = []
        select_cls = ['wire', 'pcb', 'can', 'stainless steel']
        # select_cls = ['wire', 'pcb', 'can']
        top_z = 1
        while cnt < 10:
            # seg_resp = self.segmentation.start()
            seg_resp = self.get_topk_poses()
            grasp_poses = seg_resp.topk_grasps.poses
            class_names = seg_resp.class_names
            for i, pose in enumerate(grasp_poses):
                if class_names[i] in select_cls:
                    ok = self.is_pose_normal(pose)
                    if ok:
                        # if pose.position.z < top_z:
                        #     top_z = pose.position.z
                        #     grasp_candidates.append(pose)
                        #     cls_candidates.append(class_names[i])

                        grasp_candidates.append(pose)
                        cls_candidates.append(class_names[i])
                        break

            if len(grasp_candidates) > 0:
                break
            rospy.logerr('waiting for suitable seg pose : %d', cnt)
            rospy.sleep(0.05)
            cnt += 1
        if len(grasp_candidates) == 0:
            rospy.logerr('do not find suitable seg pose in %d fixed trials', cnt+1)
            return None, None, None
        grasp, cls = grasp_candidates[0], cls_candidates[0]
        T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
        grasp_ps = PoseStamped()
        grasp_ps.header.frame_id = "camera_color_optical_frame"
        grasp_ps.pose = grasp
        self.best_grasp_pub.publish(grasp_ps)
        if cls == 'wire':
            # use right arm to grasp
            group_name = 'right_arm'
            mat_grasp = np.dot(self.T_rbase_camera, T_camera_obj)
        else:
            group_name = 'left_arm'
            mat_grasp = np.dot(self.T_lbase_camera, T_camera_obj)
        print('origin grasp is ', mat_grasp, 'cls is', cls)
        min_z = -0.005
        if mat_grasp[2, 3] < min_z:
            mat_grasp[2, 3] = min_z  # z
            rospy.logerr('z %.8f is wrong, error is %.8f', mat_grasp[2, 3], -0.005-mat_grasp[2, 3])
        print('z in camera frame is', T_camera_obj[2, 3], 'grasp z in local is', mat_grasp[2, 3])
        grasp = Transform.from_matrix(mat_grasp)
        return group_name, grasp, cls

    def select_cls_grasp(self, grasp_poses, class_names):
        grasp_candidates = []
        select_cls = ['wire', 'pcb', 'can']
        num = 0
        for i, pose in enumerate(grasp_poses):
            if class_names[i] in select_cls:
                ok = self.is_pose_normal(pose)
                if ok:
                    grasp_candidates.append(pose)
                    break
        return grasp_candidates

    def is_pose_normal(self, grasp):
        ori = grasp.orientation
        ori_norm = np.sqrt(ori.x ** 2 + ori.y ** 2 + ori.z ** 2 + ori.w ** 2)
        distance = np.sqrt(0.3 ** 2 + 0.2 ** 2 + 0.12 ** 2)
        if ori_norm > 0:
            T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
            T_base_obj = np.dot(self.T_base_camera, T_camera_obj)
            ok_dis = np.linalg.norm(T_base_obj[0:3, 3]) < distance
            x, y, z = T_base_obj[0, 3], T_base_obj[1, 3], T_base_obj[2, 3]
            ok_region = (-0.3 < x < 0.3) and (-0.2 < y < 0.2) and (z < 0.12)
            if ok_dis and ok_region:
                return True
            else:
                return False
        else:
            return False

    def plan_trajectories(self, group_name, grasp, label):
        """plan trajectories poses for dual arm.

        Args:
            grasps: [left_grasp, left_grasp]
            labels: labels for grasps
        Returns:
            trajectory_poses: a list of poses in global base frame
            [T_base_pregrasp, T_base_grasp, T_base_retreat, T_base_place]
       """
        rospy.loginfo("execute plan trajectories")

        poses = []
        T_grasp_rot = Transform.identity()
        T_grasp_rot.rotation = Rotation.from_euler('Z', -np.pi)
        T_base_grasp = grasp * T_grasp_rot  # rotate pi
        print('new rot grasp is', T_base_grasp)
        x, y, z = T_base_grasp.translation
        if not (x > 0.22 and np.abs(y) < 0.3 and z < 0.12):
            return None, None
        # T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, -0.05])  # current frame Z axis aim down, so z is -0.05
        # T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.05])  # global frame Z axis aim up, so z is 0.05
        # T_base_pregrasp = T_base_grasp * T_grasp_pregrasp  # current frame
        # T_base_retreat = T_grasp_retreat * T_base_grasp  # global frame
        if group_name == 'left_arm':
            T_base_grasp.rotation = Rotation.from_quat([0, 1, 0, 0])  # don't rotate
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])  # current frame Z axis aim down, so z is -0.05
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.12])  # global frame Z axis aim up, so z is 0.05
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp   # global frame
        T_base_retreat = T_grasp_retreat * T_base_grasp  # global frame
        # T_base_retreat.rotation = Rotation.from_quat([0, 1, 0, 0])
        # one tra policy
        # if group_name == 'right_arm':
        #     T_middle_pre = Transform.from_list([0.32, 0.10, 0.25, 0.7071068, 0.7071068, 0, 0])
        # else:
        #     T_middle_pre = Transform.from_list([0.32, -0.10, 0.25, 0, 1, 0, 0])
        #     T_base_grasp = T_base_grasp * Transform(Rotation.identity(), [0, 0, 0.003])  # press 1 mm along z axis
        base_trans = [0.30, 0.25, 0.20]
        if group_name == 'right_arm':
            T_middle_pre = Transform(Rotation.from_quat([0.7071068, 0.7071068, 0, 0]), base_trans)
            T_base_grasp = T_base_grasp * Transform(Rotation.identity(), [0, 0, 0.005])  # press 1 mm along z axis
            min_z = -0.005
            if T_base_grasp.translation[2] < min_z:
                T_base_grasp.translation[2] = min_z  # z
        else:

            base_trans[1] = -base_trans[1]
            T_middle_pre = Transform(Rotation.from_quat([0, 1, 0, 0]), base_trans)
            T_base_grasp = T_base_grasp * Transform(Rotation.identity(), [0, 0, 0.011])  # press 1 mm along z axis

        T_base_middle_place, T_base_place = self.select_place_pose(T_base_grasp.rotation, group_name, label)

        trajectory1 = [T_middle_pre, T_base_pregrasp, T_base_grasp]
        trajectory2 = [T_base_retreat, T_middle_pre, T_base_place]
        rospy.loginfo("finish plan trajectories")
        return trajectory1, trajectory2

    def select_place_pose(self, rotation, group_name, cls):
        """local frame"""
        translation = [0.13, 0.27, 0.18]
        # translation = [0.161256, 0.24, 0.2]
        # middle_translation = [0.27599, 0.32641, 0.18]
        middle_translation = [0.24, 0.25, 0.25]
        if cls == 'can':
            translation = translation
        elif cls == 'stainless steel':
            translation[0] -= 0.16
            translation[1] += 0.12
        elif cls == 'aluminum chunk':
            translation[0] -= 0.2
        elif cls == 'pcb':
            translation[1] += 0.12
        elif cls == 'wire':
            translation[0] -= 0.1
            translation[1] += 0.12
        # elif cls == 'copper chunk':
        #     translation[0] -= 0.2
        #     translation[1] += 0.12
        # elif cls == 'sponge':
        #     translation[0] -= 0.3
        #     translation[1] += 0.12

        # two rotation
        # place_rotation = Rotation.from_quat([-0.7071068, 0.7071068, 0, 0])
        place_rotation = Rotation.from_quat([0.7071068, 0.7071068, 0, 0])

        T_base_place = Transform(place_rotation, translation)

        if group_name == 'right_arm':
            T_base_middle = Transform(place_rotation, middle_translation)
            T_base_place = Transform(place_rotation, [0.07, 0.35, 0.18])
            return T_base_middle, T_base_place
        elif group_name == 'left_arm':
            T_base_middle = Transform(Rotation.from_quat([0, 1, 0, 0]), middle_translation)
            T_base_place.rotation = Rotation.from_quat([0, 1, 0, 0])
            T_base_middle.translation[1] = -T_base_middle.translation[1]
            T_base_place.translation[1] = -T_base_place.translation[1]  # -y left arm
            return T_base_middle, T_base_place
        else:
            rospy.logerr('Unknown group name')
            return None, None

    def execute_both_joint_trajectory(self, both_poses):
        rospy.loginfo("execute both poses")
        self.mc.goto_all_joint_poses(both_poses)
        rospy.loginfo("finish both poses")

    def execute_joint_trajectory(self, group_name, pose):
        rospy.loginfo("execute joint poses")
        self.mc.goto_joint_pose(group_name, pose)
        rospy.loginfo("finish joint poses")

    def execute_joint_trajectory_without_tool(self, group_name, pose):
        rospy.loginfo("execute joint poses")
        self.mc.goto_joint_pose(group_name, pose * self.T_hande_tool_inv)
        rospy.loginfo("finish joint poses")

    def execute_cart_trajectory(self,  group_name, trajectory):
        rospy.loginfo("execute cartesian trajectory")
        self.mc.goto_many_poses(group_name, trajectory)
        rospy.loginfo("finish cartesian trajectory")

    def prepare_pose(self):
        rospy.loginfo("go to prepare pose")
        init_poses = [[self.left_init, self.right_init]]
        global_poses = self.local_to_global_base(init_poses)
        self.mc.goto_all_joint_poses(global_poses[0])
        rospy.loginfo("dual arm finish prepare pose")

    def middle_pose(self):
        rospy.loginfo("go to middle pose")
        left_middle = Transform.from_list([0.2, -0.05, 0.3, 0, 1, 0, 0])
        right_middle = Transform.from_list([0.2, 0.05, 0.3, 1, 0, 0, 0])
        middle_poses = [[left_middle, right_middle]]
        global_poses = self.local_to_global_base(middle_poses)
        self.mc.goto_all_joint_poses(global_poses[0])
        rospy.loginfo("dual arm finish middle pose")


    def add_collision_object(self):
        support_w = 0.04
        camera_h = 0.79
        self.mc.add_collision_box('dual_arm', 'table', Transform.from_list([0, 0, -0.085, 0, 0, 0, 1]),
                                  [0.9, 1.2, 0.1])
        self.mc.add_collision_box('dual_arm', 'table 2', Transform.from_list([-0.57, 0, 0, 0, 0, 0, 1]),
                                  [0.01, 1.2, 0.6])
        self.mc.add_collision_box('dual_arm', 'support 1', Transform.from_list([-0.394, 0.5, 0.5, 0, 0, 0, 1]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'support 2', Transform.from_list([-0.394, -0.5, 0.5, 0, 0, 0, 1]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'support 3', Transform.from_list([-0.394, 0, camera_h, 0.707, 0, 0, 0.707]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'camera support', Transform.from_list([-0.21, 0.01, camera_h, 0, 0.707, 0, 0.707]),
                                  [0.04, 0.04, 0.38])
        dx, l, w, h, thick = 0.03, 0.58, 0.37, 0.09, 0.01
        trans_x = 0.025
        self.mc.add_collision_box('dual_arm', 'box 1', Transform.from_list([dx+trans_x, w/2, h/2, 0, 0, 0, 1]),
                                  [l, thick, h])
        self.mc.add_collision_box('dual_arm', 'box 2', Transform.from_list([dx+trans_x, -w/2, h/2, 0, 0, 0, 1]),
                                  [l, thick, h])
        self.mc.add_collision_box('dual_arm', 'box 3', Transform.from_list([l/2 + dx+trans_x, 0, h/2, 0, 0, 0, 1]),
                                  [thick, w, h])
        self.mc.add_collision_box('dual_arm', 'box 4', Transform.from_list([-l/2 + dx+trans_x, 0, h/2, 0, 0, 0, 1]),
                                  [thick, w, h])

    def get_grasp_sim(self):
        # left_grasp = Transform.from_list([0.3, -0.05, 0.02, 0.7071068, 0.7071068, 0, 0])
        left_grasp = Transform.from_list([0.46803, -0.04976, -0.008, 0.7071068, 0.7071068, 0, 0])
        right_grasp = Transform.from_list([0.4, -0.05, 0.0, 1, 0, 0, 0])
        labels = ['wire', 'aluminum chunk']
        grasps = [left_grasp, right_grasp]
        group_names = ['left_arm','right_arm']
        return group_names, grasps, labels

    def run_cartesian(self):

        rospy.loginfo("start grasp loop")
        cnt = 11
        self.add_collision_object()
        # assert 1<0
        # self.init_grippers()
        # self.mc.robotiq_vacuum_gripper_control(0)  # release
        # self.mc.robotiq_2f_gripper_control(0.03, max_effort=20)  # release
        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            rospy.loginfo("first trial")
            group_name, grasp, cls = self.get_grasp()
            if not isinstance(grasp, Transform):
                continue
            trajectory1, trajectory2 = self.plan_trajectories(group_name, grasp, cls)
            if trajectory1 is None:
                continue
            self.execute_cart_trajectory(group_name, trajectory1)
            self.close_gripper(group_name)
            self.execute_cart_trajectory(group_name, trajectory2)
            self.open_gripper(group_name)

    def close_gripper(self, group_name):
        if group_name == 'right_arm':
            self.mc.robotiq_2f_gripper_control(0, max_effort=20)  # grasp
        else:
            self.mc.robotiq_vacuum_gripper_control(1)  # grasp

    def open_gripper(self, group_name):
        if group_name == 'right_arm':
            self.mc.robotiq_2f_gripper_control(0.03, max_effort=20)  # half width
        else:
            self.mc.robotiq_vacuum_gripper_control(0)

    def init_grippers(self):
        rospy.loginfo("waiting for realease gripper")
        self.mc.robotiq_2f_gripper_control(0.03, max_effort=20)  # release
        self.mc.robotiq_vacuum_gripper_control(0)  # release
        rospy.loginfo("finish realease gripper")

    def run_cartesian_sim_with_vision(self):
        rospy.loginfo("start grasp loop")
        cnt = 2
        self.add_collision_object()
        # assert 1<0
        # goals = np.pi / 180. * np.array([0, 20, -20, 0, -90, 0])
        # self.mc.goto_joints('right_arm', goals)
        # self.mc.goto_joints('left_arm', goals)
        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            group_name, grasp, cls = self.get_grasp()
            if not isinstance(grasp, Transform):
                continue
            trajectory1, trajectory2 = self.plan_trajectories(group_name, grasp, cls)
            if trajectory1 is None:
                continue
            self.execute_cart_trajectory(group_name, trajectory1)

            self.execute_cart_trajectory(group_name, trajectory2)


    def run_cartesian_sim(self):
        rospy.loginfo("start grasp loop")
        cnt = 2
        self.add_collision_object()
        # assert 1<0

        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            group_names, grasps, cls = self.get_grasp_sim()
            num = 0
            group_name, grasp, cls = group_names[num], grasps[num], cls[num]
            trajectory1, trajectory2 = self.plan_trajectories(group_name, grasp, cls)
            self.execute_cart_trajectory(group_name, trajectory1)
            self.execute_cart_trajectory(group_name, trajectory2)
            num += 1


    # def run_left_cartesian(self):
    #     rospy.loginfo("start grasp loop")
    #     cnt = 2
    #     self.add_collision_object()
    #     # assert 1<0
    #     rospy.loginfo("waiting for realease gripper")
    #     rospy.loginfo("realease gripper first")
    #     group_name = 'right_arm'
    #     while not rospy.is_shutdown() and cnt > 0:
    #         cnt -= 1
    #         grasps, labels = self.get_grasp_sim()
    #         grasps, labels = grasps[0], labels[0]
    #         trajectory1, trajectory2 = self.plan_trajectories(grasps, labels, group_name)
    #
    #         self.execute_cart_trajectory(group_name, trajectory1)
    #         # rospy.sleep(0.3)
    #         # self.mc.robotiq_vacuum_gripper_control(1)  # grasp
    #         self.execute_cart_trajectory(group_name, trajectory2)
    #         # rospy.sleep(0.3)
    #         # self.mc.robotiq_vacuum_gripper_control(0)  # release
    #         group_name = 'left_arm'

    def run_sim(self):
        rospy.loginfo("start grasp loop")
        cnt = 1
        self.add_collision_object()
        # assert 1<0
        group_name = 'right_arm'
        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            grasps, labels = self.get_grasp_sim()
            grasps, labels = grasps[1], labels[1]
            trajectory1, trajectory2 = self.plan_trajectories(grasps, labels, group_name)
            self.execute_joint_trajectory_without_tool(group_name, trajectory1[0])
            self.execute_joint_trajectory_without_tool(group_name, trajectory1[1])
            # self.mc.robotiq_2f_gripper_control(0, max_effort=20)  # grasp
            self.execute_joint_trajectory_without_tool(group_name, trajectory2[0])
            self.execute_joint_trajectory_without_tool(group_name, trajectory2[1])

    def run_ee_sim(self):
        rospy.loginfo("start grasp loop")
        cnt = 2
        self.add_collision_object()
        # assert 1<0
        group_name = 'left_arm'
        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            grasps, labels = self.get_grasp_sim()
            grasps, labels = grasps[0], labels[0]
            trajectory1, trajectory2 = self.plan_trajectories(grasps, labels, group_name)
            self.execute_joint_trajectory(group_name, trajectory1[0])
            self.execute_joint_trajectory(group_name, trajectory1[1])
            # self.mc.robotiq_2f_gripper_control(0, max_effort=20)  # grasp
            self.execute_joint_trajectory(group_name, trajectory2[0])
            self.execute_joint_trajectory(group_name, trajectory2[1])
            group_name = 'right_arm'

    def test_6d_pose(self):
        rospy.loginfo("start grasp loop")
        cnt = 100
        # self.add_collision_object()
        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            seg_resp = self.segmentation.start()
            # rospy.sleep(0.01)

    def test_6d_pose_msg(self):
        rospy.loginfo("start grasp loop")
        cnt = 5
        # self.add_collision_object()
        while not rospy.is_shutdown() and cnt > 0:
            cnt -= 1
            seg_resp = self.get_topk_poses()
            rospy.sleep(0.5)

    # def run_cartesian_1(self):
    #     # dual arm grasp objects simultaneously, need 2 grasps
    #
    #     rospy.loginfo("start grasp loop")
    #     cnt = 2
    #     self.add_collision_object()
    #     while not rospy.is_shutdown() and cnt > 0:
    #         if cnt % 2 == 0:
    #             group_name = 'right_arm'
    #             # grasp = Transform.from_list([0.4, -0.1, 0.1, 1, 0, 0, 0])
    #             right_grasp = Transform.from_list([0.4, -0.1, 0.02, 1, 0, 0, 0])
    #             grasp = right_grasp
    #
    #         # pose = Transform.from_list([0.2, 0.1, 0.3, 0.7071068, 0.7071068, 0, 0])
    #         # self.execute_joint_trajectory(group_name, pose)
    #         # pose = Transform.from_list([0.05, 0.2, 0.25, 0.7071068, 0.7071068, 0, 0])
    #         # self.execute_joint_trajectory(group_name, pose)
    #         # cnt -= 1
    #         else:
    #             group_name = 'left_arm'
    #             # grasp = Transform.from_list([0.4, -0.1, 0.1, 0, 1, 0, 0])
    #             left_grasp = Transform.from_list([0.3, -0.05, 0.02, 0.7071068, 0.7071068, 0, 0])
    #             grasp = left_grasp
    #
    #         label = 'can'
    #         trajectory1, trajectory2 = self.plan_trajectories(grasp, label, group_name)
    #
    #         # self.execute_cart_trajectory(group_name, trajectory1)
    #         # rospy.sleep(0.3)
    #         # self.execute_cart_trajectory(group_name, trajectory2)
    #
    #         self.execute_cart_trajectory(group_name, trajectory1)
    #         rospy.sleep(0.3)
    #
    #         self.execute_cart_trajectory(group_name, trajectory2)
    #     #
    #         cnt -= 1
    #     # rospy.loginfo("finish grasp loop")


def main():
    rospy.init_node("nachi_dual_arm_grasp")
    print('dual arms is starting')
    arm_grasp = NachiGraspController()
    raw_input('please press enter to start grasp loop')

    # arm_grasp.run_cartesian()
    arm_grasp.test_6d_pose()
    # arm_grasp.test_6d_pose_msg()
    # arm_grasp.run_cartesian_sim_with_vision()


    # arm_grasp.run_sim()
    # arm_grasp.run_ee_sim()
    # arm_grasp.run_cartesian_sim_with_vision()


if __name__ == "__main__":
    main()
