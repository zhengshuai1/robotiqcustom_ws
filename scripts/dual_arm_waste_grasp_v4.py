#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from utils import ros_utils
from utils.dual_arm_control import DualArmCommander
from utils.transform import Rotation, Transform
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from segmentation_srv.srv import Segmentation, TopkSegmentation
from geometry_msgs.msg import PoseStamped, PoseArray
import time


class SegmentationPipe(object):
    def __init__(self):
        super(SegmentationPipe, self).__init__()
        print('wait for seg')
        rospy.wait_for_service('Segmentation')
        print('wait for seg end')
        self.service = rospy.ServiceProxy('Segmentation', TopkSegmentation)
        self.pub = rospy.Publisher('/detected_objects_in_image', PoseStamped, queue_size=10)
        self.topk_pub = rospy.Publisher('/topk_detected_objects_in_image', PoseArray, queue_size=2)

        self.depth = None
        self.color = None

    def segmentation_client(self):
        try:
            pose = self.service(self.color, self.depth)
            return pose
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def color_listener(self, data):
        self.color = data

    def depth_listener(self, data):
        self.depth = data

    def do_segmentation(self):
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
            # self.topk_pub.publish(resp.topk_grasps)
            rospy.logerr('end sense')
            return resp

    def start(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_listener)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_listener)
        rospy.logerr('start sense')
        seg_resp = self.do_segmentation()
        return seg_resp

class NachiGraspController(object):
    """Nachi robot grasp Control"""

    def __init__(self):
        super(NachiGraspController, self).__init__()
        # self.tf_tree = ros_utils.TransformTree()
        self.mc = DualArmCommander()

        self.T_rbase_camera = Transform.from_list(
            [0.309045, -0.0104731, 0.982099, 0.999115, -0.00655622, -0.0143316, -0.0389965])

        self.T_lbase_rbase = Transform.from_euler_list([0.919, 0, 0, np.pi, 0, 0])  # zyx
        self.T_lbase_camera = self.T_lbase_rbase * self.T_rbase_camera

        self.T_base_rbase = Transform.from_euler_list([0, 0.4595, 0, -np.pi/2, 0, 0])  # zyx
        self.T_base_lbase = Transform.from_euler_list([0, -0.4595, 0, np.pi / 2, 0, 0])  # zyx
        self.T_base_camera = self.T_base_rbase * self.T_rbase_camera

        self.T_rbase_camera = self.T_rbase_camera.as_matrix()
        self.T_lbase_camera = self.T_lbase_camera.as_matrix()
        # assert 1 < 0, print('T_lbase_camera is', self.T_lbase_camera)
        self.T_base_camera = self.T_base_camera.as_matrix()

        self.left_init = Transform.from_list([0.2, 0, 0.3, 0, 1, 0, 0])
        self.right_init = Transform.from_list([0.2, 0, 0.3, 1, 0, 0, 0])
        # self.segmentation = SegmentationPipe()
        rospy.loginfo("Ready to take action")

    def get_grasp(self):
        """get grasp pose from segmentation srv.

        Args:

        Returns:
       """
        # mask-rcnn pose
        safe_distance = 0.25  # m
        cnt = 8
        while True:
            seg_resp = self.segmentation.start()
            grasp_poses = seg_resp.topk_grasps.poses
            class_names = seg_resp.class_names

            top1_grasp = grasp_poses[0]
            top1_grasp_angle = 2 * np.arcsin(top1_grasp.orientation.z)
            state = self.is_pose_normal(top1_grasp)
            print('state is ', state, 'pose num is', len(grasp_poses))
            if state:
                candidate_grasps = []
                candidate_class_names = []
                distance = []

                # for i, pose in enumerate(grasp_poses[1:]):
                #     d = np.sqrt((pose.position.x-top1_grasp.position.x)**2 + (pose.position.y-top1_grasp.position.y)**2)
                #     print('d is', d)
                #     if d > safe_distance:
                #         candidate_grasps.append(pose)
                #         candidate_class_names.append(class_names[i+1])

                T_camera_grasp = ros_utils.from_pose_msg(top1_grasp).as_matrix()
                T_base_grasp = np.dot(self.T_base_camera, T_camera_grasp)
                for i, pose in enumerate(grasp_poses[1:]):
                    T_camera_pose = ros_utils.from_pose_msg(pose).as_matrix()
                    T_base_pose = np.dot(self.T_base_camera, T_camera_pose)
                    d = np.linalg.norm(T_base_pose[:2, 3]-T_base_grasp[:2, 3])
                    print('d is', d)
                    if d > safe_distance:
                        distance.append(d)
                        candidate_grasps.append(pose)
                        candidate_class_names.append(class_names[i+1])

                print('candidate len is', len(candidate_grasps))
                if len(candidate_grasps):
                    select_top1_group, top1_grasp = self.select_group_from_pose(top1_grasp)
                    ind = 0
                    second_group, second_grasp = None, None
                    for i, candidate_grasp in enumerate(candidate_grasps):
                        second_group, second_grasp = self.select_group_from_pose(candidate_grasp)
                        ind = i
                        if not select_top1_group == second_group:
                            print('select safe d is', distance[i], 'm')
                            second_grasp_angle = 2 * np.arcsin(candidate_grasp.orientation.z)
                            break
                    if not select_top1_group == second_group:
                        # rospy.logerr('do not find suitable pose')
                        break
                if cnt <= 0:
                    break
            rospy.logerr('waiting for suitable seg pose')
            rospy.sleep(0.01)
            cnt -= 1

        top1_grasp = Transform.from_matrix(top1_grasp)
        second_grasp = Transform.from_matrix(second_grasp)

        print('top1 group is', select_top1_group)
        if select_top1_group == 'left_arm':
            grasps = [top1_grasp, second_grasp]
            cls_names = [class_names[0], candidate_class_names[ind]]

            grasps[0].rotation, grasps[1].rotation = Rotation.from_euler('zyx', [top1_grasp_angle, np.pi, 0]), \
                                                     Rotation.from_euler('zyx', [second_grasp_angle, 0, np.pi])
        else:
            grasps = [second_grasp, top1_grasp]
            cls_names = [candidate_class_names[ind], class_names[0]]

            # grasps[0].rotation, grasps[1].rotation = Rotation.from_euler('zyx', [second_grasp_angle, np.pi, 0]), \
            #                                          Rotation.from_euler('zyx', [top1_grasp_angle, 0, np.pi])
            grasps[0].rotation, grasps[1].rotation = Rotation.from_euler('XYZ', [0, np.pi, second_grasp_angle]), \
                                                     Rotation.from_euler('XYZ', [np.pi, 0, top1_grasp_angle])
        # print('second angle is', second_grasp_angle, 'top1 angle is', top1_grasp_angle, grasps[0].as_matrix())
        global_grasps = self.local_pose_to_global_base(grasps)

        goal_poses = ros_utils.to_posearray_msg(global_grasps)

        goal_poses.header.frame_id = 'base'
        self.segmentation.topk_pub.publish(goal_poses)
        # print('left grasp:', grasps[0].as_matrix(), 'right grasp', grasps[1].as_matrix())
        print('class is', cls_names)

        return grasps, cls_names

    def select_group_from_pose(self, grasp):
        T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
        left_grasp = np.dot(self.T_lbase_camera, T_camera_obj)
        right_grasp = np.dot(self.T_rbase_camera, T_camera_obj)
        if 0 < left_grasp[0, 3] < 0.46:
            select_group = 'left_arm'
            grasp = left_grasp
        elif 0 < right_grasp[0, 3] < 0.46:
            select_group = 'right_arm'
            grasp = right_grasp
        else:
            select_group = 'right_arm'
            grasp = right_grasp
        return select_group, grasp

    def is_pose_normal(self, grasp):
        ori = grasp.orientation
        ori_norm = np.sqrt(ori.x ** 2 + ori.y ** 2 + ori.z ** 2 + ori.w ** 2)
        distance = np.sqrt(0.35 ** 2 + 0.25 ** 2 + 0.15 ** 2)
        if ori_norm > 0:
            T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
            T_base_obj = np.dot(self.T_base_camera, T_camera_obj)
            if np.linalg.norm(T_base_obj[0:3, 3]) < distance:
                return True
            else:
                return False
        else:
            return False

    def plan_trajectories(self, grasps, labels):
        """plan trajectories poses for dual arm.

        Args:
            grasps: [left_grasp, left_grasp]
            labels: labels for grasps
        Returns:
            trajectory_poses: a list of poses in global base frame
            [T_base_pregrasp, T_base_grasp, T_base_retreat, T_base_place]
       """
        rospy.loginfo("execute plan trajectories")
        group_names = ['left_arm', 'right_arm']
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        poses = []
        for group_name, grasp, label in zip(group_names, grasps, labels):
            T_base_grasp = grasp
            T_base_pregrasp = T_grasp_pregrasp * T_base_grasp
            T_base_retreat = T_grasp_retreat * T_base_grasp
            T_base_middle_place, T_base_place = self.select_place_pose(T_base_grasp.rotation, group_name, label)
            poses.append([T_base_pregrasp, T_base_grasp, T_base_retreat, T_base_place])

        poses_1 = [poses[0][0], poses[1][0]]
        poses_2 = [poses[0][1], poses[1][1]]
        poses_3 = [poses[0][2], poses[1][2]]
        poses_4 = [poses[0][3], poses[1][3]]
        trajectory_poses = [poses_1, poses_2, poses_3, poses_4]
        rospy.loginfo("finish plan trajectories")
        trajectory_poses = self.local_to_global_base(trajectory_poses)
        return trajectory_poses

    def select_place_pose(self, rotation, group_name, cls):
        """local frame"""
        translation = [0.161256, 0.24, 0.165748]
        # middle_translation = [0.27599, 0.32641, 0.18]
        middle_translation = [0.161256, 0.15, 0.25]
        if cls == 'wire':
            translation = translation
        elif cls == 'pcb':
            translation[0] = translation[0] - 0.1
        elif cls == 'stainless steel':
            translation[0] = translation[0] - 0.2
        elif cls == 'can':
            translation[1] = translation[1] + 0.12
        elif cls == 'aluminum chunk':
            translation[0] = translation[0] - 0.1
            translation[1] = translation[1] + 0.12
        elif cls == 'copper chunk':
            translation[0] = translation[0] - 0.2
            translation[1] = translation[1] + 0.12
        elif cls == 'sponge':
            translation[0] = translation[0] - 0.3
            translation[1] = translation[1] + 0.12
         ## test
        # translation = [0.161256, 0.279421, 0.165748]
        # angle = rotation.as_euler('zyx')
        # angle[0] = -angle[0]/3

        middle_rotation = rotation.from_euler('zyx', [0, 0, 0])
        place_rotation = Rotation.from_quat([1, 0, 0, 0])
        T_base_place = Transform(place_rotation, translation)

        if group_name == 'right_arm':
            T_base_middle = Transform(place_rotation, middle_translation)
            return T_base_middle, T_base_place
        elif group_name == 'left_arm':
            T_base_middle = Transform(Rotation.from_quat([0, 1, 0, 0]), middle_translation)
            T_base_place.rotation = Rotation.from_quat([0, 1, 0, 0])
            T_base_place.translation[1] = -T_base_place.translation[1]  # -y left arm
            return T_base_middle, T_base_place
        else:
            rospy.logerr('Unknown group name')

    def execute_trajectory(self, both_poses):
        rospy.loginfo("execute both poses")
        self.mc.goto_all_joint_poses(both_poses)
        rospy.loginfo("finish both poses")

    def execute_single_group_trajectory(self, group_name, pose):
        rospy.loginfo("execute both poses")
        self.mc.goto_joint_pose(group_name, pose)
        rospy.loginfo("finish both poses")

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

    def local_to_global_base(self, local_both_poses):
        global_both_poses = []
        for local_both_pose in local_both_poses:
            if local_both_pose[0] is not None and local_both_pose[1] is not None:
                local_both_pose = [self.T_base_lbase * local_both_pose[0], self.T_base_rbase * local_both_pose[1]]
            if local_both_pose[0] is not None and local_both_pose[1] is None:
                local_both_pose = [self.T_base_lbase * local_both_pose[0], None]
            if local_both_pose[0] is None and local_both_pose[1] is not None:
                local_both_pose = [None, self.T_base_rbase * local_both_pose[1]]
            global_both_poses.append(local_both_pose)
        return global_both_poses

    def local_pose_to_global_base(self, local_both_pose):
        local_both_pose = [self.T_base_lbase * local_both_pose[0], self.T_base_rbase * local_both_pose[1]]
        return local_both_pose

    def add_collision_object(self):
        self.mc.add_collision_box('dual_arm', 'table', Transform.from_list([0, 0, -0.085, 0, 0, 0, 1]),
                                  [0.9, 1.2, 0.1])
        self.mc.add_collision_box('dual_arm', 'table 2', Transform.from_list([-0.57, 0, 0, 0, 0, 0, 1]),
                                  [0.01, 1.2, 0.6])
        self.mc.add_collision_box('dual_arm', 'support 1', Transform.from_list([-0.394, 0.5, 0.5, 0, 0, 0, 1]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'support 2', Transform.from_list([-0.394, -0.5, 0.5, 0, 0, 0, 1]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'support 3', Transform.from_list([-0.394, 0, 1, 0.707, 0, 0, 0.707]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'camera support', Transform.from_list([-0.21, 0.15, 1, 0, 0.707, 0, 0.707]),
                                  [0.04, 0.04, 0.38])
        dx = 0.03
        self.mc.add_collision_box('dual_arm', 'box 1', Transform.from_list([dx, 0.2, 0.045, 0, 0, 0, 1]),
                                  [0.58, 0.01, 0.09])
        self.mc.add_collision_box('dual_arm', 'box 2', Transform.from_list([dx, -0.2, 0.045, 0, 0, 0, 1]),
                                  [0.58, 0.01, 0.09])
        self.mc.add_collision_box('dual_arm', 'box 3', Transform.from_list([0.29+dx, 0, 0.045, 0, 0, 0, 1]),
                                  [0.01, 0.4, 0.09])
        self.mc.add_collision_box('dual_arm', 'box 4', Transform.from_list([-0.29+dx, 0, 0.045, 0, 0, 0, 1]),
                                  [0.01, 0.4, 0.09])

    def get_grasp_sim(self):
        h = 0.1
        left_grasp = Transform.from_list([0.3, -0.05, h, 0.7071068, 0.7071068, 0, 0])
        right_grasp = Transform.from_list([0.4, -0.1, h, 1, 0, 0, 0])
        labels = ['copper chunk', 'aluminum chunk']
        grasps = [left_grasp, right_grasp]
        return grasps, labels

    def run_cartesian(self):
        # dual arm grasp objects simultaneously, need 2 grasps

        rospy.loginfo("start grasp loop")
        cnt = 2
        self.add_collision_object()
        # assert 1<0
        self.prepare_pose()
        while not rospy.is_shutdown() and cnt > 0:
            # rospy.loginfo('grasp : %d', 9-cnt)

            grasps, labels = self.get_grasp_sim()
            # grasps, labels = self.get_grasp()  # grasp local frame
            trajectory_poses = self.plan_trajectories(grasps, labels)
            self.execute_trajectory(trajectory_poses[0])
            self.execute_trajectory(trajectory_poses[1])
            # self.mc.move_group_gripper('dual_arm', width=0, force=20)
            self.execute_trajectory(trajectory_poses[2])
            self.middle_pose()
            self.execute_trajectory(trajectory_poses[3])
            self.middle_pose()
            cnt -= 1
        rospy.loginfo("finish grasp loop")


def main():
    rospy.init_node("nachi_dual_arm_grasp")
    print('dual arms is starting')
    arm_grasp = NachiGraspController()
    raw_input('please press enter to start grasp loop')
    arm_grasp.run_cartesian()


if __name__ == "__main__":
    main()
