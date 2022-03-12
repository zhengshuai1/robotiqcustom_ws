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
from segmentation_srv.srv import Segmentation
from geometry_msgs.msg import PoseStamped
import time


class SegmentationPipe(object):
    def __init__(self):
        super(SegmentationPipe, self).__init__()
        print('wait for seg')
        rospy.wait_for_service('Segmentation')
        print('wait for seg end')
        self.service = rospy.ServiceProxy('Segmentation', Segmentation)
        # self.bridge = CvBridge()
        self.pub = rospy.Publisher('/detected_objects_in_image', PoseStamped, queue_size=10)

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
            rospy.logerr('end loop')
            rospy.sleep(0.025)
        if self.color and self.depth:
            t1 = time.time()
            pose = self.segmentation_client()
            t2 = time.time()
            print('time is ', t2-t1)
            self.pub.publish(pose.best_grasp)
            rospy.logerr('end sense')
            return pose

    def start(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.color_listener)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_listener)
        rospy.logerr('start sense')
        pose = self.do_segmentation()
        return pose

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
        self.active_group_name = 'left_arm'
        self.left_init = Transform.from_list([0.2, -0.05, 0.3, 0, 1, 0, 0])
        self.right_init = Transform.from_list([0.2, 0.05, 0.3, 1, 0, 0, 0])
        # self.segmentation = SegmentationPipe()
        rospy.loginfo("Ready to take action")

    def get_grasp(self):
        # mask-rcnn pose
        is_grasp_normal = False
        while not is_grasp_normal:
            pose = self.segmentation.start()
            best_grasp = pose.best_grasp.pose
            cls = pose.class_name
            print('best grasp orientation is', best_grasp.orientation.x, best_grasp.orientation.y,
                  best_grasp.orientation.z, best_grasp.orientation.w)
            ori = best_grasp.orientation
            ori_norm = np.sqrt(ori.x**2 + ori.y**2 + ori.z**2 + ori.w**2)
            distance = np.sqrt(0.35**2 + 0.25**2 + 0.15**2)
            if ori_norm > 0:
                T_camera_obj = ros_utils.from_pose_msg(best_grasp).as_matrix()
                # T_camera_obj[2, 3] = T_camera_obj[2, 3] - 0.04  ### TODO simulation
                T_base_obj = np.dot(self.T_base_camera, T_camera_obj)

                if np.linalg.norm(T_base_obj[0:3, 3]) < distance:
                    is_grasp_normal = True
                    break
            rospy.logerr('waiting for suitable seg pose')
            rospy.sleep(0.01)

        left_grasp = np.dot(self.T_lbase_camera, T_camera_obj)
        right_grasp = np.dot(self.T_rbase_camera, T_camera_obj)
        # assert 1 < 0, print('left grasp:', left_grasp, 'right grasp', right_grasp)
        print('left grasp:', left_grasp, 'right grasp', right_grasp)
        rospy.logerr('class is %s', cls)
        if 0 < left_grasp[0, 3] < 0.45:
            select_group = 'left_arm'
            grasp = left_grasp
        elif 0 < right_grasp[0, 3] < 0.45:
            select_group = 'right_arm'
            grasp = right_grasp
        else:
            select_group = 'right_arm'
            grasp = right_grasp

        # T_base_obj = self.tf_tree.transform(best_grasp, 'camera_color_optical_frame', 'mz04_link0')
        # print('T_base_obj', grasp, cls)
        grasp = Transform.from_matrix(grasp)
        return select_group, grasp, cls

    def plan_trajectories(self, group_name, grasp, cls):
        rospy.loginfo("execute plan trajectories")

        left_init = self.left_init
        right_init = self.right_init
        T_base_grasp = grasp
        rotation = T_base_grasp.rotation
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp
        T_base_retreat = T_grasp_retreat * T_base_grasp
        T_base_middle_place, T_base_place = self.select_place_pose(T_base_grasp.rotation, group_name, cls)
        poses_init = [left_init, right_init]

        poses_1 = [T_base_pregrasp, T_base_place]
        poses_2 = [T_base_grasp, right_init]
        poses_3 = [T_base_retreat, None]
        if group_name == 'right_arm':
            # for i in range(len(trajectory_poses)):
            #     trajectory_poses[i] = trajectory_poses[i][::-1]

            poses_1 = [T_base_place, T_base_pregrasp]
            poses_2 = [left_init, T_base_grasp]
            poses_3 = [None, T_base_retreat]

        # poses_1 = [T_base_pregrasp, T_base_middle_place]
        # poses_2 = [T_base_grasp, T_base_place]
        # poses_3 = [T_base_retreat, T_base_middle_place]

        trajectory_poses = [poses_init, poses_1, poses_2, poses_3]
        rospy.loginfo("finish plan trajectories")
        trajectory_poses = self.local_to_global_base(trajectory_poses)
        return trajectory_poses

    def select_place_pose(self, rotation, group_name, cls):
        """local frame"""
        translation = [0.161256, 0.20, 0.165748]
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

        # activate right arm = T_base_left_middle
        if group_name == 'left_arm':
            T_base_middle = Transform(place_rotation, middle_translation)
            return T_base_middle, T_base_place
        elif group_name == 'right_arm':
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
        rospy.loginfo("go to init pose")

        init_poses = [[self.left_init, self.right_init]]
        global_poses = self.local_to_global_base(init_poses)
        self.mc.goto_all_joint_poses(global_poses[0])
        rospy.loginfo("dual arm inish init pose")

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
        self.mc.add_collision_box('dual_arm', 'box 1', Transform.from_list([0, 0.22, 0.045, 0, 0, 0, 1]),
                                  [0.8, 0.01, 0.09])
        self.mc.add_collision_box('dual_arm', 'box 2', Transform.from_list([0, -0.22, 0.045, 0, 0, 0, 1]),
                                  [0.8, 0.01, 0.09])
        self.mc.add_collision_box('dual_arm', 'box 3', Transform.from_list([0.4, 0, 0.045, 0, 0, 0, 1]),
                                  [0.01, 0.44, 0.09])
        self.mc.add_collision_box('dual_arm', 'box 4', Transform.from_list([-0.4, 0, 0.045, 0, 0, 0, 1]),
                                  [0.01, 0.44, 0.09])

    def get_grasp_sim(self):
        left_grasp = Transform.from_list([0.4, 0.15, 0.02, 0, 1, 0, 0])
        right_grasp = Transform.from_list([0.4, 0.15, 0.02, 1, 0, 0, 0])
        # left_grasp, right_grasp = self.local_to_global_base([[left_grasp, right_grasp]])[0]
        return left_grasp, right_grasp

    def run_cartesian(self):
        rospy.loginfo("start grasp loop")
        cnt = 2
        self.add_collision_object()
        self.prepare_pose()
        while not rospy.is_shutdown() and cnt > 0:
            # rospy.loginfo('grasp : %d', 9-cnt)

            left_grasp, right_grasp = self.get_grasp_sim()
            cls = 'copper chunk'
            if cnt % 2 == 0:
                group_name = 'left_arm'
                grasp = left_grasp
            else:
                group_name = 'right_arm'
                grasp = right_grasp
            # group_name = self.active_group_name

            trajectory_poses = self.plan_trajectories(group_name, grasp, cls)  # grasp local frame
            self.execute_trajectory(trajectory_poses[1])
            self.execute_trajectory(trajectory_poses[2])
            self.mc.move_group_gripper(group_name, width=0, force=20)
            if group_name == 'left_arm':
                pose0 = trajectory_poses[0][0]
                pose3 = trajectory_poses[3][0]
            else:
                pose0 = trajectory_poses[0][1]
                pose3 = trajectory_poses[3][1]
            self.execute_single_group_trajectory(group_name, pose3)
            self.execute_single_group_trajectory(group_name, pose0)
            self.prepare_pose()
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
