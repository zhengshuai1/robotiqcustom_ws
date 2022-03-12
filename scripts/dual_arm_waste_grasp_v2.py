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
        # self.pub = rospy.Publisher('/detected_objects_in_image', PoseStamped, queue_size=10)

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
            # self.pub.publish(pose.best_grasp)
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
        self.tf_tree = ros_utils.TransformTree()
        self.mc = DualArmCommander()

        self.T_rbase_camera = Transform.from_list(
            [0.309045, -0.0104731, 0.982099, 0.999115, -0.00655622, -0.0143316, -0.0389965])

        self.T_lbase_rbase = Transform.from_euler_list([0.919, 0, 0, np.pi, 0, 0])  # zyx
        self.T_lbase_camera = self.T_lbase_rbase * self.T_rbase_camera

        self.T_base_rbase = Transform.from_euler_list([0, 0.4595, 0, -np.pi/2, 0, 0])  # zyx
        self.T_base_camera = self.T_base_rbase * self.T_rbase_camera

        self.T_rbase_camera = self.T_rbase_camera.as_matrix()
        self.T_lbase_camera = self.T_lbase_camera.as_matrix()
        # assert 1 < 0, print('T_lbase_camera is', self.T_lbase_camera)
        self.T_base_camera = self.T_base_camera.as_matrix()

        self.segmentation = SegmentationPipe()
        rospy.loginfo("Ready to take action")

    def get_grasp(self):
        # mask-rcnn pose
        # top 3 or 5 grasp
        seg_resp = self.segmentation.start()
        grasps = seg_resp.grasps
        class_names = seg_resp.class_name
        assert len(grasps) == len(class_names)
        for grasp, class_name in zip(grasps, class_names):
            T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
            T_base_obj = np.dot(self.T_base_camera, T_camera_obj)
            if T_base_obj[0, 3] > 0 and T_base_obj[1, 3] > 0:
                select_group = 'right_arm'
                grasp = grasp


        # print('best grasp orientation is', best_grasp.orientation.x, best_grasp.orientation.y,
        #       best_grasp.orientation.z, best_grasp.orientation.w)

        T_camera_obj = ros_utils.from_pose_msg(best_grasp).as_matrix()
        left_grasp = np.dot(self.T_lbase_camera, T_camera_obj)
        right_grasp = np.dot(self.T_rbase_camera, T_camera_obj)
        # assert 1 < 0, print('left grasp:', left_grasp, 'right grasp', right_grasp)
        print('left grasp:', left_grasp, 'right grasp', right_grasp)
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

    def plan_trajectories(self, group_name, grasp, cls='metal'):
        rospy.loginfo("execute plan trajectories")
        T_base_grasp = grasp
        initial_pose = Transform.from_list([0.14693, -0.00002, 0.38182, 1, 0, 0, 0])
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp
        T_base_retreat = T_grasp_retreat * T_base_grasp
        # T_base_place = self.select_place_pose(cls)
        if group_name == 'left_arm':
            T_base_place = Transform.from_list([0.161256, -0.279421, 0.165748, 1, 0, 0, 0])
        elif group_name == 'right_arm':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        # elif group_name == 'dual_arm':
        # T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])

        T_place_preplace = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        # T_base_preplace = T_place_preplace * T_base_place

        trajectory1 = [T_base_pregrasp, T_base_grasp]
        trajectory2 = [T_base_retreat, T_base_place]
        # trajectory3 = [T_base_preplace]
        rospy.loginfo("finish plan trajectories")
        return trajectory1, trajectory2

    def select_place_pose(self, cls):
        if cls == 'wire':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        elif cls == 'can':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        elif cls == 'stainless steel':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        elif cls == 'copper chunk':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        elif cls == 'aluminum chunk':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        elif cls == 'sponge':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        elif cls == 'pcb':
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        else:
            T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
        return T_base_place

    def execute_trajectory(self, group_name, trajectory):
        rospy.loginfo("execute trajectory")
        self.mc.goto_many_poses(group_name, trajectory)
        rospy.loginfo("finish trajectory")

    def home(self):
        rospy.loginfo("go to home")
        goals = np.pi / 180. * np.array([0, 30, -20, 0, -90, 0])
        self.mc.goto_joints('left_arm', goals)
        self.mc.goto_joints('right_arm', goals)

    def add_collision_object(self):
        self.mc.add_collision_box('dual_arm', 'table', Transform.from_list([0, 0, -0.065, 0, 0, 0, 1]),
                                  [0.9, 1.2, 0.1])
        self.mc.add_collision_box('dual_arm', 'support 1', Transform.from_list([-0.394, 0.5, 0.5, 0, 0, 0, 1]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'support 2', Transform.from_list([-0.394, -0.5, 0.5, 0, 0, 0, 1]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'support 3', Transform.from_list([-0.394, 0, 1, 0.707, 0, 0, 0.707]),
                                  [0.04, 0.04, 1])
        self.mc.add_collision_box('dual_arm', 'camera support', Transform.from_list([-0.21, 0.15, 1, 0, 0.707, 0, 0.707]),
                                  [0.04, 0.04, 0.38])
        # self.mc.add_collision_box('dual_arm', 'box 1', Transform.from_list([0, 0.22, 0.045, 0, 0, 0, 1]),
        #                           [0.64, 0.01, 0.09])
        # self.mc.add_collision_box('dual_arm', 'box 2', Transform.from_list([0, -0.22, 0.045, 0, 0, 0, 1]),
        #                           [0.64, 0.01, 0.09])
        # self.mc.add_collision_box('dual_arm', 'box 3', Transform.from_list([0.32, 0, 0.045, 0, 0, 0, 1]),
        #                           [0.01, 0.44, 0.09])
        # self.mc.add_collision_box('dual_arm', 'box 4', Transform.from_list([-0.32, 0, 0.045, 0, 0, 0, 1]),
        #                           [0.01, 0.44, 0.09])

    def run_cartesian(self):
        rospy.loginfo("start grasp loop")
        cnt = 4
        # self.home()
        self.add_collision_object()
        # assert 1 < 0, print('1')
        while not rospy.is_shutdown() and cnt > 0:
            group_name, grasp, cls = self.get_grasp()
            trajectory1, trajectory2 = self.plan_trajectories(group_name, grasp, cls)
            self.execute_trajectory(group_name, trajectory1)
            self.mc.move_gripper(width=0, force=20)
            self.execute_trajectory(group_name, trajectory2)
            self.mc.move_gripper(width=0.08, force=0)
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
