#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from yolov5_ros.msg import yolo_object
from utils import ros_utils
from utils.mz04_control import Mz04Commander
from utils.transform import Rotation, Transform
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from segmentation_srv.srv import Segmentation
from geometry_msgs.msg import PoseStamped
import time


class SegmentationPipe(object):
    def __init__(self):
        super(SegmentationPipe, self).__init__()
        rospy.wait_for_service('Segmentation')
        self.service = rospy.ServiceProxy('Segmentation', Segmentation)
        self.bridge = CvBridge()
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
        self.tf_tree = ros_utils.TransformTree()
        self.mc = Mz04Commander()
        self.T_base_camera = Transform.from_list(
            [0.309045, -0.0104731, 0.982099, 0.999115, -0.00655622, -0.0143316, -0.0389965])
        self.T_base_camera = self.T_base_camera.as_matrix()
        self.segmentation = SegmentationPipe()
        rospy.loginfo("Ready to take action")

    # def get_grasp(self):
    #     rospy.loginfo('waiting for pos')
    #     pose = rospy.wait_for_message("/detected_objects_in_image", yolo_object)
    #     cls, pos = pose.Class, np.array([pose.x / 1000, pose.y / 1000, pose.z / 1000, 1])
    #     position_base_obj = np.dot(self.T_base_camera, pos.reshape(4, 1)).transpose()
    #     position_base_obj = position_base_obj.squeeze()[:3]
    #     rot = np.array([1, 0, 0, 0])
    #     pose_base_obj = np.append(position_base_obj, rot)
    #     print('pose_base_obj =', pose_base_obj)
    #     Tran_base_obj = Transform.from_list(pose_base_obj)
    #     return Tran_base_obj, cls

    def get_grasp(self):
        # mask rcnn pose
        pose = self.segmentation.start()
        best_grasp = pose.best_grasp
        cls = pose.class_name
        T_camera_obj = ros_utils.from_pose_msg(best_grasp).as_matrix()
        T_base_obj = np.dot(self.T_base_camera, T_camera_obj)
        # T_base_obj = self.tf_tree.transform(best_grasp, 'camera_color_optical_frame', 'mz04_link0')
        print('T_base_obj', T_base_obj, cls)
        T_base_obj = Transform.from_matrix(T_base_obj)
        return T_base_obj, cls

    def execute_grasp(self, grasp):
        rospy.loginfo("execute grasp")
        T_base_grasp = grasp

        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp
        T_base_retreat = T_grasp_retreat * T_base_grasp

        rospy.loginfo("go to grasp pre pose")
        self.mc.goto_joint_pose(T_base_pregrasp)
        rospy.loginfo("go to grasp pose")
        self.mc.goto_joint_pose(T_base_grasp)
        rospy.loginfo("return to grasp pre pose")
        self.mc.goto_joint_pose(T_base_retreat)

    def drop_pet(self):
        goals = np.pi / 180. * np.array([60.01, -18.9, -18.20, 0.0, -52.9, 60.01])
        self.mc.goto_joints(goals)
        rospy.loginfo("finish drop pet")

    def drop_metal(self):
        goals = np.pi / 180. * np.array([60.01, -18.9, -18.20, 0.0, -52.9, 60.01])
        self.mc.goto_joints(goals)
        rospy.loginfo("finish drop pet")

    def plan_trajectories(self, grasp, cls='metal'):
        rospy.loginfo("execute plan trajectories")
        T_base_grasp = grasp
        initial_pose = Transform.from_list([0.14693, -0.00002, 0.38182, 1, 0, 0, 0])
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp
        T_base_retreat = T_grasp_retreat * T_base_grasp
        # T_base_place = self.select_place_pose(cls)
        T_base_place = Transform.from_list([0.161256, 0.279421, 0.165748, 1, 0, 0, 0])
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

    def execute_trajectory(self, trajectory):
        rospy.loginfo("execute trajectory")
        self.mc.goto_many_poses(trajectory)
        rospy.loginfo("finish trajectory")

    def run_cartesian(self):
        rospy.loginfo("start grasp loop")
        cnt = 10
        while not rospy.is_shutdown() and cnt > 0:
            grasp, cls = self.get_grasp()
            cls = 'metal'
            trajectory1, trajectory2 = self.plan_trajectories(grasp, cls)
            self.execute_trajectory(trajectory1)
            self.mc.move_gripper(width=0, force=20)
            self.execute_trajectory(trajectory2)
            self.mc.move_gripper(width=0.08, force=0)
            cnt -= 1
        rospy.loginfo("finish grasp loop")

    def run_joint(self):
        rospy.loginfo("start grasp loop")
        # self.mc.robot_begin()
        cnt = 2
        while not rospy.is_shutdown() and cnt > 0:
            grasp, cls = self.get_grasp()
            self.execute_grasp(grasp)
            self.drop_pet()
            cnt -= 1
        rospy.loginfo("finish grasp loop")


def main():

    rospy.init_node("nachi_arm_grasp")
    print('arm is starting')
    arm_grasp = NachiGraspController()
    raw_input('please press enter to start grasp loop')
    # arm_grasp.run_joint()
    arm_grasp.run_cartesian()


if __name__ == "__main__":
    main()
