#!/usr/bin/env python
from __future__ import print_function


import rospy
from roport.srv import *
import numpy as np
from utils import ros_utils
from utils.dual_arm_control_v2 import DualArmCommander
from utils.transform import Rotation, Transform
from geometry_msgs.msg import PoseStamped, PoseArray
import time
import copy
from segmentation_srv.msg import SegmentationInfo

class LeftArmController(object):
    """Nachi robot grasp Control"""

    def __init__(self):
        super(LeftArmController, self).__init__()
        self.mc = DualArmCommander()

        self.T_rbase_camera = Transform.from_list(
            [0.394331, 0.73786, 0.683657, 0.999848, -0.00577672, -0.00203533, -0.0163113])
        d = 0.9306831
        # self.T_lbase_rbase = Transform(Rotation.identity(), [-0.02, d, 0])
        self.T_lbase_rbase = Transform(Rotation.identity(), [0.0, d, 0])
        self.T_lbase_camera = self.T_lbase_rbase * self.T_rbase_camera

        self.T_rbase_camera = self.T_rbase_camera.as_matrix()
        self.T_lbase_camera = self.T_lbase_camera.as_matrix()
        print(self.T_lbase_camera)
        assert 1 <0

        self.conveyor_speed = rospy.get_param('/conveyor_speed')
        # self.robot_spped = 1.4
        self.robot_spped = 0.66  # 0.6  # 60% 0.6
        # self.conveyor_speed = 0.092  # level 1
        self.group_name = 'left_arm'
        self.wait_y_pos = 0.15
        self.end_y_pos = -0.2
        self.wait_z = 0.27
        self.reset_x = 0.34
        self.reset_y = self.wait_y_pos
        self.reset_z = 0.28
        self.grasp_x_range = [0.18, 0.44]
        self.press_depth = 0.0055  # 0.0075  max press depth 0.009
        self.t_top_down_grasp = 0.26  # 0.4  # 60% 0.26
        self.t_pre_wait_t = 0.7
        self.tracking_gripper_time = 0.40
        self.min_z = 0.178   # 0.177 0.17634
        # self.z_error = 0.027  # cause collision
        self.z_error = 0.024  # 0.023 0.0178
        self.y_margin = 0.038
        self.T_base_reset = Transform.from_list([self.reset_x, self.reset_y, self.reset_z, 0, 1, 0, 0])
        self.robot_running_state = True
        if self.press_depth > 0.009 or self.min_z < 0.17634:
            assert 1<0, 'press depth is wrong'
        rospy.loginfo("Ready to take action")

    def execute_nachi_robot_trajectory(self, group_name, trajectory_lengths, points):
        rospy.loginfo("execute nachi cartesian trajectory")
        self.mc.robot_go(group_name, trajectory_lengths, points)

    def plan_execute_nachi_robot_trajectory(self, group_name, pick_traj_1, pick_and_place_traj_2):
        pick_plan = self.mc.build_cart_path(group_name, pick_traj_1)
        pick_and_place_plan = self.mc.build_cart_path(group_name, pick_and_place_traj_2)
        pick_plan_len = len(pick_plan.joint_trajectory.points)
        pick_and_place_plan_len = len(pick_and_place_plan.joint_trajectory.points)
        print('pick and palce plan len are', pick_plan_len, pick_and_place_plan_len)
        trajectory_lengths = [pick_plan_len, pick_and_place_plan_len]  # pick_and_place_plan the whole trajectory
        points = pick_and_place_plan.joint_trajectory.points
        self.execute_nachi_robot_trajectory(group_name, trajectory_lengths, points)

    def is_pose_normal(self, grasp):
        ori = grasp.orientation
        ori_norm = np.sqrt(ori.x ** 2 + ori.y ** 2 + ori.z ** 2 + ori.w ** 2)
        if ori_norm > 0:
            T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
            T_lbase_obj = np.dot(self.T_lbase_camera, T_camera_obj)

            x, y, z = T_lbase_obj[0, 3], T_lbase_obj[1, 3], T_lbase_obj[2, 3]
            ok_region = (self.grasp_x_range[0] < x < self.grasp_x_range[1]) and (1.2 < y < 2) and (0.17 < z < 0.24)
            if ok_region:
                return True
            else:
                return False
        else:
            return False

    def plan_tracking_trajectories(self, group_name, grasp, label, meeting_point_y=None):
        rospy.loginfo("execute plan shift trajectories")
        T_base_grasp = grasp
        T_base_grasp.translation[1] = self.wait_y_pos  # wait y pos
        if meeting_point_y is not None:
            T_base_grasp.translation[1] = meeting_point_y

        T_base_grasp.rotation = Rotation.from_quat([0, 1, 0, 0])  # don't rotate
        T_grasp_pregrasp = Transform(Rotation.identity(), [0.0, 0.0, 0.1])
        T_grasp_retreat = Transform(Rotation.identity(), [0.0, 0.0, 0.1])  # global frame Z axis aim up, so z is 0.05
        press_depth = self.press_depth
        dz = 0.001
        if dz > 0.002:
            assert 1 < 0
        if label == 'can':
            press_depth = self.press_depth + dz
        T_base_grasp = T_base_grasp * Transform(Rotation.identity(), [0, 0, press_depth])  # press 1 mm along z axis
        T_base_middle_place, T_base_place = self.select_place_pose(T_base_grasp.rotation, group_name, label)
        T_shift_grasp = Transform(Rotation.identity(), [0.0, -self.conveyor_speed * self.tracking_gripper_time, 0])
        T_base_shift_grasp = T_shift_grasp * T_base_grasp
        T_base_pregrasp = T_grasp_pregrasp * T_base_grasp  # global frame
        T_base_retreat = T_grasp_retreat * T_base_shift_grasp  # global frame

        T_place_shift = Transform(Rotation.identity(), [0.2, 0.0, 0.0])
        T_base_release_gripper = T_place_shift * T_base_place
        if meeting_point_y is not None:
            # all 8 points
            trajectory1 = [T_base_pregrasp, T_base_grasp, T_base_shift_grasp]
            trajectory2 = [T_base_retreat, T_base_middle_place, T_base_place, self.T_base_reset]
        else:
            # all 7 points
            trajectory1 = [T_base_grasp, T_base_shift_grasp]
            trajectory2 = [T_base_retreat, T_base_middle_place, T_base_place, self.T_base_reset]
        rospy.loginfo("finish plan trajectories")
        return trajectory1, trajectory2

    def select_place_pose(self, rotation, group_name, cls):
        """local frame"""
        translation = [-0.20, 0.27, self.reset_z]
        middle_translation = [0.22, 0.23, self.reset_z]

        if cls == 'pcb':
            translation = translation
        elif cls == 'can':
            translation[0] = translation[0] + 0.04
            translation[1] += 0.08
        place_rotation = Rotation.from_quat([0, 1, 0, 0])

        T_base_middle = Transform(place_rotation, middle_translation)
        T_base_place = Transform(place_rotation, translation)

        return T_base_middle, T_base_place

    def execute_cart_trajectory(self,  group_name, trajectory):
        rospy.loginfo("execute cartesian trajectory")
        self.mc.goto_many_poses(group_name, trajectory)
        rospy.loginfo("finish cartesian trajectory")

    def build_cart_path(self, group_name, trajectory):
        # rospy.loginfo("execute cartesian trajectory")
        plan = self.mc.build_cart_path(group_name, trajectory)
        plan_points = plan.joint_trajectory.points
        joints_angle_deg = np.zeros((len(plan_points), 6))
        for i, point in enumerate(plan_points):
            joints_angle_deg[i] = plan_points[i].positions
        # joints_angle_deg = joints_angle_deg * 180.0 / np.pi
        # joints_angle_deg[:, 1] += 90.0
        trans = []
        print('joints_angle_deg are', joints_angle_deg.shape[0], joints_angle_deg)
        for i in range(joints_angle_deg.shape[0]):
            pose = self.mc.moveit_fk(group_name, joints_angle_deg[i])
            trans.append(ros_utils.from_pose_msg(pose))
        return trans

    def add_collision_object(self):
        self.mc.add_collision_box('left_arm', 'table', Transform.from_list([-0.33, 0.48, -0.085, 0, 0, 0, 1]),
                                  [0.9, 1.2, 0.1])
        self.mc.add_collision_box('left_arm', 'convey belt', Transform.from_list([0.42, 0.6, -0.324, 0, 0, 0, 1]),
                                  [0.6, 3, 1])

    def move_to_wait(self, T_lbase_obj):
        group_name = 'left_arm'
        wait_pose = Transform.identity()
        wait_pose.translation = [T_lbase_obj.translation[0], self.wait_y_pos, self.wait_z]
        wait_pose.rotation = Rotation.from_quat([0, 1, 0, 0])
        print('wait pose trans is ', wait_pose.translation)
        trajectory = [wait_pose]
        self.execute_cart_trajectory(group_name, trajectory)
        rospy.loginfo("finish move wait cartesian trajectory")

    def wait_picking(self, T_lbase_obj, img_time_stamp):
        t_margin = self.t_top_down_grasp
        dis = T_lbase_obj.translation[1]-self.wait_y_pos
        wait_time = dis / self.conveyor_speed
        t = wait_time - t_margin
        dt = rospy.Duration(secs=t)
        print('dis is', dis, 't is', t, 'dt is', dt)
        rospy.loginfo("wait for object arrive")
        while rospy.Time.now()-img_time_stamp < dt:
            rospy.sleep(0.01)
        rospy.loginfo("object arrive, start to pick")

    def move_to_reset(self):
        group_name = 'left_arm'
        reset_pose = Transform.from_list([self.reset_x, self.reset_y, self.reset_z, 0, 1, 0, 0])
        trajectory = [reset_pose]
        self.execute_cart_trajectory(group_name, trajectory)
        rospy.loginfo("finish move reset pose")

    def activate_gripper(self, group_name):
        if group_name == 'right_arm':
            self.mc.robotiq_2f_gripper_control(0, max_effort=20)  # grasp
        else:
            self.mc.robotiq_vacuum_gripper_control(1, mod=1)  # grasp

    def release_gripper(self, group_name):
        if group_name == 'right_arm':
            self.mc.robotiq_2f_gripper_control(0.05, max_effort=20)  # half width
        else:
            self.mc.robotiq_vacuum_gripper_control(0)

    def init_robot(self):
        self.add_collision_object()
        # assert 1<0
        self.mc.robotiq_vacuum_gripper_control(0)  # release
        rospy.loginfo('finish init')

    def compute_meeting_point(self, p0, p1):
        # robot start from still, need accelerate time.
        # the whole motion contain accel and constant vel period, so add t_margin or y_margin

        v0 = self.robot_spped
        v1 = self.conveyor_speed
        dx = p1[0] - p0[0]
        dy = p0[1] - p1[1]
        dh = p0[2] - p1[2]
        a = v0 ** 2 - v1 ** 2
        b = -2 * (dh * v0 * v1 + dy * v1 ** 2)
        c = -v1 ** 2 * (dx ** 2 + dy ** 2 - dh ** 2)
        coeff = [a, b, c]
        d_root = np.roots(coeff)
        if np.max(d_root) > 0:
            dis = np.max(d_root)
            meeting_point_y = p1[1] - dis
            print('d root is', dis, 'meeting y is', meeting_point_y)
            return meeting_point_y - self.y_margin
        else:
            rospy.logerr('no solution for meeting point')
            return None

    def is_object_out_of_region(self, mat_grasp, img_time_stamp):
        pre_wait_y_pos = self.wait_y_pos + self.conveyor_speed * self.t_pre_wait_t  # 0.224m
        dis = mat_grasp[1, 3] - self.end_y_pos
        wait_time = dis / self.conveyor_speed
        t = wait_time
        dt = rospy.Duration(secs=t)
        print('dis is', dis, 't is', t, 's')
        if rospy.Time.now()-img_time_stamp > dt:
            rospy.logerr('object has out of grasp region')
            return False, None
        dt = (rospy.Time.now() - img_time_stamp)
        cur_y = mat_grasp[1, 3] - dt.to_sec() * self.conveyor_speed
        robot_reset_point = [self.reset_x, self.reset_y, self.reset_z]
        object_start_point = [mat_grasp[0, 3], cur_y, mat_grasp[2, 3]]
        meeting_point_y = self.compute_meeting_point(robot_reset_point, object_start_point)
        end_y_pos = self.end_y_pos
        if 0.35 < mat_grasp[0, 3] < 0.4:
            end_y_pos = -0.13
        if 0.4 <= mat_grasp[0, 3] < self.grasp_x_range[1]:
            end_y_pos = -0.06
        if meeting_point_y is None or meeting_point_y < end_y_pos:
            rospy.logerr('no solution or meeting_point_y have passed end line')
            return False, None
        elif self.end_y_pos <= meeting_point_y and cur_y <= pre_wait_y_pos:
            if cur_y <= self.wait_y_pos:
                rospy.loginfo('object is in grasp region right now')
            rospy.loginfo('object in pre grasp region right now')
            return True, meeting_point_y
        elif cur_y > pre_wait_y_pos:
            rospy.loginfo('object have not arrive grasp region')
            return True, None

    def do_picking(self, req):
        cls = req.class_name
        pose_msg = req.object_pose
        grasp = pose_msg.pose  # grasp pose msg
        ok = self.is_pose_normal(grasp)
        if not ok:
            rospy.logerr('detect pose is wrong')
            return
        img_time_stamp = pose_msg.header.stamp
        tic = time.time()
        T_camera_obj = ros_utils.from_pose_msg(grasp).as_matrix()
        mat_grasp = np.dot(self.T_lbase_camera, T_camera_obj)
        ok_grasp, meeting_point_y = self.is_object_out_of_region(copy.deepcopy(mat_grasp), img_time_stamp)
        if not ok_grasp:
            return
        print('origin grasp trans is', mat_grasp[:, 3])
        z_value = copy.deepcopy(mat_grasp[2, 3])
        mat_grasp[2, 3] = mat_grasp[2, 3] - self.z_error  # adjust object z value may be 0.022
        now_z = copy.deepcopy(mat_grasp[2, 3])
        min_z = self.min_z
        if mat_grasp[2, 3] < min_z:
            mat_grasp[2, 3] = min_z  # z
            rospy.loginfo('z %.8f is wrong, error is %.8f', now_z, min_z-z_value)
        print('z in camera frame is', T_camera_obj[2, 3], 'grasp z in local is', mat_grasp[2, 3])

        T_lbase_obj = Transform.from_matrix(mat_grasp)
        print('T_lbase_obj is', T_lbase_obj.translation)
        group_name = self.group_name
        T_lbase_obj_cpoy = copy.deepcopy(T_lbase_obj)
        # print('phase 1 time is', time.time()-tic)
        # assert 1<0, print('stop')
        if meeting_point_y is not None:
            tic = time.time()
            trajectory1, trajectory2 = self.plan_tracking_trajectories(group_name, T_lbase_obj_cpoy, cls,
                                                                       meeting_point_y)
            # print('meeting point phase 2 time is', time.time() - tic)
            self.plan_execute_nachi_robot_trajectory(group_name, trajectory1, trajectory1 + trajectory2)
            rospy.loginfo("finish grasp 1 object in grasp region")
        else:
            self.move_to_wait(T_lbase_obj)
            self.wait_picking(T_lbase_obj, img_time_stamp)
            tic = time.time()
            trajectory1, trajectory2 = self.plan_tracking_trajectories(group_name, T_lbase_obj_cpoy, cls,
                                                                       meeting_point_y)
            # print('phase 2 time is', time.time() - tic)
            self.plan_execute_nachi_robot_trajectory(group_name, trajectory1, trajectory1 + trajectory2)
            rospy.loginfo("finish grasp 1 object in waiting mode")

    # def pos_test(self):
    #     self.add_collision_object()
    #     # assert 1<0
    #     group_name = self.group_name
    #     cls = 'pcb'
    #     # self.move_to_reset()
    #     sim_pose = Transform.from_list([0.3, 0.15, 0.2, 0, 1, 0, 0])
    #     trajectory1, trajectory2 = self.plan_trajectories_v2(group_name, sim_pose, cls)
    #     tic = time.time()
    #     self.execute_cart_trajectory(group_name, trajectory1)
    #     self.execute_cart_trajectory(group_name, trajectory2)
    #     self.move_to_reset()
    #     end = time.time()-tic
    #     print('time is', end)
    #     rospy.loginfo("finish grasp 1 object in waiting mode")

    def cart_srv_test(self):
        self.add_collision_object()
        # assert 1<0
        group_name = self.group_name
        cls = 'can'
        # self.move_to_reset()
        tf = ros_utils.TransformTree()
        sim_pose = Transform.from_list([0.3, 0.15, 0.2, 0, 1, 0, 0])
        # trajectory1, trajectory2 = self.plan_trajectories_v2(group_name, sim_pose, cls)
        trajectory1, trajectory2 = self.plan_tracking_trajectories(group_name, sim_pose, cls)
        traj1 = self.build_cart_path(group_name, trajectory1)
        print(len(traj1))
        # self.build_cart_path(group_name, trajectory2)
        trajectory3 = trajectory1 + trajectory2
        trans = self.build_cart_path(group_name, trajectory3)
        while not rospy.is_shutdown():
            for i, tran in enumerate(trans):
                tf.broadcast_static(tran, 'arm_L_link0', 'point'+str(i+1))
            rospy.sleep(1)

    def tracking_fun_test(self):
        self.add_collision_object()
        # assert 1<0
        group_name = self.group_name
        cls = 'pcb'
        sim_pose = Transform.from_list([0.34, 0.15, 0.215, 0, 1, 0, 0])
        trajectory1, trajectory2 = self.plan_tracking_trajectories(group_name, sim_pose, cls)
        self.plan_execute_nachi_robot_trajectory(group_name, trajectory1, trajectory1 + trajectory2)
        rospy.loginfo("finish test")


def main():
    rospy.init_node("nachi_left_arm_grasp")
    arm_grasp = LeftArmController()
    raw_input('please press enter to start grasp loop')
    # arm_grasp.init_robot()
    rospy.loginfo("start grasp")
    rospy.Subscriber('/sorting_line/robot2', SegmentationInfo, arm_grasp.do_picking)
    rospy.spin()

# def main():
#     rospy.init_node("nachi_left_arm_grasp")
#     arm_grasp = LeftArmController()
#     raw_input('please press enter to start grasp loop')
#     # arm_grasp.vel_test()
#     # arm_grasp.pos_test()
#     # arm_grasp.shift_trajectories_test()
#     # arm_grasp.cart_srv_test()
#     arm_grasp.tracking_fun_test()


if __name__ == "__main__":
    main()
