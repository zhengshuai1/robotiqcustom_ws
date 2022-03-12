import numpy as np
from utils.transform import Rotation, Transform
import torch
from scipy.spatial.transform import Rotation as R
import rospy
import copy

class Test(object):
    def __init__(self):
        super(Test, self).__init__()
        self.num = 0

def test_dis():
    max_dis = 0.46
    max_x = 0.4
    y = np.sqrt(max_dis**2 - max_x**2)
    print('y is', y)
def test_rotation():
    rot = R.from_quat([0.923868, 0.38271, 0, 0])
    rot_euler = rot.as_euler('zyx', degrees=True)
    print(rot_euler)

def compute_meeting_point(p0, p1):
    v0 = 0.24
    v1 = 0.092
    dx = p1[0]-p0[0]
    dy = p0[1]-p1[1]
    dh = p0[2] - p1[2]
    a = v0**2 - v1**2
    b = -2*(dh*v0*v1 + dy * v1**2)
    c = -v1**2*(dx**2+dy**2-dh**2)
    coeff = [a, b, c]
    det = b**2 - 4*a*c
    print('a b c are', a, b, c, det)
    d_root = np.roots(coeff)
    # [p1[0], p1[1] +]
    print('root is', d_root, 'meeting point is')
    d_est = (dh+dy)*v1/(v0-v1)
    print('dest is', d_est)

def test_v1():
    qz = -0.382683
    theta = 2 * np.arcsin(qz)
    print(np.rad2deg(theta))







def main():
    test_v1()



    test_rotation()
    test_dis()
    a = Test()
    b = a.num
    b = b+1
    print('test', a.num, b)
    a = {1: [1, 2, 3]}
    b = a.copy()
    a[1].append(4)
    print(a, b)
    a = [1, 2, 3, 4, ['a', 'b']]

    b = a
    c = copy.copy(a)
    d = copy.deepcopy(a)

    a.append(5)
    a[4].append('c')

    print('a = ', a)
    print('b = ', b)
    print('c = ', c)
    print('d = ', d)
    # cal trransform
    # T_rbase_camera = Transform.from_list(
    #     [0.373978, 0.478391, 0.873961, 0.999962, 0.00261615, -0.000696031, -0.00824645])
    # T_rbase_camera = Transform.from_list(
    #     [0.369171, 0.492295, 0.869221, 0.999956, 0.00506887, 0.00501505, - 0.00613132])
    T_rbase_camera = Transform.from_list(
        [0.437882, 0.787201, 0.774616, 0.99988, 0.00158493, 0.0116128, -0.0101244])
    print('T_rbase_c  is \n', T_rbase_camera.as_matrix())
    d = 0.9306831
    np.set_printoptions(suppress=True)
    T_lbase_rbase = Transform(Rotation.identity(), [0.0, d, 0])
    T_lbase_camera = T_lbase_rbase * T_rbase_camera
    print('t is', T_lbase_camera.as_matrix())

    p1 = np.array([-0.01, -0.03, 0, 1]).reshape(4, 1)
    p2 = np.array([0, 0, 0, 1]).reshape(4, 1)
    dp = np.dot(T_lbase_camera.as_matrix(), (p2-p1))
    print('dp is', dp)


if __name__ == "__main__":
    main()



# rospy.init_node("test")
# p0 = [0.3, 0.2, 0.06]
# p1 = [0.25, -0.2, 0.0]
# compute_meeting_point(p0, p1)
# print('y is', np.sqrt(0.5**2- 0.22**2))
# t_margin = 0.3
# conveyor_speed = 0.035
# now = rospy.get_rostime()
# rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
# print(now.secs, now.nsecs)
# # grasp pos y 0.27 m
# y = 1.607
# dis = y - 0.27
# wait_time = dis / conveyor_speed
# t = wait_time - t_margin
# print('t is', t, 's')
# tic = rospy.Time.now()
# dt = rospy.Duration.from_sec(t/10.0)
# cnt = 0
# print(not None)
# # while rospy.Time.now()- tic < dt:
# #     cnt+=1
# dur = rospy.Time.now()- tic
# print('cnt is', cnt, tic, dt, dur.to_sec())
# classes = ['a', 'b']
# c = 'robot1'
# print([c]*len(classes))
# print(dict(zip(classes, [c]*len(classes))))
# theList = ['a', 'b', 'c']
# cls = 'a'
# if cls in theList:
#     print('true')
# a = np.array(range(6))
# print(a * -1.0)
# angle = 88/180.0*np.pi
# print(np.tan(angle), 1.4743*180/np.pi)
# d = np.array([1, 2, 0.2])
# a= np.array([1, 2, 3.0]).reshape(1,3)
# b = np.r_[a, a, a]
# i =2
#
# def transform_test():
#     # 0.380423 0.76155 0.679691   0.999101 0.039539 -0.00364944 -0.0148792 arm_R_link0 camera_color_optical_frame
#     t1 = Transform.from_list([0.380423, 0.76155, 0.679691, 0.999101, 0.039539, -0.00364944, -0.0148792])
#     t2 = Transform.from_list([-0.000, 0.015, 0.000, -0.499, 0.497, -0.500, 0.503])
#     cal_t1 = t1 * t2.inverse()
#     cal_t1_list = cal_t1.to_list()
#     T_l_r = Transform.from_list([0, 0.9306831, 0, 0, 0, 0, 1])
#     T_l_c = T_l_r * cal_t1
#     print('T_l_c is', T_l_c.to_list(), 't_r_c is', cal_t1_list)
# transform_test()
# print(b[:, i] * d[i])
# b[:, i] = b[:, i] * d[i]
# print(d[i].shape, 'val is', b)
# print(a.shape, b.shape, b)
# x, y, z = [0.28, 0.24, 0.01]
# w, l, h = [0.2, 0.3, 0.1]
# ok = -l < x < l and -w < y < w
# print('ok is', ok)
# a = 9
# b = np.clip(a, 1, 8)
# print(b, type(b))
# a = np.array([[0.6, 1, 0.0, 0.0],
#               [0.0, 0.4, 0.0, 0.0],
#               [0.0, 0.0, 1.2, 1.0]])
# a = np.array(range(16)).reshape(4, 4, 1)
# indx = np.array([0, 1,2])
# indy = np.array([0, 1,2])
# print(a[2, 2], a[indx, indy], a[indx, indy].shape, a[:,:,0].shape)
# # y, x = np.where(a == 8)
# # print('ind is', np.where(a == 8))
# # print(y, x.shape, x.squeeze().shape, x.squeeze()*a)
# # b= np.where(a>0)
# # a=np.array([0, 0, 1]).reshape(1,3)
# # b=np.array([0.707, 0, -0.707]).reshape(1, 3)
# # # rot = R.align_vectors(a, b)
# # print(rot[0].as_matrix())
# # x = np.arange(6).reshape(2,3)
# # a = np.argwhere(x>1)
# # # y, x= np.where(x>1)
# # print(a)
# # # print(a, y, x)
# # b= np.where(x>1)
# # # c=np.hstack((y.T, x.T))
# # print(np.transpose(b))
# # cnt = [1, 1]
# # print(np.array([y,x]).T)
# # for i in range(a.shape[0]):
# #     if a[i, :] == cnt:
# #         print(i)
# #         break
# # print('cnt', np.where(a == cnt))
#
# # depth = np.array(range(16)).reshape(4, 4)
# # print(b, b[0].shape[0], depth, depth[b], depth[b].shape)
# # d = np.vstack((b[0], b[1], np.ones_like(b[0])))
# # print(d)
#
# # e= np.array([range(1,5)])
# # print(np.multiply(e, a))
# # d= np.array(range(3)).reshape(3, 1).squeeze()
# # x=d[0]
# # print(x, d.shape)
#
# ############################################
# # r = R.from_matrix(np.eye(3))
# # print(r.as_quat())
# # r1 = R.from_quat([1, 0, 0, 0])
# # print(r1.as_matrix())
# # r2 = R.from_euler('YZ', [45, 45], degrees=True)
# # # r2 = R.from_euler('y', 45, degrees=True)
# # r = r1*r2
# # print('r is', r.as_matrix())
# # # r3 = R.from_euler('z', 45, degrees=True)
# # # r = r*r3
# # # print('r is', r.as_matrix())
# # r4 = R.from_euler('XYZ', [180, 45, 45], degrees=True)
# #
# # print('r4 is', r4.as_matrix())
#
# # a = torch.tensor([1,5,2,6,7])
# # b,a_index = a.sort(descending = True)
# #
# # print(a[a_index], b)
#
# ##########
# x = np.array([-1.2, 1.2])
# print(np.abs(x))
# def tf_transform():
#     T_lbase_cam = Transform.from_list([0.487863, 0.0536102, 0.798249, 0.717054, 0.697018, -9.05588e-05, 0.000254905])
#     T_rbase_cam = Transform.from_list([0.440856, -0.0448193, 0.794483, -0.699114, 0.714996, -0.00406471, -0.00212906])
#     T_rbase_lbase = T_rbase_cam * T_lbase_cam.inverse()
#     T_lbase_rbase = T_lbase_cam * T_rbase_cam.inverse()
#     print('T_rbase_lbase is', T_rbase_lbase.as_matrix(), T_lbase_rbase.as_matrix())
#
# tf_transform()
# # 0.309045 -0.0104731 0.982099 0.999115 -0.00655622 -0.0143316 -0.0389965 /arm_R_link0 /camera_color_optical_frame
# t1 = Transform.from_list([0.309045, -0.0104731, 0.982099, 0.999115, -0.00655622, -0.0143316, -0.0389965])
# t2 = Transform.from_list([-0.000, 0.015, 0.000, -0.499, 0.497, -0.500, 0.503])
# t3 = Transform.from_list([0.421765, -0.0368384, 1.00711, -0.702706, 0.711457, 0.0049529, -0.00278442])
# t4 = Transform.from_list([0.440856, -0.0448193, 0.794483, -0.699114, 0.714996, -0.00406471, -0.00212906])
# t5 = Transform.from_list([0.40914, 0.647593, 0.68488, 0.999948, -0.00543547, -0.00286972, -0.00813957])
# cal_t1 = t1 * t2.inverse()
# cal_t2 = t3 * t2.inverse()
# cal_t4 = t4 * t2.inverse()
# cal_t5 = t5 * t2.inverse()
# cal_t2_list = cal_t2.to_list()
# cal_t4_list = cal_t4.to_list()
# cal_t5_list = cal_t5.to_list()
# T_l_r = Transform.from_list([0, 0.9306831, 0, 0, 0, 0, 1])
# T_l_c = T_l_r * cal_t5
# new = T_l_r * t5
# print('T_l_c is', T_l_c.to_list(), new.to_list())
# # print('cal 6.30 is', t3.as_matrix(), 'cal t1 is ', cal_t1.as_matrix(), 'cal t2 is ', cal_t2.as_matrix())
# print('cal 7.10 is', t4.as_matrix(), 'cal t1 is ', cal_t1.as_matrix(), 'cal t2 is ', cal_t2.as_matrix())
# print('cal_t5_list is', cal_t5_list)
#
# a= np.array(range(16)).reshape(4,4)
# print(a[0:2, 2])
#
# #
# # arr = np.array([[1, 2, 3], [4, 5, 6]])
# # for x in arr[:, ]:
# #     print(x)
#
# # x = torch.arange(1., 6.)
# # b = torch.topk(x, 3)
# # print(b[1])
# # a= np.array(range(16)).reshape(4, 4)
# # print(a[0:3, 3], a[0:3, 3].shape)
# # distance = np.sqrt(0.35**2 + 0.25**2 + 0.15**2)
# # print(distance)
# a = np.array([2.0942130088806152, -2.7013107867337856, 4.894871938253412, 2.21678912])
# print ('degree is ', 180 / np.pi * a)
# print('degree is', np.pi * (125.0/180.0))
# #
# # arr = np.array([1, 3, 2, 4, 5])
# # for i, ar in enumerate(arr):
# #     print(i, ar)
# # arr = arr.argsort()[-3:][::-1]
# # print(arr)
# # array([4, 3, 1])
# # left_goal = Transform.from_list([0.2, 0.02, 0.25, 0, 1, 0, 0])
# # right_goal = Transform.from_list([0.4, 0.02, 0.25, 1, 0, 0, 0])
# # a= [left_goal.as_matrix(), right_goal.as_matrix()]
# # print(a[::-1])
# n=4
# k = 5 if n > 5 else n
# print(k)