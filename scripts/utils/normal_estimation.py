# examples/Python/Basic/pointcloud.py

import numpy as np
import open3d as o3d
import copy

import rospy
import struct
# from ros_utils import to_cloud_msg
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

if __name__ == "__main__":
    rospy.init_node("create_cloud_xyzrgb")
    pub = rospy.Publisher("my_point_cloud2", PointCloud2, queue_size=2)


    pcd = o3d.io.read_point_cloud("/home/hp/Desktop/experiment figures/test.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    points = np.asarray(pcd.points)
    # print(points.shape)
    # o3d.visualization.draw_geometries([pcd])
    ros_cloud = convertCloudFromOpen3dToRos(pcd, frame_id='map')
    o3d_cloud = convertCloudFromRosToOpen3d(ros_cloud)
    o3d.io.write_point_cloud("/home/hp/nachi_ws/1.pcd", o3d_cloud)
    while not rospy.is_shutdown():
        pub.publish(ros_cloud)
        rospy.loginfo("Conversion and publish success ...\n")
        rospy.sleep(2)

    #     pc2.header.stamp = rospy.Time.now()
    #     pub.publish(pc2)
    #     rospy.sleep(1.0)







    # # print("Downsample the point cloud with a voxel of 0.05")
    # downpcd = pcd.voxel_down_sample(voxel_size=0.01)
    # # o3d.visualization.draw_geometries([downpcd])
    # #
    # # downpcd = pcd
    # print("Recompute the normal of the downsampled point cloud")
    # downpcd.estimate_normals(
    #     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
    # # o3d.visualization.draw_geometries([downpcd], point_show_normal=True)
    #
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals).shape, np.asarray(downpcd.normals)[:10, :])

    # plane_model, inliers = pcd.segment_plane(distance_threshold=0.02,
    #                                          ransac_n=3,
    #                                          num_iterations=1000)
    # [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    #
    # inlier_cloud = pcd.select_by_index(inliers)
    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # outlier_cloud = pcd.select_by_index(inliers, invert=True)
    # o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

    # x = np.linspace(-3, 3, 401)
    # mesh_x, mesh_y = np.meshgrid(x, x)
    # z = np.sinc((np.power(mesh_x, 2) + np.power(mesh_y, 2)))
    # z_norm = (z - z.min()) / (z.max() - z.min())
    # xyz = np.zeros((np.size(mesh_x), 3))
    # xyz[:, 0] = np.reshape(mesh_x, -1)
    # xyz[:, 1] = np.reshape(mesh_y, -1)
    # xyz[:, 2] = np.reshape(z_norm, -1)
    # print(f"xyz, shape is {x.shape[0]: d}")
    # print(xyz)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(xyz)
    # o3d.visualization.draw_geometries([pcd])

    # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # T = np.eye(4)
    # T[:3, :3] = mesh.get_rotation_matrix_from_xyz((np.pi, np.pi / 4, np.pi / 4))
    # T[0, 3] = 1
    # T[1, 3] = 1.3
    # print(T)
    # mesh_t = copy.deepcopy(mesh).transform(T)
    # o3d.visualization.draw_geometries([mesh, mesh_t])