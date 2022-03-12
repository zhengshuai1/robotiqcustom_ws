from __future__ import print_function

import geometry_msgs.msg
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import tf2_ros

from utils.transform import Rotation, Transform


def to_point_msg(position):
    """Convert numpy array to a Point message."""
    msg = geometry_msgs.msg.Point()
    msg.x = position[0]
    msg.y = position[1]
    msg.z = position[2]
    return msg


def from_point_msg(msg):
    """Convert a Point message to a numpy array."""
    return np.r_[msg.x, msg.y, msg.z]


def from_pose_msg(msg):
    """Convert a Pose message to a Transform object."""
    transform = Transform.identity()
    # transform.translation = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    transform.translation = [msg.position.x, msg.position.y, msg.position.z]
    transform.rotation = from_quat_msg(msg.orientation)
    return transform

def to_vector3_msg(vector3):
    """Convert numpy array to a Vector3 message."""
    msg = geometry_msgs.msg.Vector3()
    msg.x = vector3[0]
    msg.y = vector3[1]
    msg.z = vector3[2]
    return msg


def from_vector3_msg(msg):
    """Convert a Vector3 message to a numpy array."""
    return np.r_[msg.x, msg.y, msg.z]


def to_quat_msg(orientation):
    """Convert a `Rotation` object to a Quaternion message."""
    quat = orientation.as_quat()
    msg = geometry_msgs.msg.Quaternion()
    msg.x = quat[0]
    msg.y = quat[1]
    msg.z = quat[2]
    msg.w = quat[3]
    return msg


def from_quat_msg(msg):
    """Convert a Quaternion message to a Rotation object."""
    return Rotation.from_quat([msg.x, msg.y, msg.z, msg.w])


def to_pose_msg(transform):
    """Convert a `Transform` object to a Pose message."""
    msg = geometry_msgs.msg.Pose()
    msg.position = to_point_msg(transform.translation)
    msg.orientation = to_quat_msg(transform.rotation)
    return msg


def to_posearray_msg(transforms):
    """Convert many `Transform` objects to a PoseArray message."""
    posearray_msg = geometry_msgs.msg.PoseArray()
    # print("length of transforms is", len(transforms))
    for transform in transforms:
        pose_msg = to_pose_msg(transform)
        # print('translation is', transform.translation, 'rotation is', transform.rotation)
        posearray_msg.poses.append(pose_msg)
    # print('poses are:', posearray_msg.poses)
    # print("length of posearray is", len(posearray_msg.poses))
    return posearray_msg


def to_transform_msg(transform):
    """Convert a `Transform` object to a Transform message."""
    msg = geometry_msgs.msg.Transform()
    msg.translation = to_vector3_msg(transform.translation)
    msg.rotation = to_quat_msg(transform.rotation)
    return msg


def from_transform_msg(msg):
    """Convert a Transform message to a Transform object."""
    translation = from_vector3_msg(msg.translation)
    rotation = from_quat_msg(msg.rotation)
    return Transform(rotation, translation)


def to_color_msg(color):
    """Convert a numpy array to a ColorRGBA message."""
    msg = std_msgs.msg.ColorRGBA()
    msg.r = color[0]
    msg.g = color[1]
    msg.b = color[2]
    msg.a = color[3] if len(color) == 4 else 1.0
    return msg


def to_cloud_msg(points, intensities=None, frame=None, stamp=None):
    """Convert list of unstructured points to a PointCloud2 message.

    Args:
        points: Point coordinates as array of shape (N,3).
        colors: Colors as array of shape (N,3).
        frame
        stamp
    """
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = stamp or rospy.Time.now()

    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = False

    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.point_step = 12
    data = points

    if intensities is not None:
        msg.fields.append(PointField("intensity", 12, PointField.FLOAT32, 1))
        msg.point_step += 4
        data = np.hstack([points, intensities])

    msg.row_step = msg.point_step * points.shape[0]
    msg.data = data.astype(np.float32).tostring()

    return msg


class TransformTree(object):
    def __init__(self):
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def transform(self, pose, source_frame, target_frame):
        assert isinstance(pose, geometry_msgs.msg.Pose), print("Pose format error")
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        # pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = source_frame
        try:
            transformed_pose = self._buffer.transform(pose_stamped, target_frame)
            return from_pose_msg(transformed_pose.pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Get transformed pose failed to do the transform")
            return None

    def lookup(self, target_frame, source_frame, time, timeout=rospy.Duration(0.5)):
        msg = self._buffer.lookup_transform(target_frame, source_frame, time, timeout)
        return from_transform_msg(msg.transform)

    def broadcast(self, transform, target_frame, source_frame):
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = target_frame
        msg.child_frame_id = source_frame
        msg.transform = to_transform_msg(transform)
        self._broadcaster.sendTransform(msg)

    def broadcast_static(self, transform, target_frame, source_frame):
        msg = geometry_msgs.msg.TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = target_frame
        msg.child_frame_id = source_frame
        msg.transform = to_transform_msg(transform)
        self._static_broadcaster.sendTransform(msg)
