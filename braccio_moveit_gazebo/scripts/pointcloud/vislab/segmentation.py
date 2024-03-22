#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion
# from sensor_msgs.msg import JointState
# import math
# from enum import Enum


from typing import Union, List
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr

from tf.transformations import quaternion_from_euler, quaternion_multiply

from vision_utils import get_transform, get_hand_tf, publish_tf_np
from vision_utils import get_pose_from_arr, get_pose_stamped_from_arr
from vision_utils import get_arr_from_pose, get_point_cloud_from_ros, get_point_cloud_from_real_rs
from vision_utils import segment_table, transform_pose_vislab, get_pose_from_transform


if __name__ == '__main__':
    node_name = "pcl_seg"
    rospy.init_node(node_name)

    print('Starting Point Cloud Processing')
    use_pyrealsense = False
    debug = True
    if use_pyrealsense:
        pcd = get_point_cloud_from_real_rs(debug)
    else:
        pcd = get_point_cloud_from_ros(debug)


    # == Transform pointcloud to table frame
    tf_camera_to_world = get_transform(parent_frame="world", child_frame="camera_depth_optical_frame")
    tran = np.array([tf_camera_to_world.transform.translation.x, tf_camera_to_world.transform.translation.y, tf_camera_to_world.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_camera_to_world.transform.rotation.w,
                                                                    tf_camera_to_world.transform.rotation.x,
                                                                    tf_camera_to_world.transform.rotation.y,
                                                                    tf_camera_to_world.transform.rotation.z]))

    pcd.rotate(rot, center=(0, 0, 0)).translate(tran)
    if debug:
        o3d.visualization.draw_geometries([pcd], window_name="PCD Transformed table")


    # == Remove points above & below a certain height
    points = np.asarray(pcd.points)
    # pcd = pcd.select_by_index(np.where(points[:, 2] < 0.08)[0])
    # points = np.asarray(pcd.points)

    object_cloud = pcd.select_by_index(np.where((points[:, 2] < 0.10) & (points[:, 2] > 0.001))[0])
    table_cloud = pcd.select_by_index( np.where(((points[:, 2] < 0.001) & (points[:, 2] > -0.05)))[0])
    # pcd = pcd.select_by_index(np.where(points[:, 2] > -0.04)[0])

    if debug:
        object_cloud.paint_uniform_color([1, 1, 0])
        table_cloud.paint_uniform_color([0, 0, 1])
        o3d.visualization.draw_geometries([table_cloud, object_cloud])
        # o3d.visualization.draw_geometries([pcd], window_name="PCD Filtered")

    # == Transform back to camera frame
    tf_world_to_camera = get_transform(parent_frame="camera_depth_optical_frame", child_frame="world")
    tran = np.array([tf_world_to_camera.transform.translation.x, tf_world_to_camera.transform.translation.y, tf_world_to_camera.transform.translation.z])
    rot = o3d.geometry.get_rotation_matrix_from_quaternion(np.array([tf_world_to_camera.transform.rotation.w,
                                                                    tf_world_to_camera.transform.rotation.x,
                                                                    tf_world_to_camera.transform.rotation.y,
                                                                    tf_world_to_camera.transform.rotation.z]))
    # pcd.rotate(rot, center=(0, 0, 0)).translate(tran)
    object_cloud.rotate(rot, center=(0, 0, 0)).translate(tran)
    table_cloud.rotate(rot, center=(0, 0, 0)).translate(tran)

    print ('Table Segmentation')
    # table_cloud, object_cloud = segment_table(pcd)

    voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)
    object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.03)

    if debug:
        object_cloud.paint_uniform_color([0, 1, 0])
        # table_cloud.paint_uniform_color([1, 0, 0])
        # o3d.visualization.draw_geometries([table_cloud, object_cloud])
        o3d.visualization.draw_geometries([object_cloud])

    # initial_pose = np.concatenate((object_cloud.get_center(), hand_tf))
    # initial_pose = get_pose_from_arr(initial_pose)