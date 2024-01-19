#!/usr/bin/env python3

import vision_utils
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import auto_targetter
import numpy as np


if __name__ == '__main__':

    # targetter = auto_targetter.BraccioObjectTargetInterface()


    rospy.init_node('pcl_vis', anonymous=True)
    point_cloud = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)

    print("starting pointcloud segmentation")
    debug = True
    pcd = vision_utils.get_point_cloud_from_ros(debug)


    mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0.45])


    print ('Table Segmentation')
    table_cloud, object_cloud = vision_utils.segment_table(pcd)

    # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.01)

    print(object_cloud)

    # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.003)
    object_cloud.paint_uniform_color([1, 0, 0])
    table_cloud.paint_uniform_color([0, 0, 1])

    
    if debug:
        o3d.visualization.draw_geometries([table_cloud, object_cloud])
        o3d.visualization.draw_geometries([object_cloud])
    # targetter.get_box_position("sherd_2::link")
