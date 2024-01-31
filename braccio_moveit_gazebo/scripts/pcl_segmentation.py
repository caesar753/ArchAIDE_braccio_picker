#!/usr/bin/env python3

import vision_utils
import rospy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
# import auto_targetter
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

    # points = np.asarray(object_cloud.points)
    
    # object_cloud = object_cloud.select_by_index(np.where((points[:, 2] > 0.02))[0])

    # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.01)

    print(object_cloud)

    # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.003)
    object_cloud.paint_uniform_color([1, 0, 0])
    table_cloud.paint_uniform_color([0, 0, 1])

    bbox_object = object_cloud.get_oriented_bounding_box()
    bbox_object.color = [1.0,0,0]

    
    if debug:
        o3d.visualization.draw_geometries([mesh_coord_frame, table_cloud, object_cloud, bbox_object])
        o3d.visualization.draw_geometries([mesh_coord_frame, object_cloud])
        o3d.visualization.draw_geometries([mesh_coord_frame, object_cloud, bbox_object])
    # targetter.get_box_position("sherd_2::link")
        
    # pointcloud_msg = PointCloud2()
    # pointcloud_msg.header = Header()
    # pointcloud_msg.header.frame_id = "frame"

    # pointcloud_msg.fields = object_cloud.fields
    # object_cloud.header.f

    stamp = rospy.Time.now()
    print(stamp.secs)
    print(stamp.nsecs)
    pc2_o3d = orh.o3dpc_to_rospc(object_cloud, "prova", stamp)
    # print(pc2_o3d)

    rate = rospy.Rate(10)
     # creating the pointcloud_pub publisher, topic is /pcl_obj, message type is PointCloud2
    pointcloud_pub = rospy.Publisher('/pcl_obj', PointCloud2, queue_size=10)

    try:
        # Publishing the PointCloud2 message 
        while not rospy.is_shutdown():
            pointcloud_pub.publish(pc2_o3d)
            # rospy.spin()
            rate.sleep()
    
    except KeyboardInterrupt:
    # User interrupt the program with ctrl+c
        print("Got Ctrl-c: closing subscriber and other processes")
        exit()