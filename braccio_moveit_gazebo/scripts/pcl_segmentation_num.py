#!/usr/bin/env python3

import vision_utils
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
# import auto_targetter
import numpy as np
# import scipy.spatial
from scipy.spatial.transform import Rotation as R
from custom_msgs.msg import SherdPose, SherdPcl, SherdPclList


def sherd_poses(objects):
    bbox_objects = []
    bbox_poses = []
    sherd_nr = 0
    for i in range(len(objects)):
        bbox_object = objects[i].get_oriented_bounding_box()
        bbox_rot = bbox_object.R
        print(f"rotation of bounding_box is {bbox_rot}")
        R_bbox_rot = R.from_matrix([bbox_rot])
        bbox_quat = R_bbox_rot.as_quat()
        print(f"rotation as quaternion is {bbox_quat}")
        print(f"rotation as quaternion is {bbox_quat.shape}")
        bbox_center = bbox_object.center
        print(f"center of bounding_box is {bbox_center}")
        bbox_object.color = [1.0,0,0]
        bbox_objects.append(bbox_object)

        #Creating the PoseStamped sherd_msg
        sherd_msg = PoseStamped()
        #Filling the message
        sherd_stamp = rospy.Time.now()
        sherd_msg.header.seq = sherd_nr
        sherd_msg.header.frame_id = ("sherd_" + str(sherd_nr))
        sherd_msg.header.stamp.secs = sherd_stamp.secs
        sherd_msg.header.stamp.nsecs = sherd_stamp.nsecs
        sherd_msg.pose.position.x = bbox_center[0]
        sherd_msg.pose.position.y = bbox_center[1]
        sherd_msg.pose.position.z = bbox_center[2]
        sherd_msg.pose.orientation.x = bbox_quat[0,0]
        sherd_msg.pose.orientation.y = bbox_quat[0,1]
        sherd_msg.pose.orientation.z = bbox_quat[0,2]
        sherd_msg.pose.orientation.w = bbox_quat[0,3]
        bbox_poses.append(sherd_msg)
        sherd_nr += 1
    return sherd_msg, bbox_poses


if __name__ == '__main__':

    # targetter = auto_targetter.BraccioObjectTargetInterface()

    rospy.init_node('pcl_vis', anonymous=True)
    point_cloud = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)

    print("starting pointcloud segmentation")

    pcd = vision_utils.get_point_cloud_from_ros()

    debug = False

    num_frescos, pcd, table_cloud, object_cloud, objects_pcl = vision_utils.get_number_of_sherds(pcd, debug)#, use_pyrealsense)
    print (f'Number of frescos detected: {num_frescos}')

    print(object_cloud)

    # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.003)
    # object_cloud.paint_uniform_color([1, 0, 0])
    # table_cloud.paint_uniform_color([0, 0, 1])
    
    sherd_msg, bbox_poses = sherd_poses(objects_pcl)

    mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0.45])
    
    if debug:
        o3d.visualization.draw_geometries([mesh_coord_frame, table_cloud, object_cloud, bbox_object])
        o3d.visualization.draw_geometries([mesh_coord_frame, object_cloud])
        # o3d.visualization.draw_geometries([mesh_coord_frame, object_cloud], bbox_object])
        o3d.visualization.draw_geometries([mesh_coord_frame, table_cloud, objects_pcl[0], bbox_objects[0], objects_pcl[1], bbox_objects[1]])
    
    # targetter.get_box_position("sherd_2::link")
        
    # pointcloud_msg = PointCloud2()
    # pointcloud_msg.header = Header()
    # pointcloud_msg.header.frame_id = "frame"

    # pointcloud_msg.fields = object_cloud.fields
    # object_cloud.header.f

    stamp = rospy.Time.now()
    print(stamp.secs)
    print(stamp.nsecs)
    ros_pcl = []
    for index, element in enumerate(objects_pcl):
        print(index)
        pc2_o3d = orh.o3dpc_to_rospc(element,("pcl_frag_" + str(index)), stamp, seq_nr = index)
        # print(pc2_o3d)
        ros_pcl.append(pc2_o3d)


    rate = rospy.Rate(10)
     # creating the pointcloud_pub publisher, topic is /pcl_obj, message type is PointCloud2
    # pointcloud_pub = rospy.Publisher('/pcl_obj', PointCloud2, queue_size=10)

    #Creating a custom SherdList msg
    sherd_list = SherdPose()
    #filling
    sherd_list.SherdList = bbox_poses
    #creating the publisher
    SherdPose_pub = rospy.Publisher('/SherdPoses', SherdPose, queue_size=10)
    

    for i in range(len(ros_pcl)):
        print(f"seq of ros_pcl is {ros_pcl[i].header.seq}")

    #Creating a custom SherdPclList msg which contains the PointCloud2 messages from ros_pcl
    sherds_list_msg = SherdPclList()
    sherds_list_msg.list = ros_pcl
    SherdPcl_pub = rospy.Publisher('/SherdList', SherdPclList, queue_size = 10)


    try:
        # Publishing the PointCloud2 message 
        while not rospy.is_shutdown():
            # pointcloud_pub.publish(pc2_o3d)
            
            # SherdPose_pub.publish(sherd_list)
            SherdPcl_pub.publish(sherds_list_msg)
            
            # rospy.loginfo("Publishing object PointCloud2 message on /pcl_obj")
            # rospy.spin()
            rate.sleep()
    
    except KeyboardInterrupt:
    # User interrupt the program with ctrl+c
        print("Got Ctrl-c: closing subscriber and other processes")
        exit()