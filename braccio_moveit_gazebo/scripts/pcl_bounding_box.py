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
      debug = False
      pcd = vision_utils.get_point_cloud_from_ros(debug)
      print(f'PointCloud height is {point_cloud.height}')
      print(f'PointCloud width is {point_cloud.width}')


      mesh_coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0.45])

      # R_o3 = o3d.geometry.get_rotation_matrix_from_xyz((-np.pi/4, 0, 0))
      # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((3*(np.pi/4),0,0)) #FOR D345 WITH 0.78 ON PITCH!
      # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((-3*(np.pi/8),0,0)) #FOR D345 WITH 0.395 ON PITCH!

      # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((5*(np.pi/6),0,0)) #FOR D345 WITH 1.15 PITCH
      # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((5*(np.pi/6),0.84,0))
      #     R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((2.60,0.84,0))
      #     print(R_o3)


      # Compute an OBB which covers pcl tightly. Robust = True helps in degenerative cases, it adds a little noise to prevent any computational error.
      oriented_bounding_box_pcl = pcd.get_oriented_bounding_box()

      rotation_oriented = oriented_bounding_box_pcl.R
      print(f"The rotation of the oriented bounding box is \
            {rotation_oriented}")

      center_oriented = oriented_bounding_box_pcl.center
      print(f"The center of the oriented bounding box is \
            {center_oriented}")
      
      # Set color since OBB will have white color which is difficult to see with white background of visualizer.
      oriented_bounding_box_pcl.color = [1.0, 0, 0]
      
      # View whole PCL oriented_bounding_box
      o3d.visualization.draw_geometries([mesh_coord_frame, pcd, oriented_bounding_box_pcl])

      #If you want to scale the bounding box, defining a center
      oriented_bounding_box_pcl_scale = oriented_bounding_box_pcl.scale(scale = 0.1, center = (0.04,-0.12,0.70))

      center_scaled = oriented_bounding_box_pcl.center
      print(f"The center of the oriented bounding box is \
            {center_scaled}")
      
      ##CREATE BOUNDING BOX FROM CENTER, EXTENSION AND ROTATION
      # oriented_bounding_box = o3d.geometry.OrientedBoundingBox(center=[0.0, 0.0, 0.65], extent=[0.10, 0.10, 0.02],
      #                                      R=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
      oriented_bounding_box = o3d.geometry.OrientedBoundingBox(center=center_scaled, extent=[0.10, 0.10, 0.02],
                                          #  R=[[np.cos(np.pi/4), -np.sin(np.pi/4), 0], [np.sin(np.pi/4),np.cos(np.pi/4), 0], [0, 0, 1]])
                                          # R=[[1, 0, 0], [-np.cos(np.pi/4), np.sin(np.pi/4), 0], [-np.sin(np.pi/4), -np.cos(np.pi/4), 0]])         
                                          # R=[[np.cos(np.pi/4), 0, np.sin(np.pi/4)], [0, 1, 0], [-np.sin(np.pi/2), 0, np.cos(np.pi/2)]])
                                          R = rotation_oriented)

      ##CROP THE VOLUME IN THE BOUNDING BOX
      point_cloud_crop = pcd.crop(oriented_bounding_box)

      #Vertices of the bounding box
      print(f'Box points are\
            {np.array(oriented_bounding_box_pcl.get_box_points())}')
      #Center of the bounding box
      print(f'Center is\
            {oriented_bounding_box_pcl.get_center()}')
      #MBOH
      print(f'Rotation matrix from axis_angle is\
            {oriented_bounding_box_pcl.get_rotation_matrix_from_xyz([0.0,0.0,0.0])}')

      print(f'Min_bound is\
            {oriented_bounding_box_pcl.get_min_bound()}')

      print(f'Max_bound is\
            {oriented_bounding_box_pcl.get_max_bound()}')

      # View scaled PCL oriented_bounding_box
      o3d.visualization.draw_geometries([mesh_coord_frame, pcd, oriented_bounding_box_pcl_scale])
      
      # View original point cloud with the cuboid, all 5 points present
      o3d.visualization.draw_geometries([mesh_coord_frame, pcd, oriented_bounding_box])

      # View cropped point cloud with the cuboid, only 3 points present
      o3d.visualization.draw_geometries([mesh_coord_frame, point_cloud_crop, oriented_bounding_box])

      # print ('Table Segmentation')
      # table_cloud, object_cloud = vision_utils.segment_table(pcd)

      # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

      # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.003)
      # object_cloud.paint_uniform_color([0, 1, 0])
      # table_cloud.paint_uniform_color([1, 0, 0])


      # if debug:
      #     o3d.visualization.draw_geometries([table_cloud, object_cloud])
      # targetter.get_box_position("sherd_2::link")

  

