#!/usr/bin/env python3

import vision_utils
import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import auto_targetter
import numpy as np


CUBOID_EXTENT_METERS = 0.100

METERS_BELOW_START = 0.020
METERS_ABOVE_START = 0.020

def getCuboidPoints(start_position):
  return np.array([
    # Vertices Polygon1
    [start_position['x'] + (CUBOID_EXTENT_METERS / 2), start_position['y'] + (CUBOID_EXTENT_METERS / 2), start_position['z'] + METERS_ABOVE_START], # face-topright
    [start_position['x'] - (CUBOID_EXTENT_METERS / 2), start_position['y'] + (CUBOID_EXTENT_METERS / 2), start_position['z'] + METERS_ABOVE_START], # face-topleft
    [start_position['x'] - (CUBOID_EXTENT_METERS / 2), start_position['y'] - (CUBOID_EXTENT_METERS / 2), start_position['z'] + METERS_ABOVE_START], # rear-topleft
    [start_position['x'] + (CUBOID_EXTENT_METERS / 2), start_position['y'] - (CUBOID_EXTENT_METERS / 2), start_position['z'] + METERS_ABOVE_START], # rear-topright

    # Vertices Polygon 2
    [start_position['x'] + (CUBOID_EXTENT_METERS / 2), start_position['y'] + (CUBOID_EXTENT_METERS / 2), start_position['z'] - METERS_BELOW_START],
    [start_position['x'] - (CUBOID_EXTENT_METERS / 2), start_position['y'] + (CUBOID_EXTENT_METERS / 2), start_position['z'] - METERS_BELOW_START],
    [start_position['x'] - (CUBOID_EXTENT_METERS / 2), start_position['y'] - (CUBOID_EXTENT_METERS / 2), start_position['z'] - METERS_BELOW_START],
    [start_position['x'] + (CUBOID_EXTENT_METERS / 2), start_position['y'] - (CUBOID_EXTENT_METERS / 2), start_position['z'] - METERS_BELOW_START],
  ]).astype("float64") 



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

    ## Start point here corresponds to an ego vehicle position start in a point cloud
    # start_position = {'x': -2755.94365061042, 'y': 722.0595600050154, 'z': -20.004812609192445}
    start_position = {'x': 0.2270930018324647,  'y': 0.2707050015882027, 'z': 0.55}
    # start_position = {'x': 0.0,  'y': 0.0, 'z': 0.65}

    ##CREATE BOUNDING BOX FROM POINTS
    # cuboid_points = getCuboidPoints(start_position)
    # points = o3d.utility.Vector3dVector(cuboid_points)
    # oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points)

    # R_o3 = o3d.geometry.get_rotation_matrix_from_xyz((-np.pi/4, 0, 0))
    # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((3*(np.pi/4),0,0)) #FOR D345 WITH 0.78 ON PITCH!
    # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((-3*(np.pi/8),0,0)) #FOR D345 WITH 0.395 ON PITCH!

    # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((5*(np.pi/6),0,0)) #FOR D345 WITH 1.15 PITCH
    # R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((5*(np.pi/6),0.84,0))
    R_o3 = o3d.geometry.get_rotation_matrix_from_axis_angle((2.60,0.84,0))
    print(R_o3)
    
    ##CREATE BOUNDING BOX FROM CENTER, EXTENSION AND ROTATION
    # oriented_bounding_box = o3d.geometry.OrientedBoundingBox(center=[0.0, 0.0, 0.65], extent=[0.10, 0.10, 0.02],
    #                                      R=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    oriented_bounding_box = o3d.geometry.OrientedBoundingBox(center=[0, 0, 0.65], extent=[0.10, 0.10, 0.02],
                                        #  R=[[np.cos(np.pi/4), -np.sin(np.pi/4), 0], [np.sin(np.pi/4),np.cos(np.pi/4), 0], [0, 0, 1]])
                                          # R=[[1, 0, 0], [-np.cos(np.pi/4), np.sin(np.pi/4), 0], [-np.sin(np.pi/4), -np.cos(np.pi/4), 0]])         
                                          # R=[[np.cos(np.pi/4), 0, np.sin(np.pi/4)], [0, 1, 0], [-np.sin(np.pi/2), 0, np.cos(np.pi/2)]])
                                        R = R_o3)
    
    ##CROP THE VOLUME IN THE BOUNDING BOX
    point_cloud_crop = pcd.crop(oriented_bounding_box)


    # Create an example pointcloud
    # pcl:o3d.geometry.PointCloud = pcd
    # Compute an OBB which covers pcl tightly. Robust = True helps in degenerative cases, it adds a little noise to prevent any computational error.
    oriented_bounding_box_pcl = pcd.get_oriented_bounding_box()

    print(f'Box points are\
          {np.array(oriented_bounding_box_pcl.get_box_points())}')

    print(f'Center is\
          {oriented_bounding_box_pcl.get_center()}')

    print(f'Rotation matrix from axis_angle is\
          {oriented_bounding_box_pcl.get_rotation_matrix_from_xyz([0.0,0.0,0.0])}')

    # oriented_bounding_box_pcl = oriented_bounding_box_pcl.scale(scale = 0.1, center = (0.04,-0.12,0.70))
    
    
    print(f'Min_bound is\
          {oriented_bounding_box_pcl.get_min_bound()}')
    
    print(f'Max_bound is\
          {oriented_bounding_box_pcl.get_max_bound()}')




    # oriented_bounding_box_pcl_crop = oriented_bounding_box_pcl()
    # Set color since OBB will have white color which is difficult to see with white background of visualizer.
    oriented_bounding_box_pcl.color = [1.0, 0, 0]

    # View original point cloud with the cuboid, all 5 points present
    o3d.visualization.draw_geometries([mesh_coord_frame, pcd, oriented_bounding_box])

    # View cropped point cloud with the cuboid, only 3 points present
    o3d.visualization.draw_geometries([mesh_coord_frame, point_cloud_crop, oriented_bounding_box])

    # View original point cloud with oriented_bounding_box_pcd
    o3d.visualization.draw_geometries([mesh_coord_frame, pcd, oriented_bounding_box_pcl])
        
    
    # print ('Table Segmentation')
    # table_cloud, object_cloud = vision_utils.segment_table(pcd)

    # voxel_pc = object_cloud.voxel_down_sample(voxel_size=0.001)

    # object_cloud, ind = voxel_pc.remove_radius_outlier(nb_points=40, radius=0.003)
    # object_cloud.paint_uniform_color([0, 1, 0])
    # table_cloud.paint_uniform_color([1, 0, 0])

    
    # if debug:
    #     o3d.visualization.draw_geometries([table_cloud, object_cloud])
    # targetter.get_box_position("sherd_2::link")





# def main():
#   ## Point Cloud
#   points = np.array([
#     ## These points lie inside the cuboid
#     [-2770.94365061042, 722.0595600050154, -20.004812609192445],
#     [-2755.94365061042, 710.0595600050154, -20.004812609192445],
#     [-2755.94365061042, 710.0595600050154, -15.004812609192445],

#     ## These points lie outside the cuboid
#     [-2755.94365061042 + CUBOID_EXTENT_METERS, 710.0595600050154, -15.004812609192445],
#     [-2755.94365061042, 710.0595600050154 + CUBOID_EXTENT_METERS, -15.004812609192445],
#   ]).reshape([-1, 3])

#   point_cloud = o3d.geometry.PointCloud()
#   point_cloud.points = o3d.utility.Vector3dVector(points)

  

