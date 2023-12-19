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

    
    ## Start point here corresponds to an ego vehicle position start in a point cloud
    # start_position = {'x': -2755.94365061042, 'y': 722.0595600050154, 'z': -20.004812609192445}
    start_position = {'x': 0.2270930018324647,  'y': 0.2707050015882027, 'z': 0.55}
    # start_position = {'x': 0.0,  'y': 0.0, 'z': 0.65}

    cuboid_points = getCuboidPoints(start_position)

    points = o3d.utility.Vector3dVector(cuboid_points)
    oriented_bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(points)
    point_cloud_crop = pcd.crop(oriented_bounding_box)

    # View original point cloud with the cuboid, all 5 points present
    o3d.visualization.draw_geometries([pcd, oriented_bounding_box])

    # View cropped point cloud with the cuboid, only 3 points present
    o3d.visualization.draw_geometries([point_cloud_crop, oriented_bounding_box])
        
    
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

  

