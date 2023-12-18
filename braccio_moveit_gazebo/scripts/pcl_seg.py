#!/usr/bin/env python3

import vision_utils
import rospy
from sensor_msgs.msg import PointCloud2


if __name__ == '__main__':

    rospy.init_node('pcl_vis', anonymous=True)
    point_cloud = rospy.wait_for_message("/camera/depth/color/points", PointCloud2)

    print("starting pointcloud segmentation")
    debug = True
    pcd = vision_utils.get_point_cloud_from_ros(debug)
    print(f'PointCloud height is {point_cloud.height}')
    print(f'PointCloud width is {point_cloud.width}')