#!/usr/bin/env python3

import rospy
import tf

# from custom_msgs.msg import SherdPclList
from gpd_ros.msg import GraspConfigList

# import open3d as o3d
# from open3d_ros_helper import open3d_ros_helper as orh

class gpd_class(object):

    def __init__(self):
        self.grasp = GraspConfigList()
        self.gpd_listen = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfigList, self.pcl_callback)

    def pcl_callback(self, msg):
        rospy.loginfo("Grasp Pose detected!")
        try:           
            self.grasp = msg
        except: 
            print("Error occurred!")

    #method to get the PCLs outside the callback method
    def return_grasp(self):
        self.gpd_listen
        
        return(self.grasp)

if __name__ == '__main__':

    # pcl = Pcl_class()
    gpd = gpd_class()
    
    rospy.init_node('gpd_listener')
    
    rate = rospy.Rate(0.1)

    while not rospy.is_shutdown():
        gpd.return_grasp()
        print(gpd.grasp)
        rate.sleep()
    