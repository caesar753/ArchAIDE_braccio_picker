#!/usr/bin/env python3

import rospy

# from custom_msgs.msg import SherdPclList
from gpd_ros.msg import GraspConfig

# import open3d as o3d
# from open3d_ros_helper import open3d_ros_helper as orh

class gpd_class(object):

    def __init__(self):
        self.grasp = GraspConfig()
        self.gpd_listen = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfig, self.pcl_callback)

    def pcl_callback(self, msg):
        rospy.loginfo("Grasp Pose detected!")
        try:           
            self.grasp = msg
        except: 
            print("Error occurred!")

    #method to get the PCLs outside the callback method
    def return_grasp(self):
        
                
        self.gpd_listen
        # rate.sleep()
        # self.gpd_listen.unregister()
        print(self.grasp)
        
        return(self.grasp)

if __name__ == '__main__':

    # pcl = Pcl_class()
    gpd = gpd_class()
    
    rospy.init_node('gpd_listener')
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        gpd.return_grasp()
        rate.sleep()
    # print(gpd.grasp)

    # pcl.return_pcls()
    # print(len(pcl.pcl_list.list))
    # for i in (range(len(pcl.pcl_list.list))):
    #     pointcl = pcl.pcl_list.list[i]
    #     rospy.loginfo("PCL name: " + str(pointcl.header.frame_id))

    #     open_3d_pcl = orh.rospc_to_o3dpc(pointcl)
    #     o3d.visualization.draw_geometries([open_3d_pcl])
