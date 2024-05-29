#!/usr/bin/env python3

import rospy

from custom_msgs.msg import SherdPclList

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh

class Pcl_class(object):

    def __init__(self):
        self.pcl_list = SherdPclList()
        self.pcl_listen = rospy.Subscriber("/SherdPcls", SherdPclList, self.pcl_callback)

    def pcl_callback(self, msg):
        rospy.loginfo("Received PCL!")
        try:           
            self.pcl_list = msg
        except: 
            print("Error occurred!")

    #method to get the PCLs outside the callback method
    def return_pcls(self):
        
        rate = rospy.Rate(1)        
        self.pcl_listen
        rate.sleep()
        self.pcl_listen.unregister()
        
        return(self.pcl_list)

if __name__ == '__main__':

    pcl = Pcl_class()
    
    rospy.init_node('pcl_listener')

    pcl.return_pcls()
    print(len(pcl.pcl_list.list))
    for i in (range(len(pcl.pcl_list.list))):
        pointcl = pcl.pcl_list.list[i]
        rospy.loginfo("PCL name: " + str(pointcl.header.frame_id))

        open_3d_pcl = orh.rospc_to_o3dpc(pointcl)
        o3d.visualization.draw_geometries([open_3d_pcl])
