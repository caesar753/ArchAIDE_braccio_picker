#!/usr/bin/env python3

# from __future__ import print_function

# import sys
import rospy
# import time

from custom_msgs.msg import SherdPclList
# from sensor_msgs.msg import PointCloud2

class Pcl_class(object):

    def __init__(self):
        self.pcl_list = SherdPclList()
        self.i = 0

    def pcl_callback(self, msg):
        # rospy.loginfo("Received PCL!")
        try:           
            for i in range(len(msg.list)):
                # print(i)
                self.pcl_list = msg
                self.i = i
        except: 
            print("Error occurred!")

  #method to get the targets outside the pcl_callback method
    def return_pcls(self):
        return(self.i, self.pcl_list)
       
    def pcl_listener(self):
        rate = rospy.Rate(1)
        pcl_listener = rospy.Subscriber("/SherdPcls", SherdPclList, self.pcl_callback)
        rospy.loginfo("PCL received!")
        rate.sleep()
        pcl_listener.unregister()


# if __name__ == '__main__':

#     pcl = Pcl_class()
    
#     rospy.init_node('pcl_listener')

#     pcl.pcl_listener()
#     print(pcl.pcl_list[2])
#     # lista2 = pcl.pcl_list
#     # print(lista2[1])