#!/usr/bin/env python3

import rospy
from custom_msgs.msg import SherdPose

class Pose_class(object):

    def __init__(self):
        self.pose_list = []
        self.i = 0

    def pose_callback(self, msg):
        rospy.loginfo("Received Sherd Pose!")
        try:           
            for i in range(len(msg.SherdList)):
                self.pose_list.append(msg.SherdList)
                self.i = i
        except: 
            print("Error occurred!")

  #method to get the targets outside the pcl_callback method
    def return_pcls(self):
        return(self.i, self.pose_list)
       
    def pose_listener(self):
        rate = rospy.Rate(10)
        pose_listener = rospy.Subscriber("/SherdPoses", SherdPose, self.pose_callback)
        rospy.loginfo("PCL received!")
        rate.sleep()
        pose_listener.unregister()
        # print(pcl_listener)
        # rospy.spin()

if __name__ == '__main__':

    pose_c = Pose_class()
    
    rospy.init_node('pose_listener')

    pose_c.pose_listener()
    print(pose_c.pose_list[1])
    # lista2 = pcl.pcl_list
    # print(lista2[1])