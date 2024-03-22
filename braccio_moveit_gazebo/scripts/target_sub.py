#!/usr/bin/env python3

import rospy
import numpy as np

from custom_msgs.msg import matrix, target
import auto_targetter

import vision_utils

def callback_matrix(msg):
    # rospy.loginfo("message received!")
    # pass
    # for i in range(len(msg.targets)):
    #     # rospy.loginfo(msg.targets[i])
    #     pippo = msg.targets[i]
    #     rospy.loginfo(pippo.center)

    pluto = msg.targets
    rospy.loginfo(pluto[5].center)
    # return pippo

if __name__ == '__main__':
    
    rospy.init_node('target_sub', anonymous=True)

    # sherd_ch = input("which sherd?")

    # targetter = auto_targetter.BraccioObjectTargetInterface(rospy)

    # targetter.load_calibrate()

    rate = rospy.Rate(100)

    center_msg = rospy.Subscriber("/targets", matrix, callback_matrix)

    # print(center_msg.targets)

    rate.sleep()

    rospy.spin()