#!/usr/bin/env python3

import rospy
from custom_msgs.msg import matrix
import auto_targetter

if __name__ == '__main__':
    
    rospy.init_node('targetter_subscriber', anonymous=True)

    sherd_choose = input("which sherd?")

    targetter = auto_targetter.BraccioObjectTargetInterface(rospy)

    targetter.load_calibrate()

    rate = rospy.Rate(100)

    # sherd_choose = input("which sherd?")

    sherd_choose = "sherd_" + sherd_choose + "::link"

    targetter.go_to_target('top', 'go_to_home_0', sherd_choose)