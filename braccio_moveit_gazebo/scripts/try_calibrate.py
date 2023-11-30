#!/usr/bin/env python3

import rospy
from custom_msgs.msg import matrix
import auto_targetter

if __name__ == '__main__':

    targetter = auto_targetter.BraccioObjectTargetInterface()
    
    # print(targetter.get_link_position('kuka::mount'))

    targetter.calibrate()