#!/usr/bin/env python3

import rospy
from custom_msgs.msg import matrix, target
import auto_targetter

if __name__ == '__main__':
    
    rospy.init_node('targetter_subscriber', anonymous=True)


    targetter = auto_targetter.BraccioObjectTargetInterface(rospy)

    targetter.load_calibrate()

    rate = rospy.Rate(100)


    try:
        for j in range(len(targetter.sherds)):
            targetter.go_to_target('top',targetter.sherds[j].home, targetter.sherds[j].sherd)
        
    except KeyboardInterrupt:
    # User interrupt the program with ctrl+c
        print("Got Ctrl-c: shutting down processes")
        exit()
