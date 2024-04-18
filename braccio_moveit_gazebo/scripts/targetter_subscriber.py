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
        for j in range(targetter.i+1):
            # targetter.get_link_choose(targetter.targets_list[j].sherd)
            targetter.go_to_target('top', targetter.targets_list[j].home, targetter.targets_list[j].sherd)
            
            targetter.go_start_position()
        
    except KeyboardInterrupt:
    # User interrupt the program with ctrl+c
        targetter.go_start_position()
        print("Got Ctrl-c: shutting down processes")
        exit()
