#!/usr/bin/env python3

import auto_targetter
import rospy

if __name__ == "__main__":

    rospy.init_node('targetter_calibration', anonymous=False)

    targetter = auto_targetter.BraccioObjectTargetInterface(rospy)
    targetter.calibrate()

    exit(0)