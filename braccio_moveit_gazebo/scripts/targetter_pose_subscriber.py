#!/usr/bin/env python3

import rospy
import numpy as np

from custom_msgs.msg import matrix, target
import auto_targetter
import pose_goal_targetter

import vision_utils


if __name__ == '__main__':
    
    rospy.init_node('targetter_pose_subscriber', anonymous=True)


    targetter = auto_targetter.BraccioObjectTargetInterface(rospy)
    pose_targetter = pose_goal_targetter.BraccioPoseGoal(rospy)

    targetter.load_calibrate()

    rate = rospy.Rate(100)

    sherds = matrix()
    sherds = targetter.centers

    try:
        for i in range(len(sherds)):

            chosen = sherds[i].center
            chosen = np.array(chosen)
            chosen = chosen/1000
            print(type(chosen[0]))
            print(chosen.shape)

            add_zero = np.array([0.000])

            new_chos = np.concatenate((chosen, add_zero))
            chosen_name = sherds[i].sherd
            print(chosen_name)

            chosen_bowl = sherds[i].home
            print(chosen_bowl)

            chosen_dimension = sherds[i].dimension
            
            pose_targetter.create_tf(new_chos)            

            targetter.go_to_target('top', chosen_name)

            # targetter.go_start_position()

            success = pose_targetter.go_to_pos(pose_targetter.arm_target_pose)

            if success:  
                targetter.transform_home(chosen_bowl, chosen_dimension)

        
    except KeyboardInterrupt:
    # User interrupt the program with ctrl+c
        targetter.go_start_position()
        print("Got Ctrl-c: shutting down processes")
        exit()
