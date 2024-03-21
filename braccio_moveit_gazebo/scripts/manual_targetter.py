#!/usr/bin/env python3

import rospy
import numpy as np

import pytransform3d.transformations as pytr
from tf.transformations import quaternion_from_euler, quaternion_multiply


from custom_msgs.msg import matrix, target
import auto_targetter
import pose_goal_targetter

import vision_utils

# def callback_matrix(msg):
#         # rospy.loginfo(msg)
#     for i in range(len(msg.targets)):
#         # print(i)
#         # print(msg.targets[i])
#         targets_list.append(msg.targets[i])
#         i = i

if __name__ == '__main__':
    
    rospy.init_node('targetter_subscriber', anonymous=True)

    sherd_ch = input("which sherd?")

    targetter = auto_targetter.BraccioObjectTargetInterface(rospy)
    pose_targetter = pose_goal_targetter.BraccioPoseGoal(rospy)

    targetter.load_calibrate()

    rate = rospy.Rate(100)

    sherds = matrix()
    sherds = targetter.centers
    rospy.loginfo(sherds[int(sherd_ch)].center)

    chosen = sherds[int(sherd_ch)].center
    chosen = np.array(chosen)
    chosen = chosen/1000
    print(type(chosen[0]))
    print(chosen.shape)

    add_zero = np.array([0.005])

    new_chos = np.concatenate((chosen, add_zero))
    print(new_chos)
    print(new_chos.shape)

    # new_chos = np.reshape(new_chos, (3,1))

    # print(new_chos)
    # print(new_chos.shape)
    chosen_name = sherds[int(sherd_ch)].sherd
    print(chosen_name)

    chosen_bowl = sherds[int(sherd_ch)].home
    print(chosen_bowl)
    # home_ch = getattr(chosen_bowl)
    # targetter.home_ch()

    chosen_dimension = sherds[int(sherd_ch)].dimension
    
    pose_targetter.create_tf(new_chos, quat_z = 1.50)
    # pose_targetter.create_tf(new_chos, quat_x = -0.39, quat_z = 2.32)

    targetter.go_to_target('top', chosen_name)

    # targetter.go_start_position()

    success = pose_targetter.go_to_pos(pose_targetter.arm_target_pose)

    if not success:
        pose_targetter.create_tf(new_chos, quat_x = -0.39, quat_z = 2.32)
    #     pose_targetter.create_tf(new_chos, quat_z = 2.32)
        success = pose_targetter.go_to_pos(pose_targetter.arm_target_pose)

    if success:
        targetter.transform_home(chosen_bowl, chosen_dimension)