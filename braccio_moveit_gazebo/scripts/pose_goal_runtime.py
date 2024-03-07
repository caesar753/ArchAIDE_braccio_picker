#!/usr/bin/env python3

import rospy
import numpy as np
import pytransform3d.transformations as pytr

# from custom_msgs.msg import matrix

import pose_goal_targetter
import vision_utils

if __name__ == '__main__':
    
    rospy.init_node('pose_goal_targetter', anonymous=True)


    pose_targetter = pose_goal_targetter.BraccioPoseGoal(rospy)

    pose_targetter.load_calibrate()

    rate = rospy.Rate(100)
    
    tf_hand = vision_utils.get_transform(parent_frame="gripper_grasp_link",\
                                            child_frame="link_5")
    # print (tf)

    hand_arm_transform = pytr.transform_from_pq([tf_hand.transform.translation.x,
                                                tf_hand.transform.translation.y,
                                                tf_hand.transform.translation.z,
                                                tf_hand.transform.rotation.w,
                                                tf_hand.transform.rotation.x,
                                                tf_hand.transform.rotation.y,
                                                tf_hand.transform.rotation.z
                                                ])
    
    hand_tf = vision_utils.get_hand_tf()


    initial_pose = np.concatenate(([0.10, 0.10, 0.03], hand_tf))
    initial_pose = vision_utils.get_pose_from_arr(initial_pose)

    ### Transform the pose from the camera frame to the base frame (world)
    hand_pose_world = vision_utils.transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
    hand_pose_world_np = vision_utils.get_arr_from_pose(hand_pose_world)

    hand_pose_world_np = vision_utils.get_arr_from_pose(initial_pose)
    
    # hand_pose_world_np[0] += 0.04
    # hand_pose_world_np[1] += 0.03
    # # hand_pose_world_np[2] = 1.15 + 0.15
    hand_pose_world_np[2] += 0.01
    hand_pose_world_np[3:] = hand_tf
    vision_utils.publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

    hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
    T0 = pytr.transform_from_pq(hand_pose_world_np)
    T1 = pytr.concat(hand_arm_transform, T0)

    arm_target_pose_np = vision_utils.get_pose_from_transform(T1)

    # arm_target_pose_np = vision_utils.get_arr_from_pose(arm_target_pose)
    vision_utils.publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose_stamped = vision_utils.get_pose_stamped_from_arr(arm_target_pose_np)
    arm_target_pose = vision_utils.get_pose_from_arr(arm_target_pose_np)
    print(type(arm_target_pose))

    pose_targetter.get_planning_infos()
    # pose_targetter.random()

    # pose_targetter.planning_pose_random()
    
    pose_targetter.planning_pose_example()
    
    # pos = input("choose one default position")
    # pose_targetter.set_standard_position(pos)

    # pose_targetter.set_joint()

    # pose_targetter.go_to_pos(arm_target_pose)