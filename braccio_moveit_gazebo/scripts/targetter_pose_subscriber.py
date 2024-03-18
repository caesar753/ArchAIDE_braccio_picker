#!/usr/bin/env python3

import rospy
import numpy as np

import pytransform3d.transformations as pytr
from tf.transformations import quaternion_from_euler, quaternion_multiply


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

            add_zero = np.array([0.002])

            new_chos = np.concatenate((chosen, add_zero))
            chosen_name = sherds[i].sherd
            print(chosen_name)

            chosen_bowl = sherds[i].home
            print(chosen_bowl)

            tf_hand = vision_utils.get_transform(parent_frame="central_gripper_link",\
                                                    child_frame="arm_grasp_link")
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

            print(hand_tf.shape)

            initial_pose = np.concatenate((new_chos, hand_tf))
            initial_pose = vision_utils.get_pose_from_arr(initial_pose)

            ### Transform the pose from the camera frame to the base frame (world)
            hand_pose_world = vision_utils.transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
            hand_pose_world_np = vision_utils.get_arr_from_pose(hand_pose_world)

            hand_pose_world_np = vision_utils.get_arr_from_pose(initial_pose)
            
            # hand_pose_world_np[0] -= 0.035
            # hand_pose_world_np[1] -= 0.035
            # # hand_pose_world_np[2] = 1.15 + 0.15
            # hand_pose_world_np[2] += 0.025
            hand_pose_world_np[2] -= 0.002
            # hand_pose_world_np[3:] = hand_tf

            hand_pose_world_quaternion = hand_pose_world_np[3:]
            print(f"hand_pose_world_quaternion is {hand_pose_world_quaternion}")

            rotation_quaternion = quaternion_from_euler(0, 0, 2.32)

            hand_pose_world_quaternion = quaternion_multiply(rotation_quaternion, hand_pose_world_quaternion)
            
            print(f"rotated hand_pose_world_quaternion is {hand_pose_world_quaternion}")

            hand_pose_world_np[3:] = hand_pose_world_quaternion
            
            print(f"final hand_pose_world_np is {hand_pose_world_np}")

            vision_utils.publish_tf_np(hand_pose_world_np, child_frame='gripper_grasp_pose')

            hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
            
            print(f"hand_pose_world_np roll is {hand_pose_world_np}")

            T0 = pytr.transform_from_pq(hand_pose_world_np)
            T1 = pytr.concat(hand_arm_transform, T0)

            arm_target_pose_np = vision_utils.get_pose_from_transform(T1)

            # arm_target_pose_np = vision_utils.get_arr_from_pose(arm_target_pose)
            vision_utils.publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
            arm_target_pose_stamped = vision_utils.get_pose_stamped_from_arr(arm_target_pose_np)
            arm_target_pose = vision_utils.get_pose_from_arr(arm_target_pose_np)

            targetter.go_to_target('top',  chosen_name)

            # targetter.go_start_position()

            success = pose_targetter.go_to_pos(arm_target_pose)

            if success:  
                targetter.transform_home(chosen_bowl)

        
    except KeyboardInterrupt:
    # User interrupt the program with ctrl+c
        targetter.go_start_position()
        print("Got Ctrl-c: shutting down processes")
        exit()
