#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion
# from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import math
from enum import Enum

from scipy.spatial.transform import Rotation


from typing import Union, List
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr

from tf.transformations import quaternion_from_euler, quaternion_multiply

from open3d_ros_helper import open3d_ros_helper as orh

import vision_utils, sherd3d, pcl_listener
import pose_goal_targetter

from custom_msgs.msg import SherdPose, SherdPcl, SherdPclList

if __name__ == '__main__':

    rospy.init_node('grasp_creator', anonymous=True)

    pcl_list = pcl_listener.Pcl_class()
    # targetter = auto_targetter.BraccioObjectTargetInterface(rospy)
    targetter = pose_goal_targetter.BraccioPoseGoal(rospy)

    debug = False
    pcl_list.pcl_listener()

    msg_list = pcl_list.pcl_list
    # print(type(msg_list))

    for i in range(len(msg_list.list)):
        # print(i)
        pcl_ros = PointCloud2()
        pcl_ros = msg_list.list[i]
        # print(type(pcl_ros))
        pcl_o3d = orh.rospc_to_o3dpc(pcl_ros)
        if debug:
            o3d.visualization.draw_geometries([pcl_o3d])
    
    
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

        pippo = pcl_o3d.get_center()
        print(pippo)
        print(type(pippo))
        print(pippo.shape)
        print(type(hand_tf))
        print(hand_tf.shape)

        initial_pose = np.concatenate((pcl_o3d.get_center(), hand_tf))
        initial_pose = vision_utils.get_pose_from_arr(initial_pose)

        ### Transform the pose from the camera frame to the base frame (world)
        hand_pose_world = vision_utils.transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
        hand_pose_world_np = vision_utils.get_arr_from_pose(hand_pose_world)

        hand_pose_world_np = vision_utils.get_arr_from_pose(initial_pose)
        
        # hand_pose_world_np[0] += 0.15
        # hand_pose_world_np[1] -= 0.05
        # # hand_pose_world_np[2] = 1.15 + 0.15
        hand_pose_world_np[2] += 0.025
        # hand_pose_world_np[3:] = hand_tf

        hand_pose_world_quaternion = hand_pose_world_np[3:]
        print(f"hand_pose_world_quaternion is {hand_pose_world_quaternion}")

        rotation_quaternion = quaternion_from_euler(0, 0, 1.17)

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
        
        targetter.go_to_pos(arm_target_pose)