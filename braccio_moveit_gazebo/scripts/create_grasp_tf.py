#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion
# from sensor_msgs.msg import JointState
import math
from enum import Enum

from typing import Union, List
import numpy as np
import open3d as o3d
import pytransform3d.transformations as pytr

from tf.transformations import quaternion_from_euler, quaternion_multiply

import vision_utils, sherd3d

if __name__ == '__main__':

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

    initial_pose = np.concatenate((sherd3d.ros_pcl[0].get_center(), hand_tf))
    initial_pose = vision_utils.get_pose_from_arr(initial_pose)

    ### Transform the pose from the camera frame to the base frame (world)
    hand_pose_world = vision_utils.transform_pose_vislab(initial_pose, "camera_depth_optical_frame", "world")
    hand_pose_world_np = vision_utils.get_arr_from_pose(hand_pose_world)
    hand_pose_world_np[0] += 0.04
    hand_pose_world_np[1] += 0.03
    hand_pose_world_np[2] = 1.15 + 0.15
    hand_pose_world_np[3:] = hand_tf
    vision_utils.publish_tf_np(hand_pose_world_np, child_frame='hand_grasp_pose')

    hand_pose_world_np[3:] = np.roll(hand_pose_world_np[3:], 1)
    T0 = pytr.transform_from_pq(hand_pose_world_np)
    T1 = pytr.concat(hand_arm_transform, T0)

    arm_target_pose_np = vision_utils.get_pose_from_transform(T1)

    # arm_target_pose_np = get_arr_from_pose(arm_target_pose)
    vision_utils.publish_tf_np(arm_target_pose_np, child_frame='arm_grasp_pose')
    arm_target_pose = vision_utils.get_pose_stamped_from_arr(arm_target_pose_np)