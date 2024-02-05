''' Helper functions to publish poses and pcl's of fragments, after segmentation'''

import vision_utils
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as orh
# import auto_targetter
import numpy as np
# import scipy.spatial
from scipy.spatial.transform import Rotation as R
from custom_msgs.msg import SherdPose, SherdPcl, SherdPclList


def sherd_poses(objects):
    bbox_objects = []
    bbox_poses = []
    sherd_nr = 0
    for i in range(len(objects)):
        bbox_object = objects[i].get_oriented_bounding_box()
        bbox_rot = bbox_object.R
        print(f"rotation of bounding_box is {bbox_rot}")
        R_bbox_rot = R.from_matrix([bbox_rot])
        bbox_quat = R_bbox_rot.as_quat()
        print(f"rotation as quaternion is {bbox_quat}")
        print(f"rotation as quaternion is {bbox_quat.shape}")
        bbox_center = bbox_object.center
        print(f"center of bounding_box is {bbox_center}")
        bbox_object.color = [1.0,0,0]
        bbox_objects.append(bbox_object)

        #Creating the PoseStamped sherd_msg
        sherd_msg = PoseStamped()
        #Filling the message
        sherd_stamp = rospy.Time.now()
        sherd_msg.header.seq = sherd_nr
        sherd_msg.header.frame_id = ("sherd_" + str(sherd_nr))
        sherd_msg.header.stamp.secs = sherd_stamp.secs
        sherd_msg.header.stamp.nsecs = sherd_stamp.nsecs
        sherd_msg.pose.position.x = bbox_center[0]
        sherd_msg.pose.position.y = bbox_center[1]
        sherd_msg.pose.position.z = bbox_center[2]
        sherd_msg.pose.orientation.x = bbox_quat[0,0]
        sherd_msg.pose.orientation.y = bbox_quat[0,1]
        sherd_msg.pose.orientation.z = bbox_quat[0,2]
        sherd_msg.pose.orientation.w = bbox_quat[0,3]
        bbox_poses.append(sherd_msg)
        sherd_nr += 1
    return sherd_msg, bbox_poses

def sherd_pcl(objects):
    ros_pcl = []
    for index, element in enumerate(objects):
        print(index)
        pc2_o3d = orh.o3dpc_to_rospc(element,("pcl_frag_" + str(index)), stamp, seq_nr = index)
        ros_pcl.append(pc2_o3d)
    return ros_pcl