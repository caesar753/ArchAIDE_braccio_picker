#!/usr/bin/env python3

import rospy
import tf2_ros

from gpd_ros.msg import GraspConfig
import geometry_msgs.msg
import tf.transformations
import pytransform3d.rotations as pyrot
from mlxtend.math import vectorspace_orthonormalization


import numpy as np

from gpd_listener_class import gpd_class

def publish_tf(grasp_config):
    # Initialize ROS node
    rospy.init_node('tf_publisher')

    # Initialize TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create a transform message
    transform_msg = tf2_ros.TransformStamped()
    transform_msg.header.stamp = rospy.Time.now()
    transform_msg.header.frame_id = 'base_link'
    transform_msg.child_frame_id = 'gripper'

    # Set the transform values from the GraspConfig message
    transform_msg.transform.translation.x = grasp_config.approach
    transform_msg.transform.translation.y = grasp_config.binoarmal
    transform_msg.transform.translation.z = grasp_config.axis

    # Publish the transform
    tf_broadcaster.sendTransform(transform_msg)

def isRotationMatrix(M):
    tag = False
    I = np.identity(M.shape[0])
    if np.all((np.matmul(M, M.T)) - I < 1e-10): #and (np.linalg.det(M)==1): tag = True
        tag = True
    return tag    

if __name__ == '__main__':
    rospy.init_node('gpd_listener')
    
    # Create a GPD class
    gpd = gpd_class()

    # Create a TF broadcaster
    br = tf2_ros.StaticTransformBroadcaster()

    # Create a GraspConfig message
    gpd_listened = GraspConfig()

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        gpd_listened = gpd.return_grasp()
        print(f"listened gpd is {gpd_listened}")
        
        # Create a Transform message
        t = geometry_msgs.msg.TransformStamped()
        
        # Fill in the Transform message
        t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "base_link"  # Replace with your source frame
        # t.child_frame_id = "grasp_frame"  # Replace with your target frame
        t.header.frame_id = "base_link"  # Replace with your source frame
        t.child_frame_id = "prova"
        t.transform.translation = gpd_listened.position  # Assuming gpd_listened.position is a geometry_msgs/Vector3
        t.transform.translation.z = 0.0
       
        # Convert the approach, axis, and binormal to a quaternion
        approach_as_floats = [gpd_listened.approach.x, gpd_listened.approach.y, gpd_listened.approach.z]
        binormal_as_floats = [gpd_listened.binormal.x, gpd_listened.binormal.y, gpd_listened.binormal.z]
        axis_as_floats = [gpd_listened.axis.x, gpd_listened.axis.y, gpd_listened.axis.z]

        if approach_as_floats[0] != 0:
            

            # Normalize the vectors
            approach_as_floats = np.array(approach_as_floats)
            approach_as_floats /= np.linalg.norm(approach_as_floats)

            binormal_as_floats = np.array(binormal_as_floats)
            binormal_as_floats /= np.linalg.norm(binormal_as_floats)

            axis_as_floats = np.array(axis_as_floats)
            axis_as_floats /= np.linalg.norm(axis_as_floats)  
            
            

            print(approach_as_floats)
            print(binormal_as_floats)
            print(axis_as_floats)

            # Print the norm of each vector
            print("Approach norm:", np.linalg.norm(approach_as_floats))
            print("Binormal norm:", np.linalg.norm(binormal_as_floats))
            print("Axis norm:", np.linalg.norm(axis_as_floats))

            rotation_matrix = np.column_stack((approach_as_floats, binormal_as_floats, axis_as_floats)) 
            print(rotation_matrix)

            rotation_matrix = vectorspace_orthonormalization(rotation_matrix)
            print(rotation_matrix)
           
            tag = isRotationMatrix(rotation_matrix)
            print(tag)

            # quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix[:3, :3])
            quaternion = pyrot.quaternion_from_matrix(rotation_matrix[:3, :3])

            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

        else:
            t.transform.rotation.x = 0.0  # Replace with your rotation values
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

        print(t)


        # Publish the TF
        br.sendTransform(t)
        
        
        rate.sleep()

    # Set the values of the GraspConfig message
    # grasp_config.approach = 0.1
    # grasp_config.binoarmal = 0.2
    # grasp_config.axis = 0.3

    # Publish the TF transform
    # publish_tf(gpd_listened)