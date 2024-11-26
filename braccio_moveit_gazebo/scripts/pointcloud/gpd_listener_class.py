#!/usr/bin/env python3

import rospy
import tf

# from custom_msgs.msg import SherdPclList
from gpd_ros.msg import GraspConfig

# import open3d as o3d
# from open3d_ros_helper import open3d_ros_helper as orh

class gpd_class(object):

    def __init__(self):
        self.grasp = GraspConfig()
        self.gpd_listen = rospy.Subscriber("/detect_grasps/clustered_grasps", GraspConfig, self.pcl_callback)

    def pcl_callback(self, msg):
        rospy.loginfo("Grasp Pose detected!")
        try:           
            self.grasp = msg
        except: 
            print("Error occurred!")

    #method to get the PCLs outside the callback method
    def return_grasp(self):
        self.gpd_listen
        
        return(self.grasp)

# if __name__ == '__main__':

#     # pcl = Pcl_class()
#     gpd = gpd_class()
    
#     rospy.init_node('gpd_listener')
    
#     rate = rospy.Rate(0.1)

#     while not rospy.is_shutdown():
#         gpd.return_grasp()
#         print(gpd.grasp)
#         rate.sleep()
    

#     # Create a Vector3
#     vector = tf.Vector3(1.0, 2.0, 3.0)

#     # Create a tf from the Vector3
#     transform = tf.Transform(vector)

#     # Print the transform
#     print(transform)
#     # Create a tf broadcaster
#     tf_broadcaster = tf.TransformBroadcaster()

#     # Set the transform frame ID and child frame ID
#     frame_id = "base_link"
#     child_frame_id = "transformed_frame"

#     # Set the translation and rotation from the Vector3
#     translation = (vector.x, vector.y, vector.z)
#     rotation = (0.0, 0.0, 0.0, 1.0)  # No rotation

#     # Publish the transform
#     tf_broadcaster.sendTransform(translation, rotation, rospy.Time.now(), child_frame_id, frame_id)
