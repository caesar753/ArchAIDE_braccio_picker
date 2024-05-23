#! /usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API


# Instantiate CvBridge

class CameraShooter(object):
    def __init__(self):
        self.bridge = CvBridge()
        # rospy.init_node('image_listener', anonymous=True)


    def image_callback(self,msg):
        # print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('camera_image.jpg', cv2_img)


    def shoot(self):
        rate = rospy.Rate(1)
        # Define your image topic
        image_topic = "/camera/color/image_raw"
        # Set up your subscriber and define its callback
        shooter = rospy.Subscriber(image_topic, Image, self.image_callback)
        rate.sleep()
        shooter.unregister()
        # rate.sleep()

    def rs_shooter(self):
        # Setup:
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device_from_file("../object_detection.bag")
        profile = pipe.start(cfg)

        # Skip 5 first frames to give the Auto-Exposure time to adjust
        for x in range(5):
            pipe.wait_for_frames()
        
        # Store next frameset for later processing:
        frameset = pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        point_cloud = frameset.get_pointcloud()

        # Cleanup:
        pipe.stop()
        print("Frames Captured")

        color = np.asanyarray(color_frame.get_data())
        plt.rcParams["axes.grid"] = False
        plt.rcParams['figure.figsize'] = [12, 6]
        plt.imshow(color)

        colorizer = rs.colorizer()
        colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        plt.imshow(colorized_depth)

        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        aligned_depth_frame = frameset.get_depth_frame()
        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

        # Show the two frames together:
        images = np.hstack((color, colorized_depth))
        plt.imshow(images)

        return color, colorized_depth

        
    # Spin until ctrl + c
    # rospy.spin()

# def main():
#    image_shoot()

# if __name__ == '__main__':
#     main()