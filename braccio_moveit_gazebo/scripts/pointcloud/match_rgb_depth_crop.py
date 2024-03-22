#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np


pc = rs.pointcloud()


pipe = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)

pipe.start(config)

try:

    frames = pipe.wait_for_frames()

    depth = frames.get_depth_frame()
    color = frames.get_color_frame()

    # These are the two different frames to align
    depth_image = np.asanyarray(depth.get_data())
    color_image = np.asanyarray(color.get_data())

    # The new color image
    # color_image_with_same_resolution = ?

finally:
    pipe.stop()