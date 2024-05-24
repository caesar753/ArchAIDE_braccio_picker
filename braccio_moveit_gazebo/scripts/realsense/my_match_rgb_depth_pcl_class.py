#!/usr/bin/env python3

import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
from matplotlib import pylab
import open3d.visualization
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

import open3d


# print("Environment Ready")

class RealSensePointCloud:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.cfg = rs.config()
        self.align = rs.align(rs.stream.depth)
        self.pc = rs.pointcloud()
        self.w = 0
        self.h = 0

    def start_pipeline(self, bag_file):
        self.cfg.enable_device_from_file(bag_file)
        self.profile = self.pipeline.start(self.cfg)

    def wait_for_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        only_color = frames.get_color_frame()
        color_frame = aligned_frames.get_color_frame()
        
        self.w = rs.video_frame(depth_frame).width
        self.h = rs.video_frame(depth_frame).height
        return only_color, color_frame, depth_frame

    def save_color (self, color, filename):
        # color_image = np.asanyarray(color_frame.get_dat/a())
        plt.rcParams["axes.grid"] = False
        plt.rcParams['figure.figsize'] = [12, 6]
        plt.figure(num='Only color frame')
        plt.imshow(color)
        plt.savefig(filename)
        plt.close()

    def calculate_pointcloud(self, depth_frame):
        points = self.pc.calculate(depth_frame)
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(self.h, self.w, 3)
        return verts
    
    def verts_roi(self, verts, crop_box):
        verts_roi = verts[crop_box[2]:crop_box[3], crop_box[0]:crop_box[1]]
        verts_roi = verts_roi.reshape(-1,3)
        return verts_roi

    def crop_color_image(self, image, crop_box):
        print(image.shape)
        image = image[crop_box[2]:crop_box[3], crop_box[0]:crop_box[1]]
        print(image.shape)

        plt.rcParams["axes.grid"] = False
        plt.rcParams['figure.figsize'] = [12, 6]
        plt.figure(num='Color aligned with depth')
        plt.imshow(image)
        plt.show()

        return image

    def create_pointcloud(self, verts, color_image):
        color_image = color_image.reshape(-1,3)
        pcd = open3d.geometry.PointCloud()
        pcd.points = open3d.utility.Vector3dVector(verts.astype(np.float32) / 255)
        pcd.colors = open3d.utility.Vector3dVector(color_image.astype(np.float32) / 255)
        return pcd

# #Example usage:
# rs_pc = RealSensePointCloud()
# rs_pc.start_pipeline("./rosbag_walk/d435i_walk_around.bag")
# color_frame, depth_frame = rs_pc.wait_for_frames()
# color_image = np.asanyarray(color_frame.get_data())
# depth_image = np.asanyarray(depth_frame.get_data())
# cropped_color_image = rs_pc.crop_image(color_image, 200, 600, 200, 600)
# cropped_depth_image = rs_pc.crop_image(depth_image, 200, 600, 200, 600)
# cropped_verts = rs_pc.calculate_pointcloud(cropped_depth_image)
# cropped_pcd = rs_pc.create_pointcloud(cropped_verts, cropped_color_image)
# open3d.visualization.draw_geometries([cropped_pcd])
