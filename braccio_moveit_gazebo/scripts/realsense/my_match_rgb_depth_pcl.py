#!/usr/bin/env python3
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
from matplotlib import pylab
import open3d.visualization
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

import open3d


print("Environment Ready")

#START REALSENSE PIPELINE
pipeline = rs.pipeline()
cfg = rs.config()

#READ FROM ROSBAG 
# cfg.enable_device_from_file("./outdoors.bag")
cfg.enable_device_from_file("./rosbag_walk/d435i_walk_around.bag")

#READ FROM ACTUAL REALSENSE
# cfg.enable_stream(rs.stream.depth)

#LOAD PROFILE
profile = pipeline.start(cfg)

align = rs.align(rs.stream.depth)

#CREATE REALSENSE POINTCLOUD OBJECT
pc = rs.pointcloud()

#WAIT FOR FRAMES AND GET THE COLOR_FRAME AND DEPTH_FRAME
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()

#DISPLAY ONLY COLOR_FRAME
only_color = np.asanyarray(color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.figure(num='Only color frame')
plt.imshow(only_color)
plt.show()

#ALIGN COLOR_FRAME WITH DEPTH_FRAME
aligned = align.process(frames)
aligned_color_frame = aligned.get_color_frame()

#DISPLAY ALIGNED COLOR_FRAME WITH DEPTH_FRAME
color_image = np.asanyarray(aligned_color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.figure(num='Color aligned with depth')
plt.imshow(color_image)
plt.show()
print(color_image.shape)
color_image_sh = color_image.reshape(-1,3)

#depth_frame = frames.first(rs.stream.depth)
#CALCULATE POINT FOR RS_POINTCLOUD FROM DEPTH_FRAME
points = pc.calculate(depth_frame)

#GET WIDTH AND HEIGHT OF DEPTH_FRAME
w = rs.video_frame(depth_frame).width
h = rs.video_frame(depth_frame).height

#TRANSFORM points INTO np.array (verts) with (h,w) shape
verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(h, w, 3)
print(verts.shape)
verts_sh = verts.reshape(-1,3)

#BOUNDING BOX
xmin, xmax, ymin, ymax = 200, 600, 200, 600 # BB

#CROP ONLY COLOR IMAGE  AND DISPLAY
color_crop = only_color[ymin:ymax, xmin:xmax, :]
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.figure(num='Cropped color frame')
plt.imshow(color_crop)
plt.show()

#CROP COLOR IMAGE ALIGNED TO DEPTH AND DISPLAY
coloraligned_crop = color_image[ymin:ymax, xmin:xmax, :]
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.figure(num='Croped color aligned with depth')
plt.imshow(coloraligned_crop)
plt.show()

#CROP REGION OF INTEREST (ROI) FROM VERTS (FROM points = pc.calculate(depth_frame))
roi = verts[ymin:ymax, xmin:xmax, :]
roi = roi.reshape(-1,3)
print(roi.shape)
roi_color = color_image[ymin:ymax, xmin:xmax, :]
roi_color = roi_color.reshape(-1,3)
plt.figure(num='Cropped RS pointcloud')
print(roi_color.shape)

#CREATE OPEN3D POINTCLOUD AND FILL WITH VERTS POINTS
pcd = open3d.geometry.PointCloud()
pcd.points = open3d.utility.Vector3dVector(verts_sh.astype(np.float32)/255)
pcd.colors = open3d.utility.Vector3dVector(color_image_sh.astype(np.float32)/255)
open3d.visualization.draw_geometries([pcd])

#CREATE OPEN3D POINTCLOUD AND FILL WITH ROI POINTS
pcd_cropped = open3d.geometry.PointCloud()
pcd_cropped.points = open3d.utility.Vector3dVector(roi.astype(np.float32)/255)
pcd_cropped.colors = open3d.utility.Vector3dVector(roi_color.astype(np.float32)/255)
open3d.visualization.draw_geometries([pcd_cropped])
# o3d.write_point_cloud(str(count)+label[i]+'.ply', pcd)
