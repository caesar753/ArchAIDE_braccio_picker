import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API

# from open3d import *
import open3d as o3d


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pc = rs.pointcloud()
points = rs.points()
# Start streaming
profile = pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)
count = 0

try:
    while count < 20000:
        count +=1

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = align.process(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        points = pc.calculate(aligned_depth_frame)

        vertices = np.asanyarray(points.get_vertices(dims=2))


        w = aligned_depth_frame.get_width()
        image_Points = np.reshape(vertices , (-1,w,3))
        if not aligned_depth_frame or not aligned_color_frame:
            continue
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(aligned_color_frame.get_data())
        rgb = color_image[...,::-1]
        image = Image.fromarray(color_image)
        image_detected , left , right , bottom , top , label = yolo.detect_image(image, depth_image,count)
        for i in range(0,len(label)):

            left_cord = left[i]
            right_cord = right[i]
            bottom_cord = bottom[i]
            top_cord = top[i]
            vertices_interest = image_Points[top_cord:bottom_cord,left_cord:right_cord,:].reshape(-1,3)

            color_interest = color_image[top_cord:bottom_cord,left_cord:right_cord,:].reshape(-1,3)
            pcd = o3d.PointCloud()
            pcd.points = o3d.Vector3dVector(vertices_interest.astype(np.float32)/255)
            pcd.colors = o3d.Vector3dVector(color_interest.astype(np.float32)/255)
            o3d.draw_geometries([pcd])
            o3d.write_point_cloud(str(count)+label[i]+'.ply', pcd)
        image_detected=np.asarray(image_detected)


        # Show images
        cv2.imshow('RealSense', image_detected)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

finally:

    pipeline.stop()
