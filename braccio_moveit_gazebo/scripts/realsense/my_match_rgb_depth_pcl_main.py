#!/usr/bin/env python3
import open3d
import numpy as np
from my_match_rgb_depth_pcl_class import RealSensePointCloud


def main():
    rs_pc = RealSensePointCloud()
    rs_pc.start_pipeline("./rosbag_walk/d435i_walk_around.bag")
    only_color, color_frame, depth_frame = rs_pc.wait_for_frames()
    color_image = np.asanyarray(color_frame.get_data())
    only_color_image = np.asanyarray(only_color.get_data())
    print(color_image.shape)
    print(only_color_image.shape)
    # depth_image = np.asanyarray(depth_frame.get_data())
    precropped_verts = rs_pc.calculate_pointcloud(depth_frame)

    #DISPLAY COMPLETE POINTCLOUD
    precropped_verts_sh = precropped_verts.reshape(-1,3)
    complete_pcd = rs_pc.create_pointcloud(precropped_verts_sh, color_image)
    open3d.visualization.draw_geometries([complete_pcd])
    
    #DISPLAY CROPPED POINTCLOUD
    crop_box = [200, 600, 200, 600]
    cropped_only_color = rs_pc.crop_color_image(only_color_image, crop_box)
    cropped_color_image = rs_pc.crop_color_image(color_image, crop_box)
    cropped_verts = rs_pc.verts_roi(precropped_verts, crop_box)
    cropped_pcd = rs_pc.create_pointcloud(cropped_verts, cropped_color_image)
    open3d.visualization.draw_geometries([cropped_pcd])

    rs_pc.save_color(only_color_image, 'color_frame.png')
    rs_pc.save_color(cropped_only_color, 'cropped_color_frame.png')

if __name__ == "__main__":
    main()  # Call the main function