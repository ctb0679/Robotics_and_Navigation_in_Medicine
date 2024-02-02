# -*- coding: utf-8 -*-
import open3d as o3d

# give file path 
filename = "/Users/christy/Desktop/TUHH/4th semester/Robotics and Navigation in Medicine/project/code&material/RNM_code/combined_pointcloud"

# read pointclouds file
pcd = o3d.io.read_point_cloud(filename)

# visualize result
o3d.visualization.draw_geometries([pcd])
