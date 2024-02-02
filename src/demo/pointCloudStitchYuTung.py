# -*- coding: utf-8 -*-

import open3d as o3d

# from pipelines.registration import (
#     registration_icp,
#     TransformationEstimationPointToPoint,
#     ICPConvergenceCriteria,
# )
import os
import pathlib
import shelve
import numpy as np

db_path = (
    os.path.dirname(pathlib.Path(__file__).parent.resolve().parent.resolve()) + "/data/"
)
db = shelve.open(db_path + "session.shelve")
T_cam2gripper = db["kinect"].cam2gripper
T_base_from_gripper_arr = []
for T in db["scanning_base_from_gripper"]:
    T_base_from_gripper_arr.append(T)
# Define the number of points
numOfPc = 5

# Create an empty point cloud to save the stitching result
combined_pcd = o3d.geometry.PointCloud()

# handle mutiple pointclouds in loop
for i in range(numOfPc):
    # read pointclouds
    file_path = (
        os.path.dirname(pathlib.Path(__file__).parent.resolve().parent.resolve())
        + "/data/pcl_files/"
    )
    pcd_paths = []
    for path in os.listdir(file_path):
        pcd_paths.append(file_path + path)
    # filename = filename = f"/Users/christy/Desktop/TUHH/4th semester/Robotics and Navigation in Medicine/project/code&material/RNM_code/rnm_data/29_06_16_3930_real_poincloud/pointclouds/pcl_{i}.pcd"

    pcd = o3d.io.read_point_cloud(pcd_paths[i])

    # Align Point Cloud
    if i > 0:
        transformation = o3d.pipelines.registration.registration_icp(
            pcd,
            combined_pcd,
            max_correspondence_distance=0.02,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=200
            ),
        )
        tf = T_base_from_gripper_arr[i] @ T_cam2gripper
        pcd.transform(tf)
        pcd.transform(
            transformation.transformation
        )  # Transform the point cloud and align it to the coordinate system of the previous point cloud

    # Add the current point cloud to the stitching result
    combined_pcd += pcd

# save the result
# o3d.io.write_point_cloud("combined_pointcloud.ply", combined_pcd)
# visualize the result
o3d.visualization.draw_geometries([combined_pcd])
