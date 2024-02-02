import open3d as o3d
import os
import pathlib
import shelve
import numpy as np


class PointCloud:
    def __init__(self, pcl=None) -> None:
        self.T_cam2gripper = None
        self.T_base_from_gripper = []
        self.pcl = self.loadPointCloud()
        self.pcl_points = None
        self.needle_target_point = None
        self.T_target = None

    def loadTransformations(self):
        db_path = (
            os.path.dirname(pathlib.Path(__file__).parent.resolve().parent.resolve())
            + "/data/"
        )
        db = shelve.open(db_path + "session.shelve")
        self.T_cam2gripper = db["kinect"].cam2gripper
        for T in db["scanning_base_from_gripper"]:
            self.T_base_from_gripper.append(T)

    def loadPointCloud(self):
        self.loadTransformations()
        # Get the pcd file paths
        file_path = (
            os.path.dirname(pathlib.Path(__file__).parent.resolve().parent.resolve())
            + "/data/pcl_files/"
        )
        pcd_paths = []
        for path in os.listdir(file_path):
            pcd_paths.append(file_path + path)
        # Start stitching point clouds
        point_clouds = []
        for pcd_path in pcd_paths:
            pcd = o3d.io.read_point_cloud(pcd_path)
            point_clouds.append(pcd)
        combined_cloud = o3d.geometry.PointCloud()
        for i, pcd in enumerate(point_clouds):
            # apply transformations
            tf = self.T_base_from_gripper[i] @ self.T_cam2gripper
            pcd.transform(tf)
            combined_cloud += pcd
        self.points = combined_cloud.points
        return combined_cloud

    def select_targets(self):
        """Select the entry and target points in pointclouds"""
        # Shift + Left click to pick the points
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(self.pcl)
        vis.run()
        vis.destroy_window()
        points = vis.get_picked_points()
        # Save the first clicked point as entry point, second as needle target point
        self.needle_target_point = points[0]
        return points

    def calculateTransformation(self):
        """Calculate the Homogenous Transformation Matrix in World Coordinates"""
        target_position = self.points[self.needle_target_point]
        self.T_target = o3d.geometry.get_rotation_matrix_from_xyz(target_position)


p = PointCloud()
p.select_targets()
