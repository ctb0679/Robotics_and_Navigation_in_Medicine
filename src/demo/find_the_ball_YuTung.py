import open3d as o3d
import numpy as np

# read pointcloud file
pcd = o3d.io.read_point_cloud("combinedWithCad_pcd.ply")

# Sphere localization using RANSAC algorithm
sphere_model, inliers = pcd.segment_plane(distance_threshold=0.0001,
                                          ransac_n=4,
                                          num_iterations=100000)

# Get the center point and radius of the sphere
center = sphere_model[0]
radius = sphere_model[1]

# Find the indices of the points within the bounds of the sphere
distances = np.linalg.norm(np.asarray(pcd.points) - center, axis=1)
indices = np.where(distances <= radius)[0]

print("indices：", indices)

# Convert pcd.colors to NumPy array
pcd.colors = np.asarray(pcd.colors)

# Color the sphere orange
color = np.array([1.0, 0.5, 0.0])  # orange（RGB）
pcd.colors[indices] = color

# visualize result
o3d.visualization.draw_geometries([pcd])

