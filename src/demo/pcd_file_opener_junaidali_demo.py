import open3d as o3d

# Read the .pcd file
# pcd_file = "/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording/stitched_with_icp.pcd"
pcd_file = "/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording/stitched.pcd"
point_cloud = o3d.io.read_point_cloud(pcd_file)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])

