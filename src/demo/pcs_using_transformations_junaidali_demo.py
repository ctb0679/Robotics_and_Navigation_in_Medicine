import os
import open3d as o3d
import shelve
import rnm

def stitch_point_clouds(pcd_files, poses):
    # Create an empty point cloud to store the stitched result
    stitched_pcd = o3d.geometry.PointCloud()

    for i, pcd_file in enumerate(pcd_files):
        # Load each point cloud file
        pcd = o3d.io.read_point_cloud(pcd_file)

        # Get the corresponding pose from the poses dictionary
        pose = poses["scanning_base_from_gripper"][i]

        # Transform the point cloud using the pose
        pcd.transform(pose)

        # Concatenate the current point cloud with the stitched point cloud
        stitched_pcd += pcd

    return stitched_pcd

# Provide the directory containing the point cloud files
directory = '/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording/pointclouds'

# Get a list of .pcd files in the directory
pcd_files = [os.path.join(directory, file) for file in os.listdir(directory) if file.endswith('.pcd')]

# Load the poses from the .shelve file
poses_file = '/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording/session.shelve'
poses = shelve.open(poses_file)

# Stitch the point clouds using the poses
stitched_point_cloud = stitch_point_clouds(pcd_files, poses)

# Save the stitched point cloud as a .pcd file
output_directory = '/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording'
output_file = os.path.join(output_directory, 'stitched.pcd')
o3d.io.write_point_cloud(output_file, stitched_point_cloud)

# Close the poses file
poses.close()

