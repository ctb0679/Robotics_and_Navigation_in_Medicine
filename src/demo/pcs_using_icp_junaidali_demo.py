import os
import open3d as o3d
import rnm

def stitch_point_clouds(pcd_files):
    # Create an empty point cloud to store the stitched result
    stitched_pcd = o3d.geometry.PointCloud()

    # Load the first point cloud as the reference
    reference_pcd = o3d.io.read_point_cloud(pcd_files[0])
    stitched_pcd += reference_pcd

    # Apply ICP registration for each subsequent point cloud
    for pcd_file in pcd_files[1:]:
        # Load the next point cloud
        next_pcd = o3d.io.read_point_cloud(pcd_file)

        # Perform ICP registration
        icp_result = o3d.pipelines.registration.registration_icp(
            next_pcd, reference_pcd, max_correspondence_distance=0.02,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))

        # Apply the transformation to the next point cloud
        next_pcd.transform(icp_result.transformation)

        # Concatenate the transformed point cloud with the stitched point cloud
        stitched_pcd += next_pcd

        # Update the reference point cloud for the next iteration
        reference_pcd = stitched_pcd

    return stitched_pcd

# Provide the directory containing the point cloud files
directory = '/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording/pointclouds'

# Get a list of .pcd files in the directory
pcd_files = [os.path.join(directory, file) for file in os.listdir(directory) if file.endswith('.pcd')]

# Stitch the point clouds using ICP
stitched_point_cloud = stitch_point_clouds(pcd_files)

# Save the stitched point cloud as a .pcd file
output_directory = '/home/junaidali/Downloads/fused_sessions/29_06_16_real_recording'
output_file = os.path.join(output_directory, 'stitched_with_icp.pcd')
o3d.io.write_point_cloud(output_file, stitched_point_cloud)

