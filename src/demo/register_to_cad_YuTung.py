import open3d as o3d
from open3d.pipelines.registration import registration_icp, TransformationEstimationPointToPoint, ICPConvergenceCriteria

# read combined pointcloud
combined_pcd = o3d.io.read_point_cloud("combined_pointcloud.ply")

# read cad
cad_mesh = o3d.io.read_triangle_mesh("Skeleton_Target.stl")

# turn the cad file into pointclous
cad_pcd = cad_mesh.sample_points_uniformly(number_of_points=10000)

cad_pcd = cad_pcd.scale(0.001,[0.0,0.0,0.0])
# align Point Cloud
transformation = registration_icp(
    combined_pcd, cad_pcd, max_correspondence_distance=0.02,
    estimation_method=TransformationEstimationPointToPoint(),
    criteria=ICPConvergenceCriteria(max_iteration=200))

# transform the pointclouds of the stitching result and align it to the coordinate system of the CAD model
combined_pcd.transform(transformation.transformation)

# introduce the color of cad to pointcloud
combined_pcd.colors = cad_pcd.colors


# save result
o3d.io.write_point_cloud("combinedWithCad_pcd.ply", combined_pcd)

cad_pcd += combined_pcd

# visualize the result
#o3d.visualization.draw_geometries([combined_pcd])
o3d.visualization.draw_geometries([cad_pcd])

