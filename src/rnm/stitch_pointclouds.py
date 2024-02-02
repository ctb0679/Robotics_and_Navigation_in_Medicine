import open3d as o3d
import os
import pathlib
import numpy as np
import shelve


def loadPointClouds():
    file_path = (
        os.path.dirname(pathlib.Path(__file__).parent.resolve().parent.resolve())
        + "/data/pcl_files/"
    )
    pcd_paths = []
    for path in os.listdir(file_path):
        pcd_paths.append(file_path + path)

    point_clouds = []
    for pcd_path in pcd_paths:
        pcd = o3d.io.read_point_cloud(pcd_path)
        point_clouds.append(pcd)
    return point_clouds


def loadTransformations():
    db_path = (
        os.path.dirname(pathlib.Path(__file__).parent.resolve().parent.resolve())
        + "/data/"
    )
    db = shelve.open(db_path + "session.shelve")
    T_base_from_gripper = []
    T_cam2gripper = db["kinect"].cam2gripper
    for T in db["scanning_base_from_gripper"]:
        T_base_from_gripper.append(T)
    return (T_cam2gripper, T_base_from_gripper)


def draw_registration_result(source, target, transformation):
    """Visualize transformation"""
    source.transform(transformation)
    o3d.visualization.draw_geometries(
        [source, target],
        zoom=0.4559,
        front=[0.6452, -0.3036, -0.7011],
        lookat=[1.9892, 2.0208, 1.8945],
        up=[-0.2779, -0.9482, 0.1556],
    )


def preprocess_point_cloud(pcd, voxel_size=0.1):
    """Downsample pointcloud, estimate normals, then compute a FPFH feature for each point"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100),
    )
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, source, target):
    """Reads a source point cloud and a target point cloud from two files. They are misaligned with an identity matrix as transformation."""
    trans_init = np.asarray(
        [
            [0.0, 0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    source.transform(trans_init)
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(
    source_down, target_down, source_fpfh, target_fpfh, voxel_size
):
    """Global registarion using RANSAC"""
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3,
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold
            ),
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999),
    )
    return result


def refine_registration(source, target, result_ransac, voxel_size):
    """Point-to-plane ICP to further refine the alignment."""
    distance_threshold = voxel_size * 0.4
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        distance_threshold,
        result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )
    return result


# Load the point clouds and transformations
point_clouds = loadPointClouds()
T_cam2gripper, T_base_from_gripper = loadTransformations()
# Define voxel_size
voxel_size = 0.05

# Apply the transformations
transformed_point_clouds = []
# Apply transformations in the shelve file before global registration
i = 0
for pcd in point_clouds:
    pcd.transform(transformation=T_base_from_gripper[i] @ T_cam2gripper)
    i += 1

for i in range(len(point_clouds) - 1):
    source = point_clouds[i]
    target = point_clouds[i + 1]
    # Prepare dataset for global registration
    (
        source,
        target,
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
    ) = prepare_dataset(voxel_size, source, target)

    # Get global registration result and the initial transformation
    result_ransac = execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size
    )

    trans_init = result_ransac.transformation

    # Apply point-to-point ICP
    threshold = 0.02
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    )
    source.transform(reg_p2p.transformation)

    # Add the transformed point cloud
    transformed_point_clouds.append(source)

combined_cloud = o3d.geometry.PointCloud()
# Add them together
for pcd in transformed_point_clouds:
    # apply transformations
    combined_cloud += pcd


o3d.visualization.draw_geometries(
    [combined_cloud],
    zoom=0.3412,
    front=[0.4257, -0.2125, -0.8795],
    lookat=[2.6172, 2.0475, 1.532],
    up=[-0.0694, -0.9768, 0.2024],
)
