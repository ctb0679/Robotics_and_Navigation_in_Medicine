import open3d as o3d
import rnm
import copy
import numpy as np


def draw_pointcloud_comparison(pcl0, pcl1):
    pcl0 = copy.deepcopy(pcl0)
    pcl1 = copy.deepcopy(pcl1)
    pcl0.paint_uniform_color([1, 0.706, 0])
    pcl1.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([pcl0, pcl1])


app = rnm.APP

app.load_session('/home/ajb/rnm_data_lab/29_06_16_3930_real_pointcloud')

# stitch and transform first two pointclouds

pcl0 = o3d.io.read_point_cloud(str(app.folder_pointclouds.joinpath('pcl_0.pcd')))
pcl1 = o3d.io.read_point_cloud(str(app.folder_pointclouds.joinpath('pcl_1.pcd')))

pcl0 = pcl0.transform(app.shelve['scanning_base_from_gripper'][0] @ app.shelve['kinect'].cam2gripper)

pcl1 = pcl1.transform(app.shelve['scanning_base_from_gripper'][1] @ app.shelve['kinect'].cam2gripper)

threshold = 0.02

evaluation = o3d.pipelines.registration.evaluate_registration(
    pcl0, pcl1, threshold)
print(evaluation)

radius = 0.005

pcl0.estimate_normals()
pcl1.estimate_normals()

pcl0.voxel_down_sample(1e-3)
pcl1.voxel_down_sample(1e-3)

ball_position = rnm.pcl_find_target(pcl0, show_clouds=False)

bounding_box = o3d.geometry.OrientedBoundingBox(center = ball_position, R = np.eye(3), extent = np.full((3, ), 0.4)) # 40cm around the ball

pcl0 = pcl0.crop(bounding_box)
pcl1 = pcl1.crop(bounding_box)

pcl0 = rnm.pcl_remove_environment(pcl0)
pcl1 = rnm.pcl_remove_environment(pcl1)

registration_result = o3d.pipelines.registration.registration_colored_icp(
    pcl1, pcl0, radius, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationForColoredICP(),
    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                      relative_rmse=1e-4,
                                                      max_iteration=100))

#registration_result = o3d.pipelines.registration.registration_icp(
#    pcl1,
#    pcl0,
#    max_correspondence_distance=0.02,
#    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
#    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
#        max_iteration=200
#    ),
#)
# note: registration result is applied to first argument (source cloud)

pcl1.transform(registration_result.transformation)

print("ICP Registration result")
print(registration_result)

# ball_position = [0.3883, 0.0889, 0.0788]


print(f"Ball position: {ball_position}")


o3d.visualization.draw_geometries([pcl0, pcl1])




