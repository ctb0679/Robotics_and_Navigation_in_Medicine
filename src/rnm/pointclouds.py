import open3d
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from ctypes import *


# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]


BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8


def convert_rgbUint32_to_tuple(rgb_uint32): return (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 &
                                      0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)


def convert_rgbFloat_to_tuple(rgb_float): return convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def convertCloudFromRosToOpen3d(ros_cloud):

    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(
        ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    
    open3d_cloud = open3d.geometry.PointCloud() 
    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        # (why cannot put this line below rgb?)
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

        # Get rgb
        # Check whether int or float
        # if float (from pcl::toROSMsg)
        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:
            rgb = [convert_rgbFloat_to_tuple(rgb)
                   for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb)
                   for x, y, z, rgb in cloud_data]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    return open3d_cloud


def mesh_marker(point, color=np.array((1, 0.0, 0)), radius=1e-3):
    """
        Create a marker mesh to assist 
    """
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.translate(point)
    sphere.paint_uniform_color(color)
    return sphere


def select_color(pcl, color, dst, invert_selection=False):
    npcolors = np.array(pcl.colors)
    has_right_color = np.sum(np.abs(npcolors - color), axis=1) <= dst
    indices = np.array(range(npcolors.shape[0]))[has_right_color]
    return pcl.select_by_index(indices, invert=invert_selection)

def select_color_normalized(pcl, color, color_dst, brightness_dst, invert_selection = False):
    npcolors = np.array(pcl.colors)
    brightness = np.sum(np.colors, axis = 1)
    normalized_colors = npcolors / brightness
    has_right_color = np.sum(np.abs(normalized_colors - color), axis=1) <= color_dst and np.abs(brightness) <= brightness_dst

    indices = np.array(range(npcolors.shape[0]))[has_right_color]
    return pcl.select_by_index(indices, invert=invert_selection)


RGB_BALL_TARGET = np.array((239, 121, 115))/255  # +- 10
RGB_BLANKET = np.array((5, 93, 81))/255  # +- 10
RADIUS_BALL = 20e-3  # 20 mm estimated (?)

def pcl_find_target(pcl, show_pointclouds = True):
    pcl_ball = select_color(pcl, RGB_BALL_TARGET, 10/255)

    o3d.visualization.draw_geometries([pcl_ball])

    pcl_ball2, _ = pcl_ball.remove_statistical_outlier(
        nb_neighbors=10, std_ratio=RADIUS_BALL)
    
    print(f"Found ball with {len(pcl_ball2.points)} points")
    if len(pcl_ball2.points) < 5:
        return None
    

    # opt: check for radius of the ball to check for quality of estimate

    center = np.average(np.array(pcl_ball2.points), axis=0)
    return center

def pcl_remove_environment(pcl):
    pcl = select_color(pcl, RGB_BLANKET, 10/255, invert_selection=True)


def select_targets_viz(pcl):
        """Select the entry and target points in pointclouds"""
        # Shift + Left click to pick the points
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcl)
        vis.run()
        vis.destroy_window()
        indices = vis.get_picked_points()
        return pcl.points[indices[0]]
