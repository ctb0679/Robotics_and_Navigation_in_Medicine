import rnm
import open3d as o3d 


app = rnm.APP

app.start_controller(mode = 'real')

rospcl = app.panda.get_pointcloud()

pcl = rnm.convertCloudFromRosToOpen3d(rospcl)

o3d.visualization.draw_geometries([pcl])