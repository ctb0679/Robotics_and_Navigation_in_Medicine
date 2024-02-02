import rnm
import rospy
from sensor_msgs.msg import PointCloud2

app = rnm.APP

app.new_session(suffix = 'rosbag_pointclouds')
app.start_controller(mode = 'rosbag')

rate = rospy.Rate(1)

start_time = rospy.get_time()

try:
    while (rospy.get_time() - start_time) < 25:
        app.record_pointcloud()

except:
    print("Error")

pointclouds = app.shelve['points2']


