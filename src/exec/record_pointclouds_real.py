import rnm
import rospy
from pathlib import Path

#path = Path.home().joinpath('rnm_data/29_06_16_1929_real_rec')

app = rnm.APP
#app.load_session(path)
app.load_current()

joints_file = app.folder_rnm_data.joinpath('new_poses_phantom.txt')

rospy.loginfo('Starting pointcloud recording')
app.start_controller(mode = 'real')

app.run_object_recording(joints_file = joints_file, record_photos = False, record_pointclouds=True)

rospy.loginfo('Finished pointcloud recording')