import rnm
import rospy

app = rnm.APP

app.new_session()
app.start_controller(mode = 'real')
joints_file = app.folder_rnm_data.joinpath('custom_poses_2.txt')

# take pictures
app.run_object_recording(joints_file=joints_file, record_photos=True, augment_poses=False)

rospy.loginfo('Recorded checkerboard photos')

calculate_calibration = True
if calculate_calibration:
    app.calculate_calibration(N_imgs = len(app.shelve['calibration_base_from_gripper']))
    app.kinect.write_to_yaml(app.file_calibration_yaml)




