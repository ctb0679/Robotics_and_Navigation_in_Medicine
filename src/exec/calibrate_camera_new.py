import rnm
import numpy as np
from pathlib import Path

app = rnm.APP
import cv2
print(cv2.__version__)

#app.load_session('/ home/ajb/rnm_data/28_06_09_rosbag_calibration')
#app.load_session('/home/ajb/rnm_data/28_06_09_2950')
app.load_session('/home/ajb/rnm_data_lab/29_06_16_1929_real_rec') # gives quite good results

#app.load_current()
app.calculate_calibration(N_imgs=len(app.shelve['calibration_base_from_gripper']))
#app.kinect.write_to_yaml(app.file_calibration_yaml)


