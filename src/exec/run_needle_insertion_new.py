import rnm
import open3d as o3d
        

app = rnm.APP

joints_file = app.folder_rnm_data.joinpath(
    'custom_poses_needle_trajectory.txt')
joint_positions = rnm.load_joints_txt(joints_file)

#app.load_current()
app.load_session('/home/ajb/rnm_data_lab/29_06_16_3930_real_pointcloud')
#app.start_controller(mode = 'real')

app.calculate_needle_path(start_config=joint_positions[0], select_via_mouse=False)
app.draw_needle_path()

app.run_needle_insertion()