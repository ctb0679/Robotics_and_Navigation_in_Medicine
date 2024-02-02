import pathlib
import shutil
from datetime import datetime
import shelve
import atexit
import open3d as o3d

from rnm.utils import *
from rnm.camera import *
from rnm.panda_controller import *
from rnm.panda_model import *
from rnm.pointclouds import *

# control functions to operate the robot on high level

# setup_controller()

# [mount camera, place checkerboard]
# take_calibration_pictures()
# calculate_calibration()
# [remove checkerboard, place phantom]
# take_phantom_pictures()
# stitch_pointcloud()
# registrate_pointcloud()
# [unmount camera, mount_needle]
# drive_needle


class ApplicationController:
    """
        High-level user interface to coordinate operations
    """

    # Fields
    # operating_folder : folder for intermediary files, defaults to ~/rnm_data/session-date
    # controller : PandaController
    # point_cloud_scanned
    # point_cloud_registered

    def __init__(self, mode = 'real') -> None:
        self.mode = mode
        self.shelve = None
        atexit.register(self.end_session)
        self.folder_rnm_data = pathlib.Path(
            __file__).parents[2].joinpath('data')
        self.file_poses = self.folder_rnm_data.joinpath('pose.txt')
        self.file_joints = self.folder_rnm_data.joinpath('joints.txt')
        self.file_trk = self.folder_rnm_data.joinpath('trk.txt')
        self.file_skeleton_target = self.folder_rnm_data.joinpath(
            'Skeleton_Target.stl')
        self.file_calibration_yaml = self.folder_rnm_data.joinpath(
            'calibration.yaml') if mode != 'real' else pathlib.Path.home().joinpath('catkin_ws/src/azure_kinect_ros_driver/config/calibration.yaml') 
        self.file_custom_joints = self.folder_rnm_data.joinpath(
            'custom_poses.txt')
        self.panda = PandaController()

    def folder_data_base(self):
        return pathlib.Path.home().joinpath('rnm_data')

    def new_session(self, folder=None, suffix='', overwrite=False):
        if type(folder) == str:
            folder = pathlib.Path(folder)
        if folder is None:
            folder = self.folder_data_base().joinpath('current')

        if overwrite:
            if folder.is_dir():
                shutil.rmtree(str(folder))

        folder.mkdir(parents=True, exist_ok=False)

        self.setup_folders(folder)
        self.folder_calibration_imgs.mkdir(exist_ok=False)
        self.folder_pointclouds.mkdir()

        self.load_session(folder)
        self.shelve['creation_stamp'] = datetime.now()

    def setup_folders(self, folder):
        self.folder = folder
        self.folder_calibration_imgs = self.folder.joinpath('imgs/')
        self.folder_pointclouds = self.folder.joinpath('pointclouds/')
        self.file_shelve = self.folder.joinpath('session.shelve')

    def load_current(self):
        self.load_session(self.folder_data_base().joinpath('current'))

    def save_current(self, suffix):
        """
            Move the 'current' session folder to one stamped with its creation date
        """
        stamp = self.shelve['creation_stamp']
        session_str = stamp.strftime('%d_%m_%H_%M%S')
        target_folder = self.folder_data_base().joinpath(session_str + suffix)
        self.folder.rename(target_folder)

    def load_session(self, folder):
        if folder == None:
            subfolders = list(self.folder_data_base().iterdir())
            # choose typographically largest string
            subfolders.sort()
            folder = self.folder_data_base().joinpath(subfolders[-1])
            print(f'Loading folder {folder}')
        if type(folder) == str:
            folder = pathlib.Path(folder)
        self.setup_folders(folder)
        self.shelve = shelve.open(str(self.file_shelve))

    def end_session(self):
        # store shelve
        if not self.shelve is None:
            self.shelve.close()
            self.shelve = None

    def start_controller(self, mode):
        self.panda.init(name='rnm_application', mode=mode)
        rate = rospy.Rate(0.5)
        if mode != 'rosbag':
            while self.panda.joint_states is None:
                rate.sleep()
            rospy.loginfo("Controller is ready")

    def restart_controller(self, mode):
        # untested
        rospy.signal_shutdown('Restarting RNM PandaController node')
        rospy.spin()  # wait until shutdown
        print("Rospy is down. Restarting panda node")
        self.panda = PandaController()
        self.start_controller(mode)

    def load_scanned_mesh(self):
        mesh = o3d.io.read_triangle_mesh(str(self.file_skeleton_target))
        
        self.scanned_mesh = mesh.sample_points_uniformly(number_of_points=10000)

    def record_pointcloud(self):
        pcl = self.panda.get_pointcloud()
        self.shelve['points2'] = self.shelve.get('points2', []) + [pcl]
        self.shelve.sync()

        filename = self.folder_pointclouds.joinpath(
            f'pcl_{self.pointcloud_index}.pcd')
        o3dpcl = convertCloudFromRosToOpen3d(pcl)
        o3d.io.write_point_cloud(str(filename), o3dpcl)

        self.pointcloud_index += 1
        rospy.loginfo('Recorded pointcloud')

    def record_photos(self, folder, index):
        rgb_img = self.panda.take_image_cv(rgb=True)
        ir_img = self.panda.take_image_cv(rgb=False)

        filename = str(folder.joinpath(
            'pic_' + str(index)))
        cv2.imwrite(filename + '_rgb.png', rgb_img)
        cv2.imwrite(filename + '_ir.png', ir_img)

    def _record_pose(self, transformation_key, joint_states_key):
        self.panda.model.set_joint_angles(self.panda.joint_states)
        self.shelve[transformation_key] = self.shelve.get(transformation_key, []) + \
            [self.panda.model.world_from_endeff]
        self.shelve[joint_states_key] = self.shelve.get(joint_states_key, []) + \
            [self.panda.joint_states]
        self.shelve.sync()

    def _take_photo_callback(self):
        if rospy.get_time() - self.last_picture_time >= self.min_picture_interval and self.panda.is_resting():

            if self.is_recording_photo:
                self.record_photos(folder = self.folder_calibration_imgs, index = self.picture_index)
                self.picture_index += 1
                self._record_pose('calibration_base_from_gripper',
                                  'calibration_joint_states')
                rospy.loginfo('Took photos')
            if self.is_recording_pointcloud:
                self.record_photos(folder = self.folder_pointclouds, index = self.pointcloud_index)
                self.record_pointcloud()
                self._record_pose('scanning_base_from_gripper',
                                  'scanning_joint_states')
                rospy.loginfo('Recorded pointcloud')

            self.last_picture_time = rospy.get_time()

    def run_object_recording(self, joints_file=None, record_photos=True, record_pointclouds=False, augment_poses = False):

        assert (record_photos != record_pointclouds) 
        # note that recording both screws up pose data

        if joints_file is None:
            joints_file = self.file_custom_joints

        self.recording_configurations = load_joints_txt(
            str(joints_file))  # list of joint configurations

        self.last_picture_time = rospy.get_time() + 3
        self.min_picture_interval = 6
        self.picture_index = 0
        self.pointcloud_index = 0
        self.is_recording_photo = record_photos
        self.is_recording_pointcloud = record_pointclouds

        self.panda.on_loop_callbacks = [self._take_photo_callback]

        self.recording_configurations.insert(0, self.panda.joint_states)

        if augment_poses:
            for i in range(0, len(self.recording_configurations)-1):
                self.recording_configurations.insert(2*i+1, 0.5 * (self.recording_configurations[2*i] + self.recording_configurations[2*i + 1]))
            

        # trajectories = construct_quintic_takephoto_trajectories(self.recording_configurations)
        trajectories = construct_takephoto_trajectories(
            self.recording_configurations, smooth_linear2, wait_time=self.min_picture_interval-1.0)

        self.panda.trajectory_queue = trajectories
        rospy.loginfo('Start recording object')
        self.panda.run_controller(-1)
        rospy.loginfo('Ended recording object')

    def calculate_calibration(self, N_imgs=10):
        img_indices = list(range(0, N_imgs))

        rgb_files = [str(self.folder_calibration_imgs.joinpath("pic_" + str(i) + "_rgb.png"))
                     for i in img_indices]
        ir_files = [str(self.folder_calibration_imgs.joinpath("pic_" + str(i) + "_ir.png"))
                    for i in img_indices]

        self.rgb_imgs = CalibrationImages(
            rgb_files, self.shelve['calibration_base_from_gripper'])
        self.ir_imgs = CalibrationImages(
            ir_files, self.shelve['calibration_base_from_gripper'])

        self.rgb_imgs.compute_checkboard_corners()
        rgb_cam = self.rgb_imgs.compute_calibrated_camera()

        self.ir_imgs.compute_checkboard_corners()
        ir_cam = self.ir_imgs.compute_calibrated_camera()

        self.kinect = compute_stereo_calibration(
            cam_rgb=rgb_cam, cam_ir=ir_cam, rgb_imgs=self.rgb_imgs, ir_imgs=self.ir_imgs)
        
        #self.kinect.write_to_yaml(self.file_calibration_yaml)
        self.rgb_imgs.compute_hand_eye_calibration(self.kinect)
        
        self.shelve['kinect'] = self.kinect
        self.shelve.sync()

    def stitch_pointclouds(self):
        
        # convert to open3d pointclouds (optional: voxel downsampling)
        # transform pointclouds to base frame
        # base_from_gripper * kinect.cam2gripper
        # stitch together using icp
        # voxel downsample



        pass

    def calculate_registration(self):
        # todo
        # register using icp
        pass

    def create_needle_model(self):
        DHparams = np.copy(PANDA_MDH_PARAMS)
        NEEDLE_LENGTH = 88e-3 # in meters
        DHparams = np.hstack([DHparams, np.reshape(np.array((0, 0, NEEDLE_LENGTH, 0)), (4, 1))])
        self.needle_model = PandaModel(DHparams)

    def calculate_needle_path(self, start_config = None, show_pointclouds = True, select_via_mouse = False):
        if start_config is None:
            start_config = self.panda.joint_states

        self.create_needle_model()

        for i in range(0, len(self.shelve['scanning_base_from_gripper'])):


            self.pcl = o3d.io.read_point_cloud(str(self.folder_pointclouds.joinpath(f'pcl_{i}.pcd')))
            
            transformation = self.shelve['scanning_base_from_gripper'][i] @ self.shelve['kinect'].cam2gripper
            self.pcl = self.pcl.transform(transformation)
            if show_pointclouds:
                o3d.visualization.draw_geometries([self.pcl])

            if select_via_mouse:
                center_target = select_targets_viz(self.pcl)
            else:
                center_target = pcl_find_target(self.pcl)

            if center_target is None:
                continue
            print(f'Center of target at base coords: \n {center_target}')
            start_position = self.needle_model.position(start_config)[0:3]

            self.needle_trajectory = construct_needle_trajectory(start_position, center_target, 100, model = self.needle_model)
            return


    def draw_needle_path(self, N = 100):
        # visualize needle path with skeleton in base frame
        
        markers = []
        for t in np.linspace(0, self.needle_trajectory.duration, N):
            theta = self.needle_trajectory.position(t)
            needle_marker = mesh_marker(self.needle_model.position(theta)[0:3])
            endeffector_marker = mesh_marker(self.panda.model.position(theta)[0:3], color = [0, 0, 1]) # endeffector in blue, needle pos in red
            markers += [needle_marker, endeffector_marker]
        
        o3d.visualization.draw_geometries(markers + [self.pcl])

    def run_needle_insertion(self):
        assert not self.needle_trajectory is None
        to_start_trajectory = smooth_linear2(self.panda.joint_states, self.needle_trajectory.position(0))
        self.panda.trajectory_queue += [to_start_trajectory, self.needle_trajectory]
        print('Start insertion')
        self.panda.run_controller(-1)
        print('Insertion finished')


    def fuse_session_shelves(self, folder: pathlib.Path):
        """
        Fuse the session contained in 'folder' into the one given by `self`, that is, move all files and merge the shelve files
        """
        shelve2 = shelve.open(str(folder.joinpath('session.shelve')))
        dct = dict(shelve2)
        dct.update(self.shelve)
        self.shelve.update(dct)
        self.shelve.sync()


APP = ApplicationController()
