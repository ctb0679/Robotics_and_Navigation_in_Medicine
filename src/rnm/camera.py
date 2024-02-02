import rospy
import cv2
import numpy as np
import shelve
import yaml

from rnm.utils import *


class MonoCamera:
    """
    A single camera with its calibration parameters
    """

    def __init__(self, K_intrinsics, T_extrinsics, distortion, avg_rms) -> None:
        self.K_intrinsics = K_intrinsics
        self.T_extrinsics = T_extrinsics
        self.distortion = distortion
        self.avg_rms = avg_rms

    def distortion_string(self):
        ks = np.zeros(6)
        ps = np.zeros(2)
        # order
        # k₁,k₂, p₁, p₂, k₃, k₄, k₅, k₆ [,s₁ ,… , s₄, τ_x, τ_y]
        ks[:2] = self.distortion.ravel()[0:2]
        ps = self.distortion.ravel()[2:4]
        ks[2:6] = self.distortion.ravel()[4:8]
        kparam_string = "\n".join((f"k{i+1} : {ks[i]}" for i in range(0, 6)))
        pparam_string = "\n".join((f"p{i+1} : {ps[i]}" for i in range(0, 2)))
        return kparam_string + "\n" + pparam_string

    def calibration_state(self):
        return f"""
Camera Matrix:
{self.K_intrinsics}
Distortion:
{self.distortion_string()}
Extrinsics:
{self.T_extrinsics}
        """
    
    def yaml_repr(self): # intrinsics
        ks = np.zeros(6)
        ps = np.zeros(2)
        ks[:2] = self.distortion.ravel()[0:2]
        ps = self.distortion.ravel()[2:4]
        ks[2:6] = self.distortion.ravel()[4:8]
        ks = list(map(float, ks))
        ps = list(map(float, ps))

        return {'camera_matrix': py_matrix(self.K_intrinsics), 'k1': ks[0], 'k2' : ks[1], 'k3': ks[2], 'k4': ks[3], 'k5': ks[4], 'k6': ks[5], 'p1':ps[0], 'p2':ps[1], 'rms':float(self.avg_rms)}


class KinectCamera:
    """
        Class containing configuration and calibration data of the kinect
        with functions to handle the transformational aspects (transform a pointcloud from camera coordinates to link0 coordinates, stitch pointclouds)
        Fields:
        cam_rgb : rgb-camera
        cam_depth : K-Matrix for Depth / IR - camera
        T_cam_arm (Transformation Camera ← Arm; from Hand-Eye-Calibration)
    """

    def __init__(self, cam_depth: MonoCamera, cam_rgb: MonoCamera) -> None:
        self.cam_rgb = cam_rgb
        self.depth_cam = cam_depth

    def calibration_string(self):
        return f"""
Depth Camera:
{self.depth_cam.calibration_state()}

RGB Camera:
{self.cam_rgb.calibration_state()}
        """
    

    
    def write_to_yaml(self, file):
        with open(file, 'wt') as io:
            extrinsics = {'rgb_to_depth': py_matrix(self.cam_rgb.T_extrinsics), 'depth_to_rgb' : py_matrix(np.linalg.inv(self.cam_rgb.T_extrinsics))}
            dictionary = {'extrinsics': extrinsics, 'intrinsic_color': self.cam_rgb.yaml_repr(), 'intrinsic_depth': self.depth_cam.yaml_repr()}
            yaml.dump(dictionary, io, Dumper = yaml.CSafeDumper)

    def transform_to_link0(self, recorded_pointcloud, joint_angles):
        """
          Transform the given pointcloud to a pointcloud in link0 - coordinates (Robot Base Effector)
        """
        pass

    def stitch_to_pointcloud(self, images, poses):
        pass

CHECKERBOARD_DIMS = (8, 5)
QUAD_SIZE = 40e-3  # 40 mm

TERMINATION_CRITERIA = (cv2.TERM_CRITERIA_EPS +
                        cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def load_images(filenames, shelve_file):
    """
        Load images and a pose shelve file to create a CalibrationImages object
    """
    with shelve.open(str(shelve_file)) as db:
        base_from_gripper = db['calibration_base_from_gripper']
        return CalibrationImages(filenames, base_from_gripper)


class CalibrationImages:
    """
        An calibration image dataset, with associated information
    """

    def __init__(self, image_files, base_from_gripper=None) -> None:
        """
        base_from_gripper : homogeneous transformation matrix
        """
        self.image_files = image_files
        self.base_from_gripper = base_from_gripper

    def compute_checkboard_corners(self):

        # 3D points for each checkerboard image
        self.objpoints = []
        # 2D points for each checkerboard image
        self.imgpoints = []

        # world coordinates for 3D points
        board_world_coords = np.zeros(
            (1, CHECKERBOARD_DIMS[0] * CHECKERBOARD_DIMS[1], 3), np.float32)
        board_world_coords[0, :, :2] = QUAD_SIZE * np.mgrid[0:CHECKERBOARD_DIMS[0],
                                                            0:CHECKERBOARD_DIMS[1]].T.reshape(-1, 2)

        self.valid_indices = []

        for (i, img_file) in enumerate(self.image_files):
            img = cv2.imread(str(img_file))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            success, corners = cv2.findChessboardCorners(
                gray, CHECKERBOARD_DIMS, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

            if success == True:
                self.objpoints.append(board_world_coords)
                # refining pixel coordinates for given 2d points.
                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), TERMINATION_CRITERIA)

                self.imgpoints.append(corners2)

                self.shape = img.shape[:2]
                self.valid_indices.append(i)
            else:
                self.objpoints.append(None)
                self.imgpoints.append(None)
                rospy.loginfo(
                    'No checkerboard found in image: \n%s' % str(img_file))

    def compute_calibrated_camera(self):
        self.objpoints
        self.rms_error, K_intrinsics, distortion, rvecs, tvecs, stdDeviationIntrinsics, stdDeviationExtrinsics, self.perViewErrors = cv2.calibrateCameraExtended(
            _filter_none(self.objpoints), _filter_none(self.imgpoints), self.shape, None, None, flags=cv2.CALIB_RATIONAL_MODEL)
        
        self.rvecs = rvecs
        self.tvecs = tvecs
        print(f"Camera calibration rms reprojection error: {self.rms_error} px (< 1 is ok)")
        if self.rms_error >= 1:
            print("Warning: Calibration error is high")
            print("Per-view errors:")
            print(self.perViewErrors)

        return MonoCamera(K_intrinsics, None, distortion, self.rms_error)

    def compute_hand_eye_calibration(self, kinect: KinectCamera):

        valid_base_from_gripper = _select_indices(
            self.base_from_gripper, self.valid_indices)
        R_gripper2base = [T[:3, :3] for T in valid_base_from_gripper]
        t_gripper2base = [T[:3, 3] for T in valid_base_from_gripper]

        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base, t_gripper2base=t_gripper2base, R_target2cam=self.rvecs, t_target2cam=self.tvecs) # , method = 
        

        kinect.cam2gripper = hom_matrix(R_cam2gripper, t_cam2gripper.ravel())
        # test  base_from_gripper @ cam2gripper @ transformation_sample_point constant (= base_from_object)
        object_transformations = [hom_matrix(cv2.Rodrigues(src = self.rvecs[i])[0], self.tvecs[i].ravel()) for i in range(0, len(self.valid_indices))]
        base_from_object = np.array([self.base_from_gripper[i] @ kinect.cam2gripper @ object_transformations[i0] for (i0, i) in enumerate(self.valid_indices)])
        # test how the large the deviation is
        _avg = np.average(base_from_object, axis=0)
        dev_translation = np.average(np.abs(base_from_object[:, :, 3]- _avg[:, 3]), axis = 0)
        dev_matrix = np.average(np.abs(base_from_object - _avg), axis = 0)
        print(f"Translation average error: {dev_translation}")
        print(f"Homogeneous matrix average deviation: {dev_matrix}")

        return R_cam2gripper, t_cam2gripper


def _select_indices(list, indices):
    return [list[i] for i in indices]


def _filter_none(list):
    return [x for x in list if not x is None]


def compute_stereo_calibration(cam_rgb: MonoCamera, cam_ir: MonoCamera, rgb_imgs: CalibrationImages, ir_imgs: CalibrationImages):
    # pick out images for which both rgb and ir calibrations were successful (imgpoints != None)
    N_imgs = len(ir_imgs.image_files)
    indices = [i for i in range(0, N_imgs) if not (
        ir_imgs.imgpoints[i] is None or rgb_imgs.imgpoints[i] is None)]

    objpoints = _select_indices(rgb_imgs.objpoints, indices)
    ir_imgpoints = _select_indices(ir_imgs.imgpoints, indices)
    rgb_imgpoints = _select_indices(rgb_imgs.imgpoints, indices)
    
    # other opencv version
    #stereo_rms, K_rgb, distortion_rgb, K_ir, distortion_ir, R, T, E, F, rvecs, tvecs, perViewRMS = cv2.stereoCalibrateExtended(
    #    objpoints, rgb_imgpoints, ir_imgpoints, cam_rgb.K_intrinsics, cam_rgb.distortion, cam_ir.K_intrinsics, cam_ir.distortion, rgb_imgs.shape, R = None, T = None, flags= cv2.CALIB_RATIONAL_MODEL) # cv2.CALIB_FIX_INTRINSICS
    
    stereo_rms, K_rgb, distortion_rgb, K_ir, distortion_ir, R, T, E, F,  rvecs, tvecs, perViewRMS = cv2.stereoCalibrateExtended(
        objpoints, rgb_imgpoints, ir_imgpoints, cam_rgb.K_intrinsics, cam_rgb.distortion, cam_ir.K_intrinsics, cam_ir.distortion, rgb_imgs.shape, R = None, T = None, flags= cv2.CALIB_RATIONAL_MODEL) # cv2.CALIB_FIX_INTRINSICS
    if not stereo_rms:
        raise RuntimeWarning("Stereo calibration not successful")

    cam_ir.T_extrinsics = np.identity(4, dtype=np.float64)
    cam_rgb.T_extrinsics = hom_matrix(R, np.reshape(T, 3))

    print(f"Stereo calibration rms error: {stereo_rms} (px)")

    return KinectCamera(cam_ir, cam_rgb)


# yaml
def py_matrix(npmat):
    return [list(map(float, arr)) for arr in npmat]

