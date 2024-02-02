#import pathlib
#import sys
#sys.path.append(str(pathlib.Path(__file__).absolute().parent.parent))
import rnm
from rnm.panda_model import _set_MDHmatrix

import numpy as np
import unittest

class TestRobotModel(unittest.TestCase):

    def setUp(self) -> None:
        self.model = rnm.PandaModel()

    def assertListsAlmostEqual(self, x1, x2):
        self.assertEqual(len(x1), len(x2))
        for i in range(0, len(x1)):
            self.assertAlmostEqual(x1[i], x2[i])
    def assertNPArraysAlmostEqual(self, x1:np.ndarray, x2:np.ndarray):
        self.assertEqual(x1.shape, x2.shape)

        for i in range(0, len(x1.ravel())):
            self.assertAlmostEqual(x1.ravel()[i], x2.ravel()[i], places=4)

    def test_in_range(self):
        self.model.set_joint_angles(rnm.PANDA_QMIN)
        self.assertTrue(self.model.check_in_range())
        self.model.set_joint_angles(rnm.PANDA_QMAX)
        self.assertTrue(self.model.check_in_range())
        

    def test_mdh_matrix(self):
        N = 10
        for i in range(0, N):
            alpha, a, theta, d = np.random.rand(4)
            MDH1 = rnm.MDHmatrix(alpha, a, theta, d)
            MDH2 = np.eye(4, 4)
            _set_MDHmatrix(MDH2, alpha, a, theta, d)
            self.assertNPArraysAlmostEqual(MDH1, MDH2)

    def test_endeffector(self):
        self.assertListsAlmostEqual(self.model.endeffector_position(), [0.088, 0.0, 0.926, 1.0])
        self.assertListsAlmostEqual(self.model.endeffector_position(), self.model.joint_pos(7))

    def test_kinematics_against_reference(self):
        poses = rnm.load_poses_txt(rnm.APP.file_poses)
        configurations = rnm.load_joints_txt(rnm.APP.file_joints)

        for (pose, config) in zip(poses, configurations):
            self.assertNPArraysAlmostEqual(self.model.transformation_matrix(config), pose)
            self.assertNPArraysAlmostEqual(self.model.position(config), pose[:4, 3])
            self.model.set_joint_angles(config)
            self.assertNPArraysAlmostEqual(self.model.world_from_endeff, pose)

            # test max velocity, etc

if __name__ == '__main__':
    unittest.main()
