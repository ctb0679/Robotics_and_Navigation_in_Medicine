import rnm
import unittest
import numpy as np

N = 7


class TestTrajectory(unittest.TestCase):

    def setUp(self):
        super().setUp()
        self.model = rnm.PandaModel()

    def test_trajectory(self):
        start_end_configs = [(np.zeros(7), np.ones(7))]
        for (q_start, q_end) in start_end_configs:
            smooth_traj = rnm.smooth_linear(q_start, q_end)
            duration = smooth_traj.duration
            self.assertNPArraysAlmostEqual(smooth_traj.position(0), q_start)
            self.assertNPArraysAlmostEqual(
                smooth_traj.position(duration), q_end)
            # todo add checks for speed and maximum acceleration etc

    # short consistency checks for trajectory implementation
    def test_trajectories(self):
        Ntests = 10
        for i in range(0, Ntests):
            start_config = rnm.to_range(np.random.rand(
                N), rnm.PANDA_QMIN, rnm.PANDA_QMAX)
            end_config = rnm.to_range(np.random.rand(
                N), rnm.PANDA_QMIN, rnm.PANDA_QMAX)

            duration = 10

            trajectories = [rnm.QuinticTrajectory(
                start_config, end_config, duration)]

            for trajectory in trajectories:
                self.assertNPArraysAlmostEqual(
                    trajectory.position(0), start_config)
                self.assertNPArraysAlmostEqual(
                    trajectory.position(duration), end_config)

    # todo: implement velocity control and tests

    def test_orientation_matrix(self):
        for i in range(0, 10):
            vec = 5 * np.random.rand(3)
            mat = rnm.orientation_matrix(vec)
            self.assertNPArraysAlmostEqual(
                mat @ np.transpose(mat), np.eye(3, 3))

    def position(self, configuration):
        self.model.set_joint_angles(configuration)
        return self.model.endeffector_position()[0:3]

    def test_needle_trajectory(self):
        start_config = 0.5*(rnm.PANDA_QMIN + rnm.PANDA_QMAX)
        end_config = 0.4 * rnm.PANDA_QMIN + 0.6 * rnm.PANDA_QMAX

        x_start = self.position(start_config)[0:3]
        x_end = self.position(end_config)[0:3]

        traj = rnm.construct_needle_trajectory(x_start, x_end, 10)

        self.assertNPArraysAlmostEqual(
            self.position(traj.position(0)), x_start)
        self.assertNPArraysAlmostEqual(
            self.position(traj.position(traj.duration)), x_end)

        # test e.g. distance to linear path

    def assertNPArraysAlmostEqual(self, x1: np.ndarray, x2: np.ndarray):
        self.assertEqual(x1.shape, x2.shape)
        for i in range(0, len(x1)):
            self.assertAlmostEqual(x1.ravel()[i], x2.ravel()[i], places=5)
