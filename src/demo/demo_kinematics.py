import rnm
import numpy as np
from numpy.linalg import norm
import time
import pathlib

model = rnm.PandaModel()
N = 7


def random_configuration():
    return rnm.to_range(np.random.rand(N), rnm.PANDA_QMIN, rnm.PANDA_QMAX)


iikinematics = rnm.IIKinematics()


filepath = pathlib.Path(__file__).parents[2].joinpath('data/pose.txt')

target_matrices = rnm.load_poses_txt(str(filepath))

for target_matrix in target_matrices:
    config = iikinematics.matrix_inverse_kinematics(target_matrix)

    model.set_joint_angles(config)

    err = norm(model.world_from_endeff - target_matrix)
    print('Error %s' % str(err))

print('Random configurations')

for i in range(0, 10):
    config = random_configuration()
    model.set_joint_angles(config)
    target_matrix = model.world_from_endeff
    
    t1 = time.time()
    angles = iikinematics.matrix_inverse_kinematics(target_matrix)
    print("Calculated inverse kinematics in %d ms" %
          ((time.time() - t1) * 1000))
    model.set_joint_angles(angles)
    error = norm(model.world_from_endeff - target_matrix)  # todo
    if error >= 1e-5:
        print("Warning: Error is high:")
        print("Total Error:" + str(error))
    print('Error %s' % str(err))
