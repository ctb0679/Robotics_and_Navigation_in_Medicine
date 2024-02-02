import numpy as np
from numpy import pi
from numpy.linalg import norm
from rnm.utils import *


# homogeneous origin
e0 = np.array([0.0, 0, 0, 1])
e_z = np.array([0.0, 0, 1, 1])  # homogeneous z axis

# modified Denavit-Hartenberg parameters for the Panda robot (7 joints, flange)
PANDA_MDH_PARAMS = [[0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0],  # a
                    [0.0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0.0],  # alpha
                    # d params (d1 : d7, flange)
                    [0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]  # theta

# panda limits
PANDA_QMAX = np.array([2.8973, 1.7628, 2.8973, -0.0698,
                      2.8973, 3.7525, 2.8973])  # rad
PANDA_QMIN = np.array([-2.8973, -1.7628, -2.8973, -
                      3.0718, -2.8973, -0.0175, -2.8973])  # rad
PANDA_QDOTMAX = np.array(
    [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])  # rad/s
PANDA_QACCMAX = np.array([15, 7.5, 10, 12.5, 15, 20, 20])  # rad / s^2
PANDA_QJERKMAX = np.array(
    [7500, 3750, 5000, 6250, 7500, 10000, 10000])  # rad / s^3
PANDA_TAUMAX = np.array([87, 87, 87, 87, 12, 12, 12])  # Nm
PANDA_TAUDOTMAX = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000])  # Nm /s


class PandaModel:
    """
        The robot model, storing joint angle data, 
        robot configuration (modified Denavit-Hartenberg) and 
        collision data
    """

    def __init__(self, DHparams=None):
        if DHparams is None:
            DHparams = PANDA_MDH_PARAMS
        self.a = list(DHparams[0])
        self.alpha = list(DHparams[1])
        self.d = list(DHparams[2])
        self.theta = list(DHparams[3])
        self._update_world_from_endeff()

    def Njoints():
        return 7

    def joint_angles(self):
        return self.theta[0:7]
    # to :exclusive; _from inclusive

    def transformation_matrix(self, theta=None, _to=-1, _from=None):
        """
        calculate the transformation matrix M_{to_from}
        acting such that 
        to_vector = M_{to_from} @ from_vector
        (where to_vector is a vector in the coordinate system with index _to)
        """

        if theta is None:
            theta = self.theta
        else:
            theta_new = np.copy(self.theta)
            theta_new[0:len(theta)] = theta
            theta = theta_new
        if _from == None:
            _from = len(self.a)-1

        mat = np.eye(4, dtype=np.float64)
        temp = np.eye(4, dtype=np.float64)
        # step := sign to allow stepping down if _from > _to
        for i in range(_from, _to, 1 if _from <= _to else -1):
            _set_MDHmatrix(temp, self.alpha[i],
                           self.a[i], theta[i], self.d[i])
            np.matmul(temp, mat, out=mat)
        return mat
    
    def position(self, angles):
        assert len(angles) == PandaModel.Njoints()
        return self.transformation_matrix(theta=angles)[:4, 3]
    
    def _update_world_from_endeff(self):
        """recalculate the transformation matrix"""
        self.world_from_endeff = self.transformation_matrix(_from=None, _to = -1)

    def set_joint_angles(self, thetas):
        self.theta[0:7] = thetas  # only the first seven angles are adaptive
        self._update_world_from_endeff()

    def endeffector_position(self):  # opt arg frame = world
        return self.world_from_endeff[:, 3]

    def endeffector_orientation(self):
        return self.world_from_endeff[:, 2]

    def joint_pos(self, i):
        """
        return the world coordinates of the coordinate system center of joint i
        """
        mat = self.transformation_matrix(_to = -1, _from = i)
        return mat[:, 3]

    def check_in_range(self):
        return all(PANDA_QMIN <= self.theta[0:7]) and all(self.theta[0:7] <= PANDA_QMAX)


def MDHmatrix(alpha, a, theta, d):
    """
    modified denavit hartenberg; note that alpha,a is for  while z is for i
    i-1 ← i ∀ i ∈ 1:N
    Rot_{x, α, i} ∘ T_{x, a, i-1} ∘ Rot_{z, θ, i} ∘ T_{z, d}
    """
    return _MDHmatrix(a, d, np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta))

# source for matrix formula: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters#Modified_DH_parameters


# more general matrix to allow for symbolic substitutions
def _MDHmatrix(a, d, cosalpha, sinalpha, costheta, sintheta):
    ct = costheta
    st = sintheta
    ca = cosalpha
    sa = sinalpha
    # use tuple because it is faster than a list
    return np.array(((ct, -st, 0.0, a),
                     (st * ca, ct * ca, -sa, -d * sa),
                     (st * sa, ct * sa, ca, d * ca),
                     (0.0, 0.0, 0.0, 1.0)))


def _set_MDHmatrix(A, alpha, a, theta, d):
    """
    update a 4 × 4 matrix A in-place to the modified Denavit Hartenberg (MDH) matrix
    only update necessary values
    """

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    A[0, 0] = ct
    A[0, 1] = -st
    A[0, 3] = a
    A[1, 0] = st * ca
    A[1, 1] = ct * ca
    A[1, 2] = -sa
    A[1, 3] = -d * sa
    A[2, 0] = st * sa
    A[2, 1] = ct * sa
    A[2, 2] = ca
    A[2, 3] = d * ca

