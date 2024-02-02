from scipy.optimize import minimize
from scipy.optimize import minimize, Bounds
from rnm.panda_model import *

class IIKinematics:
    """
        Numerical incremental inverse kinematics, using the `PandaModel` for numerical gradient calculation
    """

    def __init__(self, model = None) -> None:
        self.model = model if not model is None else PandaModel()

    def _matrix_error_func(self, thetas):
        self.model.set_joint_angles(thetas)
        return np.sum((self.model.world_from_endeff - self.target_matrix)**2)

    def matrix_inverse_kinematics(self, target_matrix, start_configuration=0.5 * (PANDA_QMIN + PANDA_QMAX), options={'ftol': 1e-12, 'disp': False}):
        self.target_matrix = target_matrix

        bounds = Bounds(PANDA_QMIN, PANDA_QMAX)

        result = minimize(self._matrix_error_func, start_configuration,
                          method='SLSQP', bounds=bounds, options=options)
        
        return result.x