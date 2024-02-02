import sympy as sp
import numpy as np
import rnm

# symbolic utility functions

def npsymvec(length, name):
    "create a numpy column vector of sympy variables named name[i] and its matrix symbol"
    X = sp.var(name)
    matsym = sp.MatrixSymbol(X, length, 1)
    return matsym, np.array([matsym[i] for i in range(0, length)])


class IIKinematics:
    def __init__(self, model) -> None:
        self.model = model

    def build_IK_error_func(self):
        """
        build the inverse kinematics (IK) error function symbolically
        ```
            Ψ(Θ) := 0.5(endeff_pos(θ) - target_position)² + 0.5(endeff_orientation - target_orientation)²
        ```
        where `target` is the 4×1 homogeneous column vector that should be reached
        """

        # todo correct to 7 (make np-vec with additional fixed 0)
        N_unknowns = 8
        self.target_position_symbol, target_position = npsymvec(
            3, 'target_position')
        self.target_orientation_symbol, target_orientation = npsymvec(
            3, 'target_orientation')

        # build the symbolic matrix expression to the endeffector
        self.theta_symbols = sp.MatrixSymbol('theta', N_unknowns, 1)
        mat = np.eye(4)

        for i in range(len(self.model.a)-1, -1, -1):
            mat = rnm._MDHmatrix(self.model.a[i], self.model.d[i], np.cos(self.model.alpha[i]), np.sin(
                self.model.alpha[i]), sp.cos(self.theta_symbols[i]), sp.sin(self.theta_symbols[i])) @ mat

        def dot2(xs): return np.dot(xs, xs)
        # build error expression
        self.error_expr = dot2((mat @ rnm.e0)[0:3] - target_position) / \
            2 + dot2((mat @ rnm.e_z)[0:3] - target_orientation) / 2

        # build derivative
        self.gradient_expr = [
            sp.diff(self.error_expr, theta) for theta in self.theta_symbols]

    def _build_concrete_exprs(self, target_position, target_orientation):

        target_position_mat = sp.Matrix(3, 1, target_position[0:3])
        target_orientation_mat = sp.Matrix(3, 1, target_position[0:3])

        substitutions = [(self.target_position_symbol, target_position_mat),
                         (self.target_orientation_symbol, target_orientation_mat)]

        self.concrete_error_expr = self.error_expr.subs(substitutions)
        self.concrete_gradient_expr = [error_gradient_i.subs(
            substitutions) for error_gradient_i in self.gradient_expr]

    def error_gradient(self, thetas: np.ndarray):
        subs = {self.theta_symbols: sp.Matrix(len(thetas), 1, thetas)}
        return np.array([error_gradient_i.evalf(subs=subs) for error_gradient_i in self.concrete_gradient_expr])

    def error_func(self, thetas):
        subs = {self.theta_symbols: sp.Matrix(len(thetas), 1, thetas)}
        return self.concrete_error_expr.evalf(subs=subs)

    def configuration_from_cartesian(self, target_position, target_orientation):
        """
            Return a configuration for `(target_position, target_orientation)` by 
            incrementally improving the configuration estimate, starting from the current position.
            (In other words, implementing incremental inverse kinematics)
        """
        self._build_concrete_exprs(target_position, target_orientation)

        configuration = rnm.find_minimum_GD(
            self.error_gradient, self.model.theta)

        if configuration is None:
            raise ValueError("Did not find a suitable configuration for (position, orientation): %s" % str(
                target_position, target_orientation))

        return configuration
