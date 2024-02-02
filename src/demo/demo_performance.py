import sympy as sp
import numpy as np
from numpy.linalg import norm
import pathlib
import sys
sys.path.append(str(pathlib.Path(__file__).absolute().parent.parent))
import rnm
import timeit

def DH_function_old(N = 7): # to imitate matrix multiplication of the old function
    mat = np.eye(4, 4)
    for i in range(0, N):
        mat = rnm.MDHmatrix(0,0,0,0) @ mat

    return mat

model = rnm.PandaModel()

n_iterations = 10000
avg_time_inplace = timeit.timeit("model._update_world_from_endeff()",
                         number=n_iterations, globals=globals()) / n_iterations

# typical: ≈180μs
print("Average Evaluation Time In-Place Implementation: %d μs" % (avg_time_inplace * 1000_000))

avg_time_recreation = timeit.timeit("DH_function_old()", number = n_iterations, globals=globals()) / n_iterations

# typical: ≈230μs
print("Average Matrix Reallocating Function: %d μs" % (avg_time_recreation*1000_000))


