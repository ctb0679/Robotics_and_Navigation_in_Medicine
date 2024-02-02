import numpy as np
import cv2

# utility functions

def hom(vec):
    "returned homogeneous version of `vec` by appending a `1.0` at the end"
    res = np.zeros((len(vec)+1,))
    res[0:len(vec)] = vec
    res[-1] = 1.0
    return res

def hom_matrix(matrix3x3, translation):
    mat4 = np.zeros((4, 4), dtype=np.float64)
    mat4[0:3, 0:3] = matrix3x3
    mat4[0:3, 3] = translation
    mat4[3, 3] = 1.0
    return mat4
    
def dotsquare(vec):
    return np.dot(vec, vec)

def normalize(vec):
    return vec / np.linalg.norm(vec)

def rms(arr):
    return np.sqrt(np.sum(arr ** 2) / len(arr))


def orientation_matrix(target_z_axis):
    orientation = normalize(target_z_axis)
    e2 = normalize(np.cross(orientation, e_(1, 3)))
    e3 = normalize(np.cross(orientation, e2))
    
    return np.transpose(np.vstack((e2, e3, orientation)))

def bucket(x, mins, maxs):
    "return `x` clipped to the n-dimensional box defined by `mins` and `maxs`"
    return np.minimum(np.maximum(mins, x), maxs)


def to_range(ratio, mins, maxs):
    """
    Scale and transform the [0-1] values in ratio linearly to the interval [mins, maxs]
    """
    return ratio * mins + (np.ones_like(ratio) - ratio) * maxs


def load_poses_txt(filename):
    """Get all the poses from the text file and find the suitable joint configurations for each of them"""

    with open(filename) as file:
        lines = file.readlines()
        poses = [np.reshape(line2ndarray(line), (4,4)) for line in lines]
        return poses

def write_poses_txt(filename, arrays):
    with open(filename, mode = 'a') as file:
        for array in arrays:
            file.write(' '.join([str(number) for number in array]))
            file.write('\n')
            


def line2ndarray(line : str):
    return np.array(list(map(float, line.strip().split())))

def load_joints_txt(file):
    """
    load joints from a text file
    """
    with open(file) as joints_file:
        lines = joints_file.readlines()
        joint_states = [line2ndarray(line) for line in lines]
        return joint_states



def e_(i, N):
    "Construct the `i`-th unit vector in `ℝ^N` as `np.ndarray`"
    arr = np.zeros(N, dtype=np.float64)
    arr[i] = 1.0
    return arr
