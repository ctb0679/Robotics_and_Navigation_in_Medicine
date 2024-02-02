# trajectory planning
from typing import Any
import numpy as np
import scipy.interpolate

from rnm.panda_model import *
from rnm.inverse_kinematics import *



class Trajectory:
    """
        Abstract class defining the interface of a trajectory.

        A trajectory is a time-parameterized path through the configuration space.
    """

    def position(self, time: float):
        "return the position in c-space at time"

    def velocity(self, time):
        "return the configuration velocity at time"

    def acceleration(self, time):
        "return the acceleration at time"

    # duration : float (required field)


class RestingTrajectory(Trajectory):
    def __init__(self, configuration, duration) -> None:
        super().__init__()
        self.configuration = configuration
        self.duration = duration

    def position(self, Δtime: float):
        return self.configuration
    def velocity(self, time):
        return np.zeros_like(self.configuration)
    def acceleration(self, time):
        return np.zeros_like(self.configuration)


class LinearTrajectory(Trajectory):
    def __init__(self, q_start, q_end, duration) -> None:
        super().__init__()
        self.duration = duration
        self.q_start = q_start
        self.q_end = q_end

    def position(self, Δt: float):
        alpha = Δt / self.duration
        return self.q_start * (1-alpha) + self.q_end * alpha
    
    def velocity(self, time):
        return (self.q_end - self.q_start) / self.duration

    def max_velocity(self):
        return self.velocity(0)
    def acceleration(self, time):
        return np.zeros_like(self.q_start)

def sine_interpolation(tau):
    return 0.5*(np.sin((2*tau-1.0) * np.pi / 2) + 1)


class QuinticTrajectory(Trajectory):
    """
    A robot path given as quintic polynomial in each joint angle
    """

    # https://frankaemika.github.io/docs/control_parameters.html#joint-trajectory-requirements
    def __init__(self, start_position, end_position, duration) -> None:
        super().__init__()
        self.duration = duration
        self.a0 = np.copy(start_position)
        self.a1 = np.zeros_like(start_position)
        self.a2 = np.zeros_like(start_position)
        self.a3 = (20 * end_position - 20 * start_position) / (2 * duration**3)
        self.a4 = (30 * start_position - 30 * end_position) / (2 * duration**4)
        self.a5 = (12 * end_position - 12 * start_position) / (2 * duration**5)

    def position(self, tau):
        return (
            self.a0
            + self.a1 * tau
            + self.a2 * tau**2
            + self.a3 * tau**3
            + self.a4 * tau**4
            + self.a5 * tau**5
        )

def construct_quintic_takephoto_trajectories(configurations, wait_time = 5, time_per_trajectory = 25):
    return construct_takephoto_trajectories(configurations,QuinticTrajectory, wait_time, time_per_trajectory)



def construct_takephoto_trajectories(configurations, trajectory_constructor, wait_time=5, *options):

    quintic_trajectories = [trajectory_constructor(
        configurations[i-1], configurations[i], *options) for i in range(1, len(configurations))]

    for i in range(0, len(quintic_trajectories)):
        resting_trajectory = RestingTrajectory(configurations[i+1], wait_time)
        quintic_trajectories.insert(2*i + 1, resting_trajectory)

    return quintic_trajectories


class MotionQuinticTrajectory:
    """
    A quintic polynomial to be used for robot motion, with only `q₀, q₄, q₅ != 0`
    """

    def __init__(self, duration, q0, q4, q5) -> None:
        self.duration = duration
        self.q0 = q0
        self.q4 = q4
        self.q5 = q5

    def __call__(self, t):
        return self.q0 + self.q4 * t**4 + self.q5 * t**5

    def position(self, t):
        return self(t)
    def velocity(self, t):
        return 4 * self.q4 * t ** 3 + 5 * self.q5 * t**4

    def acceleration(self, t):
        return 12 * self.q4 * t**2 + 20 * self.q5 * t ** 3

    def max_velocity(self):
        return self.velocity(self.duration)

    def max_acceleration(self):
        return self.acceleration(self.duration * 2 / 3)


def calculate_quintic(q0, q_dot_motion, qacc_max):
    """
        Calculate a `MotionQuinticTrajectory` that starts with zero (velocity, acceleration, jerk) in `q0` and ends with velocity `q_dot_motion` while accelerating maximally with `qacc_max`, calculating a suitable duration
    """
    duration = 64/9 * np.max(abs(q_dot_motion / qacc_max))

    q4 = q_dot_motion / duration ** 3
    q5 = - 3/5 * q_dot_motion / duration ** 4

    return MotionQuinticTrajectory(duration, q0, q4, q5)


class SmoothLinearTrajectory(Trajectory):
    """
        A trajectory mainly going linear in the main middle segment, with two starting and ending quintic polynomial segments to smoothly accelerate to 0
    """

    def __init__(self, quintic1: MotionQuinticTrajectory, quintic2: MotionQuinticTrajectory, linear: LinearTrajectory) -> None:
        super().__init__()
        self.quintic1 = quintic1
        self.quintic2 = quintic2
        self.linear = linear
        self.rest = RestingTrajectory(self.quintic2(0), 100)
        self.duration = quintic1.duration + quintic2.duration + linear.duration

    def segment(self, Δt):
        if Δt <= self.quintic1.duration:
            return self.quintic1, Δt
        elif Δt <= self.quintic1.duration + self.linear.duration:
            return self.linear, (Δt - self.quintic1.duration)
        elif Δt <= self.duration:
            return self.quintic2, (self.duration - Δt)
        else: 
            return self.rest, 0 # return last value (resting)
        
    def position(self, Δt):
        segment, Δt = self.segment(Δt)
        return segment.position(Δt)
    
    def velocity(self, Δt):
        segment, Δt = self.segment(Δt)
        if segment == self.quintic2:
            return -segment.velocity(Δt)
        else:
            return segment.velocity(Δt)
        
    def acceleration(self, Δt):
        segment, Δt = self.segment(Δt)
        return segment.acceleration(Δt)

    def max_velocity(self):
        return max(self.quintic1.max_velocity(), self.quintic2.max_velocity(), self.linear.max_velocity())
    
    def max_acceleration(self):
        return max(self.quintic1.max_acceleration(), self.quintic2.max_acceleration())
    
class PerJointTraj(Trajectory):
    def __init__(self, joint_trajectories) -> None:
        self.joint_trajectories = joint_trajectories
        self.duration = np.max([self.joint_trajectories[i].duration for i in range(0, len(joint_trajectories))])

    def position(self, time: float):
        return np.array([traj.position(time) for traj in self.joint_trajectories])
    
    def velocity(self, time):
        return np.array([traj.velocity(time) for traj in self.joint_trajectories])
    
    def acceleration(self, time):
        return np.array([traj.acceleration(time) for traj in self.joint_trajectories]) 
    
def smooth_linear(q_start, q_end):
    sec_factor = 0.01
    qdot_max = PANDA_QDOTMAX * sec_factor
    qacc_max = PANDA_QACCMAX * sec_factor / 5
    Δq = q_end - q_start
    durations = np.abs(Δq) / qdot_max
    critical_joint = np.argmax(durations)

    duration = durations[critical_joint]
    qdot_motion = Δq / duration


    joint_trajectories = []
    for i in range(0, len(q_start)):
        quintic1 = calculate_quintic(q_start[i], qdot_motion[i], qacc_max[i])
        quintic2 = calculate_quintic(q_end[i], -qdot_motion[i], qacc_max[i])
        q_linear_start = quintic1(quintic1.duration)
        q_linear_end = quintic2(quintic2.duration)
        linear_duration = (q_linear_end - q_linear_start) / qdot_motion[i]
        linear_traj  = LinearTrajectory(quintic1(quintic1.duration), quintic2(quintic2.duration), linear_duration)
        joint_trajectories.append(SmoothLinearTrajectory(quintic1, quintic2, linear_traj))

    return PerJointTraj(joint_trajectories)



def smooth_linear2(q_start, q_end):
    """
    Create a smooth linear trajectory interpolating from q_start to q_end in a suitable amount of time
    WARNING: discontinuous, only left as alternative until smooth_linear2 has been tested in simulator
    """
    sec_factor = 0.02
    qdot_max = PANDA_QDOTMAX * sec_factor
    qacc_max = PANDA_QACCMAX * sec_factor / 5
    Δq = q_end - q_start
    durations = np.abs(Δq) / qdot_max
    critical_joint = np.argmax(durations)

    duration = durations[critical_joint]
    qdot_motion = Δq / duration

    quintic1 = calculate_quintic(q_start, qdot_motion, qacc_max)
    quintic2 = calculate_quintic(q_end, -qdot_motion, qacc_max)
    q_linear_start = quintic1(quintic1.duration)
    q_linear_end = quintic2(quintic2.duration)

    Δq_linear = q_linear_end - q_linear_start
    duration_linear = np.max(np.abs(Δq_linear / qdot_max))
    linear_trajectory = LinearTrajectory(
        q_linear_start, q_linear_end, duration_linear)

    return SmoothLinearTrajectory(quintic1, quintic2, linear_trajectory)


class QuinticSplineTraj(Trajectory):
    
    def __init__(self, ts, configurations) -> None:

        # set boundary velocities, accelerations to 0 
        # (note that start and end configurations are then not given by data) (?)
        N = len(configurations[0])
        _bc_derivatives_zero = [(1, np.zeros(N)), (2, np.zeros(N))]
        bc_type = (_bc_derivatives_zero, _bc_derivatives_zero)

        configurations = np.array(configurations)

        self.bspline = scipy.interpolate.make_interp_spline(ts, configurations, k = 5, bc_type=bc_type, axis = 0)

        self.duration = ts[-1]

    def position(self, Δt):
        return self.bspline(Δt)
    


def construct_needle_trajectory(x_start, x_end, N_samples, model = None):
    
    Δx = x_end - x_start
    qdot_max = PANDA_QDOTMAX * 0.01

    points = [x_start + alpha * Δx for alpha in np.linspace(0, 1, N_samples)]

    iikinematics = IIKinematics(model)
    
    _orientation_matrix = orientation_matrix(Δx)
    configurations = []
    for (i, point) in enumerate(points):
        start_config = 0.5 * (PANDA_QMIN + PANDA_QMAX) if i == 0 else configurations[-1]
        configurations.append(iikinematics.matrix_inverse_kinematics(hom_matrix(_orientation_matrix, point), start_config))

    q0 = configurations[0]
    qend = configurations[-1]
    duration = np.max(np.abs((qend - q0) / qdot_max))
    ts = np.linspace(0, duration, N_samples)

    return QuinticSplineTraj(ts, configurations)
