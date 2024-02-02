from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import rnm

model = rnm.PandaModel()
def position(configuration):
    model.set_joint_angles(configuration)
    return model.endeffector_position()[0:3]


program = 'needle_trajectory' # 'needle_trajectory' # 'plot_interpolation'

if program == 'smooth_linear':
    N = 3
    q_start = np.array([0.0, 0.1, 0.3, 0, 0,0,0])# np.array([-2/4 * i for i in range(0, 7)])
    q_end = np.array([0.1, 1.0, 0.5, 0.1, 0.1, 0.1, 0.1]) #np.array([1/7 * i for i in range(0, 7)])

    # trajectory = rnm.QuinticTrajectory(q_start, q_end, 10)
    trajectory = rnm.smooth_linear(q_start, q_end)

    ts = np.linspace(0, trajectory.duration, 2000)

    qs_low_res = np.zeros((len(ts), N))

    for i in range(0, len(ts)):
        qs_low_res[i] = trajectory.acceleration(ts[i])[0:N]

    #assert sum(abs(trajectory.position(0) - q_start)) <= 1e-5
    #assert sum(abs(trajectory.position(trajectory.duration) - q_end)) <= 1e-5

    plt.figure(figsize=(10, 6))
    plt.plot(ts, qs_low_res)
    plt.legend([f"Joint {i}" for i in range(0, N)])
    plt.xlabel("time [s]")
    plt.ylabel("acceleration [radians/time^2]")
    plt.savefig("smooth_linear_acceleration.png")

    plt.show()
    
elif program == 'plot_interpolation':

    qs_low_res = np.array([0.1, 0.3, 0.2, 1.0])
    ts = np.linspace(0.0, 1.0, len(qs_low_res))

    trajectory = rnm.QuinticSplineTraj(ts, qs_low_res)

    ts2 = np.linspace(0.0, 1.0, 400)
    qs2 = trajectory.position(ts2)

    plt.plot(ts2, qs2)
    plt.show()

elif program == 'needle_trajectory':
    start_config = 0.5*(rnm.PANDA_QMIN + rnm.PANDA_QMAX)
    end_config = 0.4 * rnm.PANDA_QMIN + 0.6 * rnm.PANDA_QMAX

    N_samples = 10

    x_start = position(start_config)[0:3]
    x_end = position(end_config)[0:3]
    xs_target = np.array([x_start + alpha *
                 (x_end - x_start) for alpha in np.linspace(0, 1, N_samples)])

    traj_low_res = rnm.construct_needle_trajectory(x_start, x_end, N_samples)
    traj_high_res = rnm.construct_needle_trajectory(x_start, x_end, 100)

    ts = np.linspace(0, traj_low_res.duration, num = 1000)
    qs_low_res = traj_low_res.position(ts)
    xs_low_res = np.zeros((len(qs_low_res), 3))

    qs_high_res = traj_high_res.position(ts)
    xs_high_res = np.zeros((len(qs_low_res), 3))

    for i in range(0, len(qs_low_res)):
        xs_low_res[i, :] = position(qs_low_res[i])
        xs_high_res[i, :] = position(qs_high_res[i])

    ax = plt.axes(projection='3d')

    ax.plot3D(xs_low_res[:, 0], xs_low_res[:, 1], xs_low_res[:, 2], label = "N = 10 (low resolution)")
    ax.plot3D(xs_high_res[:, 0], xs_high_res[:, 1], xs_high_res[:, 2], label = "N = 100 (high resolution)")
    ax.scatter3D(xs_target[:, 0], xs_target[:, 1], xs_target[:, 2], cmap = 'Greens', label = "Target line points")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend()
    plt.show()

else:
    raise NotImplementedError()
