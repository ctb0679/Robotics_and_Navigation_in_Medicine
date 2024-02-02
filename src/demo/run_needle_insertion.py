import rnm
import rospy

app = rnm.APP



joints_file = app.folder_rnm_data.joinpath('custom_poses_needle_trajectory.txt')
joint_positions = rnm.load_joints_txt(joints_file)

app.start_controller(mode = 'real')

qstart, qend = joint_positions

model = rnm.PandaModel()
model.set_joint_angles(qstart)
xstart = model.endeffector_position()[0:3]

model.set_joint_angles(qend)
xend = model.endeffector_position()[0:3]


# use even farther out needle starting position to ensure not colliding with phantom
needle_trajectory = rnm.construct_needle_trajectory(xstart, xend, 100)
to_start_trajectory = rnm.smooth_linear(app.panda.joint_states, needle_trajectory.position(0))

print('Start insertion')

app.panda.trajectory_queue += [to_start_trajectory, needle_trajectory]
app.panda.run_controller(-1)
print('Insertion ended')
