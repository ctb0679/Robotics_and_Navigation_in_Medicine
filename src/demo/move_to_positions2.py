import rnm
import rospy

app = rnm.APP


app.start_controller(mode='real')
controller = app.panda


target_matrices = rnm.load_poses_txt(str(app.file_poses))

configurations = [controller.inverse_kinematics(target_matrix) for target_matrix in target_matrices]

start_config = controller.joint_states
print(f'Start Config: {start_config}')


configurations.insert(0, controller.joint_states)


duration = 25

quintic_trajectories = [rnm.QuinticTrajectory(configurations[i-1], configurations[i], duration) for i in range(1, len(configurations))]


trajectories =  rnm.construct_quintic_takephoto_trajectories(configurations)
#trajectories = rnm.construct_takephoto_trajectories(configurations, rnm.smooth_linear)
# [rnm.RestingTrajectory(start_config, 30)]
controller.trajectory_queue +=  trajectories #quintic_trajectories

rospy.loginfo("Start Move Loop")
controller.run_controller(100)