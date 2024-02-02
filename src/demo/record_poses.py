import rnm

app = rnm.APP
app.start_controller(mode = 'real')

poses = []
while True:
    inp = input("Press enter to record pose, exit to exit")
    if inp == 'exit':
        break
    poses.append(app.panda.joint_states)

print(f"Num poses: {len(poses)}")
rnm.write_poses_txt(app.file_custom_joints, poses)

# interpolate poses 