import rnm
import rospy

app = rnm.APP

app.start_controller(mode = 'real')

rate = rospy.Rate(0.5)
rate.sleep()

print(f'Initial Joint states: {app.panda.joint_states}')

# simply moving, recording nothing
app.run_object_recording(record_photos=False, record_pointclouds=False)