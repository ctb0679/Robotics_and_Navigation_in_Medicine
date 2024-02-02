#! /usr/bin/env python3
import rnm
import rospy
from sensor_msgs.msg import JointState
import numpy as np

# init panda controller


controller = rnm.PandaController()

controller.init(mode='sim') # init

rospy.wait_for_message("joint_states", JointState)

rospy.loginfo("Joint states: \n %s" % str(controller.joint_states))

start_position = np.array(controller.joint_states)
#end_position = np.full((7,), np.pi/2, dtype=np.float64)


trajectory = rnm.LinearTrajectory(start_position, rnm.PANDA_QMIN, 10)

rospy.loginfo("start moving")
controller.exec_trajectory(trajectory)
rospy.loginfo("move finished")
rospy.loginfo(""" 
Joint States:
%s
""" % str(controller.joint_states))
