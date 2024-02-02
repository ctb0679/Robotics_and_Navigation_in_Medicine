#! /usr/bin/env python3

#import pathlib
#import sys
#sys.path.append(str(pathlib.Path(__file__).absolute().parent.parent))

import rnm
import rospy
import geometry_msgs
import numpy as np

res = rnm.hom_matrix(np.eye(3), np.array([0, 1, 2]))


rospy.loginfo("Started listen_for_point process")
print("Started Python script")

controller = rnm.PandaController("/joint_position_example_controller_sim/joint_command")

controller.init(name = 'listen_for_point')


def on_point_clicked(msg_: geometry_msgs.msg.PointStamped):
    rospy.loginfo("Point clicked")
    # assume point is in link0 reference frame ;
    point = msg_.point

    target_position = np.array((point.x, point.y, point.z))
    # target_position / norm(target_position) # point outwards
    target_orientation = np.eye(3, 3) # z upwards
    target_matrix = rnm.hom_matrix(target_orientation, target_position)

    config = controller.inverse_kinematics(
        target_matrix, start_configuration=controller.joint_states)

    path = rnm.LinearPath(controller.joint_states, config)
    controller.trajectory_queue.append(
        rnm.SimpleTrajectory(path, 5, rnm.sine_interpolation))
    rospy.loginfo("Trajectory added")

clicked_point_subscriber = rospy.Subscriber(
    "/clicked_point", geometry_msgs.msg.PointStamped, on_point_clicked)

rospy.loginfo("Started listening for point")

controller.run_move_loop()
