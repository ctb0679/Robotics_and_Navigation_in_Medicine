# python robot controller node

import rospy
import numpy as np

from sensor_msgs import msg
from sensor_msgs.msg import JointState, Image, PointCloud2

import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray  # for position messages

from rnm.panda_model import *
from rnm.trajectories import *
from rnm.inverse_kinematics import *

import cv2
import cv_bridge
BRIDGE = cv_bridge.CvBridge()


REAL_COMMAND_TOPIC = '/joint_position_example_controller/joint_command'
SIM_COMMAND_TOPIC = '/joint_position_example_controller_sim/joint_command'
JOINT_TOPIC = '/joint_states'
DESIRED_JOINT_STATES = '/franka_state_controller/joint_states_desired' 
RGB_RAW_TOPIC = '/k4a/rgb/image_raw'  # type sensor_msgs.msg.Image
IR_RAW_TOPIC = '/k4a/ir/image_raw'
DEPTH_RAW_TOPIC = '/k4a/depth/image_raw' 
POINTCLOUD_TOPIC = '/k4a/points2' # type sensor_msgs.msg.PointCloud2
CLICKED_POINT_TOPIC = '/clicked_point' # type geometry_msgs.msg.PointStamped


class PandaController:
    """
    The panda controller, controlling state of movement and movement itself.

    Stores joint angles, allows to send joint angles to the robot 
    and updates the current joint angles from a subscriber.

    Additionally allows to listen for other topics, such as `/clicked_point`.
    Allows to run through a trajectory.
    """

    # Fields:
    # joint_states : ndarray # 7 elements, current read joint angles
    # joint_velocities : ndarray
    # joint_subscriber # the subscriber instance subscribing to joint_states
    # joint_publisher # publisher publishing joint commands on command_topic
    # current_trajectory: Trajectory # current trajectory being executed
    # trajectory_queue # trajectories scheduled for execution
    # on_loop_callbacks # list of functions to be executed on every loop iteration
    # duration # duration of the current controller loop, after which the loop terminates
    # start_time # rostime of when the controller was started

    def __init__(self, on_loop_callbacks=[]) -> None:
        self.model = PandaModel()
        self.invkinematics = IIKinematics()
        self.joint_states = None #np.zeros(7)
        self.joint_velocities = np.zeros(7)
        self.current_trajectory = None
        self.trajectory_queue = []
        self.on_loop_callbacks = on_loop_callbacks
        self.duration = -1.0
        self.start_time = -1.0

    def init(self, name='panda_controller', mode = 'sim'):
        """
            Initialize a ros node with name `name`, initializing the given mode, which is one of:
            - 'sim' simulation, initializing joint states 
            - 'rosbag' simulation
            - 'real' , initializing the non-simulation controllers
        """
        rospy.init_node(name, anonymous=False)
        joints_topic = None
        command_topic = None
        if not mode in {'sim', 'rosbag', 'real'}:
            raise ValueError(f'mode {mode} is not a valid mode')
        
        self.mode = mode
        
        if mode == 'sim':
            command_topic = SIM_COMMAND_TOPIC
            joints_topic = JOINT_TOPIC
        elif mode == 'rosbag':
            command_topic = SIM_COMMAND_TOPIC # publishing without any node subscribing
            joints_topic = JOINT_TOPIC
            self.joint_states = np.zeros_like(PANDA_QMIN)
        elif mode == 'real':
            command_topic = REAL_COMMAND_TOPIC
            joints_topic = DESIRED_JOINT_STATES


        if not joints_topic is None:
            self.joint_subscriber = rospy.Subscriber(
                joints_topic, msg.JointState, self.joint_states_callback)
        if not command_topic is None:
            self.joint_publisher = rospy.Publisher(
                command_topic, Float64MultiArray, queue_size=1)
            

    def joint_states_callback(self, _msg: JointState):
        self.joint_states = np.array(_msg.position)
        self.joint_velocities = np.array(_msg.velocity)

    def inverse_kinematics(self, target_matrix, start_configuration=None):
        """
        Invoke the inverse kinematics provider to return a suitable configuration
        """
        if start_configuration is None:
            start_configuration = 0.5*(PANDA_QMIN + PANDA_QMAX)
        config = self.invkinematics.matrix_inverse_kinematics(target_matrix)
        return config

    def get_world_coordinates(self, msg_: geometry_msgs.msg.PointStamped):
        """
        Calculate the world coordinates of the point given by `msg_` in the respective frame
        given by its header
        """
        # todo
        raise NotImplementedError()

    def _publish_pose(self, configuration):
        """configuration should be a float list or array"""
        _msg = Float64MultiArray()
        _msg.data = configuration
        self.joint_publisher.publish(_msg)

    def _exec_loop(self):
        """
        Run one loop iteration of the controller node, i.e.
        - run all loop callbacks
        - start the next trajectory if there is no trajectory being executed
        - update the joint positions interpolated from the trajectory
        """
        for callback in self.on_loop_callbacks:
            callback()

        if self.current_trajectory is None and len(self.trajectory_queue) >= 1:
            self.current_trajectory = self.trajectory_queue.pop(0)
            self.start_time = rospy.get_time()
            rospy.loginfo(f"Started next trajectory: {self.current_trajectory}")

        if not self.current_trajectory is None:
            curtime = rospy.get_time()
            relative_time = curtime - self.start_time
            if relative_time <= self.current_trajectory.duration:
                configuration = self.current_trajectory.position(relative_time)
                self._publish_pose(configuration)
            else:
                self.current_trajectory = None

    def run_controller(self, duration = -1, rate = None):
        if rate is None:
            rate = rospy.Rate(2000)
        self.start_time = rospy.get_time()
        cur_time = self.start_time
        self.duration = duration
        while (cur_time - self.start_time < self.duration or self.current_trajectory != None or len(self.trajectory_queue) > 0 ) and not rospy.is_shutdown():
            self._exec_loop()
            rate.sleep()
            cur_time = rospy.get_time()

    def exec_trajectory(self, trajectory):
        self.current_trajectory = trajectory
        self.run_controller(trajectory.duration)
        self.current_trajectory = None

    def is_topic_published(self, topic : str):
        topics = rospy.get_published_topics()
        return any((topic == _topic for (_topic,_) in topics))
    
    def take_image(self, rgb = True, timeout = 5):
        topic =  RGB_RAW_TOPIC if rgb else IR_RAW_TOPIC
        
        image = rospy.wait_for_message(topic, Image, timeout=timeout)
        return image
        
    
    def take_image_cv(self, rgb = True, timeout = 5):
        ros_img = self.take_image(rgb, timeout)
        return BRIDGE.imgmsg_to_cv2(ros_img)
        
    
    def get_pointcloud(self, timeout = 2):
        return rospy.wait_for_message(POINTCLOUD_TOPIC, PointCloud2, timeout=timeout)
        
    def is_resting(self):
        if self.mode == 'rosbag':
            return np.linalg.norm(self.joint_velocities) <= 1e-5
        else:
            return self.current_trajectory == None or type(self.current_trajectory) == RestingTrajectory
        