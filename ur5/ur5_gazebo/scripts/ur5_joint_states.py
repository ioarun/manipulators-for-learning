import sys
import numpy as np
import roslib
import rospy
import rostopic
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import JointState
import time
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
import moveit_msgs.msg

from std_msgs.msg import Header

from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint

import csv

rospy.init_node('ur5_joint_states')


trajectory_file = "trajectory_file.csv"
file = open(trajectory_file, 'wt')
writer = csv.writer(file)
writer.writerow(['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', \
	'wrist_1_joint', 'wrist_2_joint','wrist_3_joint'])


def callbackJointStates(data):
	#  [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
	writer.writerow([float(data.position[0]), float(data.position[1]), float(data.position[2]), float(data.position[3]), \
		float(data.position[4]), float(data.position[5])])

rospy.Subscriber("/joint_states", JointState, callbackJointStates)


while not rospy.is_shutdown():
	pass

file.flush()

