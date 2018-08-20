# python script to control the ur5
# get_mode_state service; set_model_state service
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

import threading

rospy.init_node('ur5_control_script')
rate = rospy.Rate(50)
reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
reset_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
# reset_cube_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)


# class Publisher(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self)
#         pass      

#     def run(self):
#         print "hello"


class MoveItCartesianPath:
    def __init__(self):
        # rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        # rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

    def go_home(self):

    	waypoints = self.set_target([0.2, 0.0, 0.5])

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints

        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)


        # plan = self.arm.plan()

        # If we have a complete plan, execute the trajectory
        if 1-fraction < 0.2:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed")


    def set_target(self, target):
    	# Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters
        # wpose.position.x = 0.2
        # wpose.position.y = 0.0
        # wpose.position.z = 0.5

        wpose.position.x = target[0]
        wpose.position.y = target[1]
        wpose.position.z = target[2]

        waypoints.append(deepcopy(wpose))
        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
            +(wpose.position.x-start_pose.position.x)**2)<0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        return waypoints

    def move_arm(self, waypoints):

        # self.arm.set_pose_target(wpose)
    	
        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints

        """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(	 
                self, waypoints, eef_step, jump_threshold, avoid_collisios= True)
    
           Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the 
           poses specified as waypoints. Configurations are computed for every eef_step meters; 
           The jump_threshold specifies the maximum distance in configuration space between consecutive points 
           in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed, 
           the actual RobotTrajectory. 

        """
        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)


        # plan = self.arm.plan()

        # If we have a complete plan, execute the trajectory
        if 1-fraction < 0.2:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed")

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

class UR5():
	"""docstring for ClassName"""
	def __init__(self):
		pass

	def reset(self):
		print "reset"

		rospy.wait_for_service('/gazebo/reset_world')
		try:
			reset_world()
		except(rospy.ServiceException) as e:
			print "reset_world failed!"
	    
		rospy.wait_for_service('/gazebo/reset_simulation')
		try:
		    reset_simulation()
		except(rospy.ServiceException) as e:
		    print "reset_simulation failed!"
		
		reset_cube = ModelState()
		reset_cube.model_name = "cube2"
		reset_cube.reference_frame = "world"
		reset_cube.pose.position.x = 0
		reset_cube.pose.position.y = 0
		reset_cube.pose.position.z = 0
		reset_cube.pose.orientation.x = 0
		reset_cube.pose.orientation.y = 0
		reset_cube.pose.orientation.z = 0
		reset_cube.twist.linear.x = 0
		reset_cube.twist.linear.y = 0
		reset_cube.twist.linear.z = 0

		set_model = SetModelState()

		set_model.request.model_state = reset_cube


		# rospy.wait_for_service("cube2", [0, 0, 0],[0, 0, 0],  )
		# '{model_state: { model_name: cube2, pose: { position: { x: 0, y: 0 ,z: 1 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'

		rospy.wait_for_service('/gazebo/set_model_configuration')

		try:
		    reset_joints("robot", "robot_description", ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], [1.0,-1.8,0.0, 0.0, 0.0, 0.0])

		except (rospy.ServiceException) as e:
		    print "reset_joints failed!"

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
		    pause()
		except (rospy.ServiceException) as e:
		    print "rospause failed!"

		rospy.wait_for_service('/gazebo/unpause_physics')

		try:
		    unpause()
		except (rospy.ServiceException) as e:
		    print "/gazebo/pause_physics service call failed"

		print "reset done."

	def randomize_goal(self):
		pass
# rospy.Subscriber("/joint_states", JointState, callbackJointStates)

env = UR5()
moveit = MoveItCartesianPath()
waypoints = moveit.set_target([0.7, 0.0, 0.0])

# while not rospy.is_shutdown():
# 	env.reset()
# 	time.sleep(5)

# thread = Publisher()

# # Start new Thread
# thread.start()

# while not rospy.is_shutdown():
try:
    moveit.move_arm(waypoints)
    moveit.go_home()

except KeyboardInterrupt:
    print "Shutting down MoveItCartesianPath node."
