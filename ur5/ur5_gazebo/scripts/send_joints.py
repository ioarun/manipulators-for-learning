#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import csv
from decimal import Decimal

trajectory_file = "trajectory_file.csv"
file = open(trajectory_file, 'rb')
reader = csv.reader(file)


def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/trajectory_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    reader.next()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        # for row in reader:
        #     shoulder_pan_joint = (row[0])
        #     shoulder_lift_joint = (row[1])
        #     elbow_joint = (row[2])
        #     wrist_1_joint = (row[3])
        #     wrist_2_joint = (row[4])
        #     wrist_3_joint = (row[5])
        #     pts.positions = [shoulder_pan_joint, shoulder_lift_joint, \
        #         elbow_joint, wrist_1_joint, wrist_2_joint, \
        #         wrist_3_joint]

        #     pts.time_from_start = rospy.Duration(0.02)

        #     # Set the points to the trajectory
        #     traj.points = []
        #     traj.points.append(pts)
        #     # Publish the message
        #     pub.publish(traj)

        pts.positions = [1.2778283886, -0.6807085686, -0.2895342907, -0.5970439271, -0.2896611142, 8.45356745875492E-05]


        pts.time_from_start = rospy.Duration(0.02)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)

    


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
