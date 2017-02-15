#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  find_robot_rotation.py
#  
#  
#
#  This node will monitor the robot orientation in Euler angle

import rospy, os, sys, curses, time, cv2, tf
import numpy as np
from numpy import deg2rad, cos, sin, rad2deg
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32

ekfOdom = Odometry()

def receiveEKFOdom(pose_data):
	global ekfOdom

	try:

		ekfOdom = pose_data
		euler_angle_in_rad = euler_from_quaternion([ekfOdom.pose.pose.orientation.x,ekfOdom.pose.pose.orientation.y,ekfOdom.pose.pose.orientation.z,ekfOdom.pose.pose.orientation.w])
	#	rospy.loginfo(euler_angle_in_rad)
		euler_angle_in_degree = [rad2deg(euler_angle_in_rad[0]),rad2deg(euler_angle_in_rad[1]),rad2deg(euler_angle_in_rad[2])]

		#We are taking only yaw
		rotate_angle = euler_angle_in_degree[2]


#		rospy.loginfo(rotate_angle)

		robot_rotate_angle.publish(rotate_angle)


	except:
		rospy.logwarn("Exception from find robot rotation node")
		pass

def Find_Robot_Rotation():


	global robot_rotate_angle


	rospy.init_node("find_robot_rotation")

	rospy.loginfo('Start finding robot rotation')

	rospy.Subscriber("/robot_pose_ekf/odom", PoseWithCovarianceStamped, receiveEKFOdom)

	robot_rotate_angle = rospy.Publisher("/robot_angle",Float32, queue_size=1)

	rospy.spin()

if __name__ == '__main__':
	#This will find the robot rotation along the z axis in euler angle degree
	Find_Robot_Rotation()

