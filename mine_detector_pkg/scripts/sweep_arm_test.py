#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  sweep_arm.py
#  
#  
#
#  This node will sweep arm until shutdown


import rospy, os, time, signal,sys
#Modules for handling arm commands
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import JointState

def publishArm(armSweepValue, armLiftValue, pubArm):
	msg = JointTrajectory()
	msg.header.stamp = rospy.get_rostime()
	msg.points.append(JointTrajectoryPoint())
	msg.joint_names.append("upper_arm_joint")
	msg.points[0].positions.append(armLiftValue)
	msg.points[0].velocities.append(0.5)
	msg.points[0].accelerations.append(0.5)
	msg.joint_names.append("arm_axel_joint")
	msg.points[0].positions.append(armSweepValue)
	msg.points[0].velocities.append(0.5)
	msg.points[0].accelerations.append(0.5)
	msg.points[0].time_from_start = rospy.Duration.from_sec(0.5)
	pubArm.publish(msg)

#Arm control topics
arm_value_pub = rospy.Publisher('/arm_controller/command', JointTrajectory)
# Arm sweeping range from -0.8 to 0.8
armSweepValue = 0.0

#Lifting arm joint
global armLiftValue
global time_step
global angle_step

armLiftValue = rospy.get_param("/arm_lift_value",0.2)
time_step = rospy.get_param("/arm_time_step",0.2)
angle_step = rospy.get_param("/arm_angle_step",13)

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)

#Continous Loop for arm running
def Main_loop():

	while not rospy.is_shutdown():

		global armLiftValue
		global time_step
		global angle_step


		publishArm(-0.5,0.04, arm_value_pub) 

		time.sleep(2)
		
		publishArm(0.8,0.04, arm_value_pub) 

		time.sleep(2)

		'''
		rospy.logwarn("First point")
		for i in range(0,80,angle_step):

			armSweepValue = float(i)/100

			print "first arm"
			print armSweepValue

			publishArm(armSweepValue,armLiftValue, arm_value_pub) 

		'''
		
if __name__ == '__main__':

	rospy.init_node('Sweep_arm')
	rospy.loginfo('Starting sweeping arm')
	signal.signal(signal.SIGINT, signal_handler)

	try:
		Main_loop()
        except rospy.ROSInterruptException:
                pass


