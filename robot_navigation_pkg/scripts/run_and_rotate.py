#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  open_test_run.py
#
#
#  This node will simply publist Twist command to husky robot and receive command velocity from main algo node

import rospy,sys
from nav_msgs.msg import Odometry
from numpy import deg2rad, cos, sin, rad2deg
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from numpy import deg2rad, cos, sin
from std_msgs.msg import Float32


try:
	step_value_velocity = rospy.get_param("/step_distance",0.01)
#max_robot_vel = rospy.get_param("/max_robot_velocity",0.6)


#In degrees to rad
	step_value_angle = deg2rad(rospy.get_param("/step_angle",2))
	step_size_angle = deg2rad(rospy.get_param("/max_step_size",50))

#In degrees to rad
#max_robot_angle = deg2rad(rospy.get_param("/max_robot_angle",90))
except:
	rospy.logwarn("Start main simulation first")
	sys.exit(0)


Twist_msg = Twist()

global robot_current_angle
global current_robo_angle

robot_current_angle = 0.0
current_robo_angle = 0.0
#Will update robot current angle

def update_robot_angle(angle_dat):
	global robot_current_angle

	if(angle_dat.data > 0 and angle_dat.data < 180):
		robot_current_angle = angle_dat.data

	if(angle_dat.data < 0 and angle_dat.data > -180):
		robot_current_angle = (180 + angle_dat.data)+180



#	rospy.logwarn("Robot current angle %f",robot_current_angle)


def Do_run_rotate(main_cmnd_velocity):

	global publish_vel
	global move_step
	global Twist_msg

	global step_value_velocity
	global max_robot_vel


	max_robot_vel = main_cmnd_velocity.linear.x

	max_robot_angle = main_cmnd_velocity.angular.z


	temp_step_vel = int(step_value_velocity * 1000)
	temp_max_robot_vel = int(max_robot_vel * 1000)

	temp_step_angle = int(step_value_angle * 1000)
	temp_max_robot_angle = int(max_robot_angle * 1000)

	temp_size_angle = int(step_size_angle * 1000)



	if(max_robot_vel > 0.0 or max_robot_vel < 0.0):

		rospy.set_param("/is_move_complete",False)

		if(max_robot_vel > 0.0):

			for translation in range(0, temp_max_robot_vel,int(temp_step_vel)):

				temp_translation = float(translation) / 1000
				Twist_msg.linear.x = temp_translation
				Twist_msg.angular.z = 0.0

#				rospy.loginfo("Publising forward accelereation to robot")
				publish_vel.publish(Twist_msg)

				rospy.sleep(0.05)

			for translation in range(temp_max_robot_vel,0,int(-temp_step_vel)):
				temp_translation = float(translation) / 1000
				Twist_msg.linear.x = temp_translation
				Twist_msg.angular.z = 0.0

#				rospy.loginfo("Publising forward deccelertion to robot")
		                publish_vel.publish(Twist_msg)

				rospy.sleep(0.05)

			rospy.loginfo("Robot translation completed")


			rospy.sleep(1)

#Move backward
####################################################################################################################################
		if(max_robot_vel < 0.0):

			for translation in range(0, temp_max_robot_vel, int(-temp_step_vel)):
				temp_translation = float(translation) / 1000

				Twist_msg.linear.x = temp_translation
				Twist_msg.angular.z = 0.0
#				rospy.loginfo("Publising backward acceleration to robot")

				publish_vel.publish(Twist_msg)

				rospy.sleep(0.05)


			for translation in range(-temp_max_robot_vel,0,int(temp_step_vel)):

				temp_translation = float(translation) / 1000

				Twist_msg.linear.x = temp_translation
				Twist_msg.angular.z = 0.0

#				rospy.loginfo("Publising backward decceleration to robot")

		                publish_vel.publish(Twist_msg)
				rospy.sleep(0.05)

			rospy.loginfo("Robot translation completed")


			rospy.sleep(1)


	if(max_robot_angle > 0.0 or max_robot_angle < 0.0):


#Rotate clockvice
####################################################################################################################################
		if(max_robot_angle > -180.0 and max_robot_angle < 0.0):


			global current_robo_angle

			current_robo_angle = abs(robot_current_angle)

			final_angle = current_robo_angle  + rad2deg(max_robot_angle)
			if final_angle < 0:
				final_angle = (180 + final_angle) + 180


			rospy.loginfo("Robot current angle = %f , final angle = %f",current_robo_angle,final_angle)

#			temp_diff = final_angle - current_robo_angle

#			rospy.loginfo(temp_diff)

			while (abs(robot_current_angle - final_angle) > 10 ):

					for rotation in range(0, temp_size_angle, int(temp_step_angle)):

						temp_rotation = float(rotation) / 1000

						Twist_msg.linear.x = 0.0
						Twist_msg.angular.z = -temp_rotation

#						rospy.loginfo(robot_current_angle)
						publish_vel.publish(Twist_msg)

						rospy.loginfo("Robot current angle = %f , final angle = %f",robot_current_angle,final_angle)

						if(abs(robot_current_angle - final_angle) < 10):
							break


						rospy.sleep(0.01)






#Rotate anticlockvice
####################################################################################################################################
#			rospy.loginfo("Robot rotation completed clockvice")

#			rospy.loginfo(robot_current_angle)

#			rospy.sleep(1)

#Working good
		if(max_robot_angle > 0.0 and max_robot_angle < 180):

#			global current_robo_angle

			current_robo_angle = abs(robot_current_angle)

			final_angle = current_robo_angle  + rad2deg(max_robot_angle)

			if(final_angle > 360):
				final_angle = final_angle - 360



#			rospy.loginfo("Robot current angle = %f , final angle = %f",current_robo_angle,final_angle)




			while (abs(robot_current_angle - final_angle) > 10):

					for rotation in range(0, temp_size_angle, int(temp_step_angle)):

						temp_rotation = float(rotation) / 1000

						Twist_msg.linear.x = 0.0
						Twist_msg.angular.z = temp_rotation

#						rospy.loginfo(robot_current_angle)
						publish_vel.publish(Twist_msg)

						rospy.loginfo("Robot current angle = %f , final angle = %f, diff = %f",robot_current_angle,final_angle,abs(robot_current_angle-final_angle))
						if(abs(robot_current_angle - final_angle) < 10):
							break


						rospy.sleep(0.01)





	rospy.set_param("/is_move_complete",True)


def Test_run():





	rospy.init_node('Simple_test_run')


	global publish_vel

	publish_vel = rospy.Publisher('/p3at/cmd_vel', Twist)

	#Subscriber for receving Twist message from main algorithm node
	rospy.Subscriber("/main_cmd_velocity", Twist, Do_run_rotate)


	rospy.set_param("/is_move_complete",False)


	rospy.Subscriber("/robot_angle",Float32, update_robot_angle)

	rospy.loginfo("Starting Run and rotate node")

	rospy.spin()



#Move forward
####################################################################################################################################







if __name__ == '__main__':


	Test_run()
