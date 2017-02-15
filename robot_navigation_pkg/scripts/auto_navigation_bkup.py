#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  auto_navigation.py
#
#  This node will execute the algorithm for movement of robot and doing all operations for mine detection
#
#The mine are positioned at random, so we can only build an algorithm not a sequence for this robot. This algorithm moves robot in random postion, set mine while traveling
#and avoid mines by moving away from it and avoiding explosion.


#Assumption
#1)Assume that robot in random position with a random pose
#2)Assume that we can detect the limit of area by GPS coordinates
#3) Rotation must be must also translation must be smooth

#Algorithm

#1)Sweep at first, move a little, sweep, Rotate {check_mine(), set_mine()}
#2)After one rotation, again check_mine on the current path
#3)If there is mine detected, stop, move_little backward, rotate a random angle(as rosparam set) say 90 and move one step, rototae
#4){Check_mine, set_mine} after each step, if there is mine do 3
#5)Constantly check robot is in limits, if it goes above limit,come inside and if it reached the threshold make an angle and go away
#6) Repeat the procedure


import rospy,sys,signal
from nav_msgs.msg import Odometry
from numpy import deg2rad, cos, sin
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped
from numpy import deg2rad, cos, sin

from std_msgs.msg import Bool,String, Float32

global robot_step
global robot_full_rotation


#global is_robot_in
global robot_obstacle
global is_mine_detect


is_mine_detect = False
robot_current_angle = 0.0
try:
	robot_step = rospy.get_param("/robot_main_steps",0.2)
	robot_full_rotation = rospy.get_param("/robot_main_angle",deg2rad(90))

except:
	rospy.logwarn("Start main simulation first")
	sys.exit(0)

robot_main_speed=Twist()






def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    rospy.signal_shutdown("Keyboard interrupt")
    sys.exit(0)



#Set a param for some time if mine detect
def set_mine_status(is_mine):

	try:
		is_mine_detect = is_mine.data
	#	rospy.loginfo("Mine set callback begin")
		rospy.set_param("/mine_is_set",True)
		rospy.sleep(rospy.get_param("/mine_set_time",1))
		rospy.set_param("/mine_is_set",False)
	#	rospy.loginfo("Mine not set end")

	except:
#		rospy.logwarn("Exception in set mine status function for a particular time")
		pass
#Set a param if limit reached

def set_area_limit_status(plot_side):

	try:

		global is_side_hit

		is_side_hit = plot_side.data

#		rospy.logwarn("Data from callback")
#		rospy.logwarn(is_side_hit)

#		rospy.loginfo("Side hit callback begin")
		rospy.set_param("/side_hit",True)
		rospy.sleep(rospy.get_param("/side_hit_time",0.1))
		rospy.set_param("/side_hit",False)
#		rospy.loginfo("Side hit callback end")

		is_side_hit = "n"

	except:
#		rospy.logwarn("Exception in set_area_limit_status for a particular time")
		is_side_hit = "n"

		pass

########################################################################################################################

def set_robot_angle(angle):

	try:

		global robot_current_angle
		robot_current_angle = angle.data

	except:
#		rospy.logwarn("Exception in set robot angle")
		pass


########################################################################################################################

def set_robot_in_out(robot_in):
	try:

		global is_robot_in
		is_robot_in = robot_in.data

	except:
#		rospy.logwarn("Exception in set robot angle")
		pass


def Detect_Side():

	global is_side_hit

	if(is_side_hit == 'l'):
		return (True,"Left")
	if(is_side_hit == 'r'):
		return (True,"Right")
	if(is_side_hit == 'b'):
		return (True,"Bottom")
	if(is_side_hit == 't'):
		return (True,"Top")

	else:
		return (False,'n')

	return(False,'n')

####################################################################################################################################################

def Detect_Mine():

	try:

		if(rospy.get_param("/mine_is_set") == True):
	#		rospy.loginfo("Mine detected")
			return True
		if(rospy.get_param("/mine_is_set") == False):
	#		rospy.loginfo("Mine not detected")
			return False

	except:
#		rospy.logwarn("Exception in Detect_Mine function")
		pass
####################################################################################################################################################
def Detect_Hit():


	try:


		if(rospy.get_param("/side_hit") == True):
	#		rospy.loginfo("Side hit detected")
			return(Detect_Side())

		if(rospy.get_param("/side_hit") == False):
	#		rospy.loginfo("Side hit not detected")
			return(False,'n')
	except:
#		rospy.logwarn("Exception in Detect_Hit function")
		return(False,'n')
		pass


####################################################################################################################################################
#def Get_the_robot_li


#Function will take care of task associated with side hit and avoidace
def Avoid_Side_Hit(robot_angle,temp_is_hit,hit_side):

	try:

		if(temp_is_hit == True):
			side_of_hit = hit_side
			current_robo_angle = robot_angle

			rospy.loginfo("Robot_angle = %f, Is_hit=%r , Hit_Side=%s",current_robo_angle,temp_is_hit,side_of_hit)


########################################################################################################################################################################

			if(side_of_hit == "Right"):


				#1-Condition when robot approaches on right side with the arm on front perpendicular
				if((current_robo_angle > 0 and current_robo_angle < 30) or (current_robo_angle < 0 and current_robo_angle > -30)):
					rospy.loginfo("#1-Robot hit in right with almost perpendicular with side with arm on the side")



					#Move backward
					move_robot(-0.4)
					#Rotate a to 120 deg
					rotate_robot(150)

					move_robot(0.2)




				#2-Condition when robot approaches on right side with arm in front but not perpendicular
				if((current_robo_angle > 30 and current_robo_angle < 60) or (current_robo_angle > -60 and current_robo_angle < -30)):
					rospy.loginfo("#2-Robot hit in right side with arm in front but  not perpendicular")



					if(current_robo_angle > 0):
					#Move backward
					#Rotate a to 120 deg
						rotate_robot(current_robo_angle+20)
						move_robot(0.4)


					if(current_robo_angle < 0):
					#Move backward
					#Rotate a to 120 deg
						rotate_robot(-(current_robo_angle+20))
						move_robot(0.4)




				#3-Robot almost parallel to the right side going down and try to rotate clock or anticlockvice
				if(current_robo_angle < -60 and current_robo_angle > -120):
					rospy.loginfo("#3-Robot almost parallel to the right side going down and try to rotate clock or anticlockvice")


					rotate_robot(-60)
					move_robot(0.4)






				#4-Robot almost parallel to the right side going up and try to rotate clock or anticlockvice
				if(current_robo_angle > 60 and current_robo_angle < 120):
					rospy.loginfo("#4-Robot almost parallel to the right side going up and try to rotate clock or anticlockvice")




					rotate_robot(60)
					move_robot(0.4)





				#5-Condition when robot approaches with back on right side with arm not facing on right side
				if((current_robo_angle > 120 and current_robo_angle < 150) or (current_robo_angle > -150 and current_robo_angle < -120)):
					rospy.loginfo("#5-Condition when robot approaches with back on right side with arm not facing on right side")


					rotate_robot(30)
					move_robot(0.4)






				#6-Robot hit by backside with almost perpendicular to right side
				if((current_robo_angle > 150 and current_robo_angle < 180) or (current_robo_angle < -150 and current_robo_angle > -180)):
					rospy.loginfo("Robot hit in right with almost half perpendiculr to the side")



					move_robot(0.4)



########################################################################################################################################################################

			if(side_of_hit == "Left"):

				#1-Condition when robot approaches on left side with the arm on front perpendicular
				if((current_robo_angle > 150 and current_robo_angle < 180) or (current_robo_angle < -150 and current_robo_angle > -180)):
					rospy.loginfo("#1-Robot hit in left with almost perpendicular with side with arm on the side")


					#Move backward
					move_robot(-0.4)
					#Rotate a to 120 deg
					rotate_robot(150)

					move_robot(0.2)





				#2-Condition when robot approaches on left side with arm in front but not perpendicular
				if((current_robo_angle > -150 and current_robo_angle < -120) or (current_robo_angle > 120 and current_robo_angle < 150)):
					rospy.loginfo("#2-Robot hit in left side with arm in front but  not perpendicular")




					if(current_robo_angle > 0):
					#Move backward
					#Rotate a to 120 deg
						rotate_robot(-60)
						move_robot(0.4)


					if(current_robo_angle < 0):
					#Move backward
					#Rotate a to 120 deg
						rotate_robot(60)
						move_robot(0.4)


				#3-Robot almost parallel to the left side going down and try to rotate clock or anticlockvice
				if(current_robo_angle >  -120 and current_robo_angle < -60):
					rospy.loginfo("#3-Robot almost parallel to the left side going down and try to rotate clock or anticlockvice")





					rotate_robot(60)
					move_robot(0.4)




				#4-Robot almost parallel to the left side going up and try to rotate clock or anticlockvice
				if(current_robo_angle > 60 and current_robo_angle < 120):
					rospy.loginfo("#4-Robot almost parallel to the left side going up and try to rotate clock or anticlockvice")



					rotate_robot(-60)
					move_robot(0.4)





				#5-Condition when robot approaches with back on left side with arm not facing on right side
				if((current_robo_angle > 30 and current_robo_angle < 60) or (current_robo_angle > -60 and current_robo_angle < -30)):
					rospy.loginfo("#5-Condition when robot approaches with back on left side with arm not facing on right side")

					rotate_robot(30)
					move_robot(0.4)





				#6-Robot hit by backside with almost perpendicular to left side
				if((current_robo_angle > 0 and current_robo_angle < 30) or (current_robo_angle < 0 and current_robo_angle > -30)):
					rospy.loginfo("Robot hit in left with almost half perpendiculr to the side")



					move_robot(0.4)




########################################################################################################################################################################

			if(side_of_hit == "Top"):

				#1-Condition when robot approaches on top side with the arm on front perpendicular
				if((current_robo_angle > 90 and current_robo_angle < 120) or (current_robo_angle < 90  and current_robo_angle > 60)):
					rospy.loginfo("#1-Robot hit in top with almost perpendicular with side with arm on the side")




					#Move backward
					move_robot(-0.4)
					#Rotate a to 120 deg
					rotate_robot(-100)

					move_robot(0.2)




				#2-Condition when robot approaches on top side with arm in front but not perpendicular
				if((current_robo_angle > 120 and current_robo_angle < 150) or (current_robo_angle > 30 and current_robo_angle < 60)):
					rospy.loginfo("#2-Robot hit in top side with arm in front but  not perpendicular")




					if(current_robo_angle > 120 and current_robo_angle < 150):
					#Move backward
					#Rotate a to 120 deg
						move_robot(-0.4)
						rotate_robot(60)


					if(current_robo_angle > 30 and current_robo_angle < 60):
					#Move backward
					#Rotate a to 120 deg
						move_robot(0.4)
						rotate_robot(60)





				#3-Robot almost parallel to the top side going down and try to rotate clock or anticlockvice
				if((current_robo_angle < -150 and current_robo_angle > -180) or (current_robo_angle < 180 and current_robo_angle > 150)):
					rospy.loginfo("#3-Robot almost parallel to the top side going down and try to rotate clock or anticlockvice")



					rotate_robot(60)
					move_robot(0.4)







				#4-Robot almost parallel to the top side going up and try to rotate clock or anticlockvice
				if((current_robo_angle > -30 and current_robo_angle < 0) or (current_robo_angle > 0 and current_robo_angle < 30)):
					rospy.loginfo("#4-Robot almost parallel to the top side going up and try to rotate clock or anticlockvice")



					rotate_robot(-60)
					move_robot(0.4)






				#5-Condition when robot approaches with back on top side with arm not facing on right side
				if((current_robo_angle > -150 and current_robo_angle < -120) or (current_robo_angle > -60 and current_robo_angle < -30)):
					rospy.loginfo("#5-Condition when robot approaches with back on top side with arm not facing on right side")



					rotate_robot(30)
					move_robot(0.4)




				#6-Robot hit by backside with almost perpendicular to top side
				if((current_robo_angle > -120 and current_robo_angle < -90) or (current_robo_angle < -60 and current_robo_angle > -90)):
					rospy.loginfo("Robot hit in top with almost half perpendiculr to the side")


					move_robot(0.4)




########################################################################################################################################################################


			if(side_of_hit == "Bottom"):


				#1-Condition when robot approaches on bottom side with the arm on front perpendicular
				if((current_robo_angle > -90 and current_robo_angle < -60) or (current_robo_angle < -90  and current_robo_angle > -120)):
					rospy.loginfo("#1-Robot hit in bottom with almost perpendicular with side with arm on the side")



					#Move backward
					move_robot(-0.4)
					#Rotate a to 120 deg
					rotate_robot(-100)

					move_robot(0.2)





				#2-Condition when robot approaches on bottom side with arm in front but not perpendicular
				if((current_robo_angle > -60 and current_robo_angle < -30) or (current_robo_angle > -150 and current_robo_angle < -120)):
					rospy.loginfo("#2-Robot hit in bottom side with arm in front but  not perpendicular")



					if(current_robo_angle > -60 and current_robo_angle < -30):
					#Move backward
					#Rotate a to 120 deg
						rotate_robot(60)

						move_robot(0.4)


					if(current_robo_angle > -150 and current_robo_angle < -120):
					#Move backward
					#Rotate a to 120 deg
						rotate_robot(-60)


						move_robot(0.4)




				#3-Robot almost parallel to the bottom side going down and try to rotate clock or anticlockvice
				if((current_robo_angle < 30 and current_robo_angle > 0) or (current_robo_angle < 0 and current_robo_angle > -30)):
					rospy.loginfo("#3-Robot almost parallel to the bottom side going down and try to rotate clock or anticlockvice")



					rotate_robot(60)
					move_robot(0.4)




				#4-Robot almost parallel to the bottom side going up and try to rotate clock or anticlockvice
				if((current_robo_angle < -150 and current_robo_angle > -180) or (current_robo_angle < 180 and current_robo_angle > 150)):
					rospy.loginfo("#4-Robot almost parallel to the bottom side going up and try to rotate clock or anticlockvice")



					rotate_robot(60)
					move_robot(0.4)





				#5-Condition when robot approaches with back on bottom side with arm not facing on right side
				if((current_robo_angle > 30 and current_robo_angle < 60) or (current_robo_angle > 120 and current_robo_angle < 150)):
					rospy.loginfo("#5-Condition when robot approaches with back on bottom side with arm not facing on right side")



					rotate_robot(30)
					move_robot(0.4)




				#6-Robot hit by backside with almost perpendicular to bottom side
				if((current_robo_angle > 60 and current_robo_angle < 90) or (current_robo_angle < 120 and current_robo_angle > 90)):
					rospy.loginfo("Robot hit in bottom with almost half perpendiculr to the side")



					move_robot(0.4)


########################################################################################################################################################################
	except:
#		rospy.logwarn("Exception in Avoid side hit")
		pass



#This function will move the robot and automatically set mine
def Move_Set_Mine(mine_set_or_not):

	if(mine_set_or_not != True):
		move_robot(0.22)
	elif(mine_set_or_not == True):
		move_robot(-0.22)
		rotate_robot(80)

def set_robot_obstacle():
	global robot_obstacle
	try:
		robot_obstacle = rospy.get_param("/is_obstacle")
		rospy.sleep(0.01)

	except:
#		rospy.logwarn("Exception in set robot angle")
		pass


def Avoid_Obstacle():
	rospy.logwarn("Obstacle found, moving away from it")

	move_robot(-0.22)
	rotate_robot(80)


def Goto_Arena():
	rospy.logwarn("Robot is outside, trying to enter to arena")
	try:
		temp_rotate_angle = rospy.get_param("/robot_out_turn_angle")
		rospy.logwarn("Required robot turn angle")


#		rotate_angle_new = float(robot_current_angle - temp_rotate_angle)

		#rotate_robot(11)

		#rospy.sleep(1)


		temp_rotate_angle = rospy.get_param("/robot_out_turn_angle")

		rotate_robot(temp_rotate_angle)
                move_robot(-0.22)
		rospy.logwarn("Rotating angle")
		rospy.logwarn(temp_rotate_angle)

#		if(rotate_angle_new < 0):

#			rotate_robot(rotate_angle_new)
#			move_robot(0.22)

#		if(rotate_angle_new > 0):

#			rotate_robot(-rotate_angle_new)
#			move_robot(0.22)

		rospy.loginfo("Robot is outside arena, trying to come inside")

		set_robot_in_out()
#		while (is_robot_in != False):
		while True:

#			rospy.logwarn(temp_rotate_angle)
#			rotate_robot(11)
			rospy.logerr("Robot is outside, trying to move inside")
			set_robot_in_out()
			move_robot(0.22)
			rospy.sleep(0.1)
			if(is_robot_in == True):
				break



	except:
		rospy.loginfo("Exception while robot reached to enter the area")
		pass


#Main loop algorithm
def Move_Loop():


	is_mine_set = False
	is_hit = False
	hit_object = []

        #global is_robot_in
	global robot_obstacle
	while not rospy.is_shutdown():


		try:
                        #import pdb;pdb.set_trace()
			#set_robot_in_out()

			set_robot_obstacle()


			#rospy.loginfo("Robot is in or out")
			#rospy.loginfo(is_robot_in)


                        #if(is_robot_in == False):
			#	Goto_Arena()


#			rospy.logerr("You shit, quit the loop")

			if(robot_obstacle == True):
				Avoid_Obstacle()
			if(Detect_Mine() == True):
	#			rospy.loginfo("Mine detected")
				is_mine_set = True
			elif(Detect_Mine() == False):
	#			rospy.loginfo("Mine not detected")
				is_mine_set = False




			hit_object = Detect_Hit()


			if(hit_object[0] == True):
       # 			rospy.loginfo("Side hit detected")
				is_hit = True
				temp_hit_side = hit_object[1]

			elif(hit_object[0] == False):
        #			rospy.loginfo("Side hit not detected")
				is_hit = False
				temp_hit_side = hit_object[1]




			#Check if there are any side collsion
                        Avoid_Side_Hit(robot_current_angle,is_hit,temp_hit_side)



			#Check mine and move robot
			Move_Set_Mine(is_mine_set)




####################################################################################################################################################

		except:
			rospy.logwarn("Exception found on main loop")
			pass

		rospy.sleep(0.04)







####################################################################################################################################################

def move_robot(speed):

	try:



		global robot_main_speed
	#	print robot_step
		global main_cmd_vel


		robot_step = speed

		rospy.sleep(0.5)

		robot_main_speed.linear.x = robot_step
		robot_main_speed.angular.z = 0.0
		rospy.loginfo("Executing translation")
		main_cmd_vel.publish(robot_main_speed)
		Wait_for_completion()

	except:
#		rospy.logwarn("Exception in move robot function")
		pass
####################################################################################################################################################

def rotate_robot(angle):

	try:



		global robot_main_speed
		global robot_full_rotation
		global main_cmd_vel

		rospy.sleep(0.2)

		robot_full_rotation = deg2rad(angle)

		robot_main_speed.linear.x = 0.0
		robot_main_speed.angular.z = robot_full_rotation
		rospy.loginfo("Executing rotation")
		main_cmd_vel.publish(robot_main_speed)

		Wait_for_completion()
	except:
#		rospy.logwarn("Exception in rotate robot function")
		pass

def Wait_for_completion():

	try:

		while(rospy.get_param("/is_move_complete") != True):
#			rospy.loginfo("Waiting for completion")
			rospy.sleep(0.4)


		#Setting false to start new movement
		rospy.set_param("/is_move_complete",False)

	except:
#		rospy.logwarn("Exception in Waiting for completion")
		rospy.set_param("/is_move_complete",False)
		pass
#Initialization
def Init_Move():



#	global robot_step
#	global robot_full_rotation

#	while(rospy.get_param("/is_move_complete") != True):
#		rospy.loginfo("Waiting for completion")
#		rospy.sleep(0.2)


#	move_robot(0.2)
#	move_robot(0.2)

#	rospy.loginfo("Completed step, now rotation")


	rotate_robot(-38.5)
        move_robot(0.40)
#	rospy.loginfo("Completed rotation")


	#Move robot around mine area
	Move_Loop()

def Run_auto_mode():


	rospy.init_node('Main_auto_node')

	rospy.loginfo('Started autonomous mode')
	global main_cmd_vel


	main_cmd_vel = rospy.Publisher("/main_cmd_velocity", Twist)
	rospy.set_param("/is_move_complete",False)
	rospy.set_param("/mine_is_set",False)
	rospy.set_param("/side_hit",False)

	rospy.Publisher('/robot_in_out', Bool, set_robot_in_out)

	rospy.Subscriber("/is_detect_mine",Bool, set_mine_status)
	rospy.Subscriber("/reached_one_side_limit",String, set_area_limit_status)

	rospy.Subscriber("/robot_angle",Float32, set_robot_angle)





	signal.signal(signal.SIGINT, signal_handler)

	#The initial move performed by robot when start
	Init_Move()

	rospy.spin()



if __name__ == '__main__':
	#This function will move robot in autonomous mode
	Run_auto_mode()

