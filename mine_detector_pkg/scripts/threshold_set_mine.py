#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  threshold_set_mine.py
#
#  This code will subscribe smooth value and do thresholding and setting mine




import rospy,tf
import numpy as np


from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool

#Module for getting transformation and setting mine position
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped

global transListene
global mine_detect_message
global mine_detected
#Mine detection var
mine_detection_threshold_left_val = rospy.get_param("/left_coil_threshold", 0.75)
#mine_detection_threshold_middle_val = rospy.get_param("/middle_coil_threshold",-75000)
mine_detection_threshold_right_val = rospy.get_param("/right_coil_threshold",0.75000)



#Mine detection peak
mine_detection_peak_left_val = rospy.get_param("/left_coil_peak",0.75000)
#mine_detection_peak_middle_val = rospy.get_param("/middle_coil_peak",-75000)
mine_detection_peak_right_val = rospy.get_param("/right_coil_peak",0.75000)



#Mine distance threshold
#Use will not detect  mine near to adjacent, it may useful for reducing multiple detection
mine_distance_threshold_x = rospy.get_param("/mine_distace_threshold_x",0.5)
mine_distance_threshold_y = rospy.get_param("/mine_distace_threshold_x",0.5)

#Variables to handle previous mine position
prev_mine_x = 0.0
prev_mine_y = 0.0

#Current mine
current_mine_x = 0.0
current_mine_y = 0.0

#Leave the first mine
mine_detected_count = 0



#Threshold cut
left_lower_thresh = None
left_upper_thresh = None


#middle_lower_thresh = None
#middle_upper_thresh = None


right_lower_thresh = None
right_upper_thresh = None
#Robot pose detect
transListener = None



def left_threshold(data):
	left_alarm = data.data
	#Calling function for thresholding
	left_threshold_data(left_alarm)

#def middle_threshold(data):
#	middle_alarm = data.data
#	middle_threshold_data(middle_alarm)

def right_threshold(data):
	right_alarm = data.data
	right_threshold_data(right_alarm)

def left_threshold_data(alarm_value):
    if alarm_value>0.75:
		mine_detect_message.publish("Mine detected on Left Coil")
		mine_detected.publish(True)
		#Routine to detect mine position set
		find_mine_pose('l')

def left_threshold_data_bkup(alarm_value):
	#Threshold detection

        #Boolean value to handle two cut off at lower and higher
	global left_lower_thresh
	global left_upper_thresh

	if(alarm_value > mine_detection_peak_left_val):
		left_lower_thresh = False
		left_upper_thresh = False


	if(alarm_value < mine_detection_threshold_left_val):
		left_lower_thresh = True

	if(alarm_value > mine_detection_threshold_left_val):
		left_upper_thresh = True

	if(left_lower_thresh == True and left_upper_thresh == True):

		mine_detect_message.publish("Mine detected on Left Coil")
		mine_detected.publish(True)

#		rospy.loginfo("Mine detected on Left Coil")

		#Routine to detect mine position set
		find_mine_pose('l')

		left_lower_thresh =False
		left_upper_thresh = False



"""def middle_threshold_data(alarm_value):
	#Threshold detection

        #Boolean value to handle two cut off at lower and higher
	global middle_lower_thresh
	global middle_upper_thresh

	if(alarm_value > mine_detection_peak_middle_val):
		middle_lower_thresh = False
		middle_upper_thresh = False

	if(alarm_value < mine_detection_threshold_middle_val):
		middle_lower_thresh = True

	if(alarm_value > mine_detection_threshold_middle_val):
		middle_upper_thresh = True

	if(middle_lower_thresh == True and middle_upper_thresh == True):

		mine_detect_message.publish("Mine detected on Middle Coil")
		mine_detected.publish(True)

#		rospy.loginfo("Mine detected on Middle Coil")

		find_mine_pose('m')

		middle_lower_thresh =False
		middle_upper_thresh = False

"""

def right_threshold_data(alarm_value):
    if alarm_value >0.75:
		mine_detect_message.publish("Mine detected on Right Coil")
		mine_detected.publish(True)
		find_mine_pose('r')

def right_threshold_data_bkup(alarm_value):
	#Threshold detection

        #Boolean value to handle two cut off at lower and higher
	global right_lower_thresh
	global right_upper_thresh

	if(alarm_value > mine_detection_peak_right_val):
		right_lower_thresh = False
		right_upper_thresh = False



	if(alarm_value < mine_detection_threshold_right_val):
		right_lower_thresh = True

	if(alarm_value > mine_detection_threshold_right_val):
		right_upper_thresh = True

	if(right_lower_thresh == True and right_upper_thresh == True):


		mine_detect_message.publish("Mine detected on Right Coil")
		mine_detected.publish(True)


#		rospy.loginfo("Mine detected on Right Coil")

		find_mine_pose('r')

		right_lower_thresh =False
		right_upper_thresh = False
##########################################################################################33

def Mine_Position_Diff(current_pose,prev_pose):
        global mine_detected
	global mine_distance_threshold_x
	global mine_distance_threshold_y

	temp_diff_x = prev_pose[0] - current_pose[0]
	temp_diff_y = prev_pose[1] - current_pose[1]

#	rospy.logwarn("Mine position difference")
#	rospy.loginfo(temp_diff_x)
#
#	rospy.loginfo(temp_diff_y)


	try:



		if(abs(temp_diff_x) > abs(mine_distance_threshold_x) or abs(temp_diff_y) > abs(mine_distance_threshold_y)):
			rospy.set_param("/new_mine",True)

#			rospy.loginfo("New mine detected")

		if(abs(temp_diff_x) < abs(mine_distance_threshold_x) or abs(temp_diff_y) < abs(mine_distance_threshold_y)):
			rospy.set_param("/new_mine",False)
#			rospy.loginfo("New mine not detected")

		if(temp_diff_x ==0 and temp_diff_y == 0):
			rospy.set_param("/new_mine",True)
#			rospy.loginfo("New mine detected")



	except:
#		rospy.logwarn("Exception in setting new mine value")
		pass



def find_mine_pose(coil_name):

        global transListener, pose

	global prev_mine_x,prev_mine_y

	global current_mine_x, current_mine_y

	global mine_detected_count

        minePose = PoseStamped()

#	rospy.loginfo("Retreving mine pose")



        try:
		#For hardware
		'''
		if(coil_name == 'l'):
			(trans, rot) = transListener.lookupTransform('odom', 'left_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('odom', 'left_coil', rospy.Time(0))

		if(coil_name == 'm'):
			(trans, rot) = transListener.lookupTransform('odom', 'middle_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('odom', 'middle_coil', rospy.Time(0))

		if(coil_name == 'r'):
			(trans, rot) = transListener.lookupTransform('odom', 'right_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('odom', 'right_coil', rospy.Time(0))
		'''
		#For simulation

		if(coil_name == 'l'):
			(trans, rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))

		#if(coil_name == 'm'):
		#	(trans, rot) = transListener.lookupTransform('minefield', 'middle_coil', rospy.Time(0))
		#	(trans, rot) = transListener.lookupTransform('minefield', 'middle_coil', rospy.Time(0))

		if(coil_name == 'r'):
			(trans, rot) = transListener.lookupTransform('minefield', 'right_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('minefield', 'right_coil', rospy.Time(0))




        except:
		print "Error"
                return


        cx, cy, cz = trans
        minePose.pose.position.x=cx
        minePose.pose.position.y=cy

	current_mine_x = cx
	current_mine_y = cy


	rospy.loginfo("One Mine position recieved")
	rospy.loginfo(minePose.pose.position.x)
	rospy.loginfo("Checking it is already detected")



#	print "Mine count",mine_detected_count

	if(mine_detected_count < 2):

#		rospy.loginfo('Setting first mines')

#		pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)
#		pubMine.publish(minePose)


		prev_mine_x = current_mine_x
		prev_mine_y = current_mine_y



	try:

		Mine_Position_Diff([current_mine_x,current_mine_y],[prev_mine_x,prev_mine_y])

#		rospy.loginfo("New mine status = %r",rospy.get_param("/new_mine"))




		if(rospy.get_param("/new_mine") == True):

			rospy.loginfo('Setting actual mine')

			mine_detected_count = mine_detected_count + 1

			pubMine  = rospy.Publisher('/HRATC_FW/set_mine', PoseStamped)
			pubMine.publish(minePose)
			rospy.set_param("/no_of_mines",mine_detected_count)
			rospy.loginfo(minePose.pose.position.x)

		#Find mine position difference

		prev_mine_x = current_mine_x
		prev_mine_y = current_mine_y

		rospy.set_param("/no_of_mines",mine_detected_count)

	except:
#		rospy.loginfo("Exception in setting mine")
		pass

#	print cx,cy

def Do_threshold_set_mine():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.

    #Global variable for tf listener
	global transListener
	global mine_detect_message
	global mine_detected


	rospy.init_node('Thresholder_node', anonymous=True)
	#Start listening to robot transform
	transListener = tf.TransformListener()


	#If mine is detected it set output true
	mine_detect_message = rospy.Publisher('/mine_detect_msg',String , queue_size=1)
	#Output detected mine pose
	mine_detected= rospy.Publisher('/is_detect_mine', Bool,queue_size=1)

	#Subcribing smoothened alarm values
	rospy.Subscriber('/left_coil_alarm', Float32, left_threshold)
#	rospy.Subscriber('/middle_coil_smooth', Float32, middle_threshold)
	rospy.Subscriber('/right_coil_alarm', Float32, right_threshold)

	rospy.set_param("/new_mine",True)
	rospy.set_param("/no_of_mines",0)

	rospy.loginfo("Start running threshold for mine detector alarm value")

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':

    #Do threshold and set mine
    Do_threshold_set_mine()
