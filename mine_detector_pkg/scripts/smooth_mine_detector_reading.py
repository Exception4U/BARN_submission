#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  smooth_mine_detector_reading.py
#
#
#  This node will smooth the alarm value reading of each coils




import rospy
#Handle array computation
import numpy as np
from std_msgs.msg import Float32


left_smooth_window_size = rospy.get_param("/left_coil_window",3)
#middle_smooth_window_size = rospy.get_param("/middle_coil_window",2)
right_smooth_window_size = rospy.get_param("/right_coil_window",3)


left_count=0
#middle_count=0
right_count=0



left_temp_array = np.zeros(left_smooth_window_size)
#middle_temp_array = np.zeros(middle_smooth_window_size)
right_temp_array = np.zeros(right_smooth_window_size)



def left_coil_callback(left_alarm_data):

	left_alarm = left_alarm_data.data
	smooth_left_alarm_data(left_alarm)

def smooth_left_alarm_data(left_alarm_value):


	global left_temp_array
	#Variable for data count
	global left_count

	#Window size of data ,can be put in parameter
	global left_smooth_window_size

	left_temp_array[left_count]=left_alarm_value
	left_count=left_count + 1

	if(left_count >= left_smooth_window_size):

		temp_array_avg=np.average(left_temp_array)
		#Publishing smoothened value
		left_coil_smooth.publish(temp_array_avg)

		left_count = 0
		left_temp_array=np.zeros(left_smooth_window_size)




"""
def middle_coil_callback(middle_alarm_data):

	middle_alarm = middle_alarm_data.data
	smooth_middle_alarm_data(middle_alarm)

def smooth_middle_alarm_data(middle_alarm_value):


	global middle_temp_array
	#Variable for data count
	global middle_count

	#Window size of data ,can be put in parameter
	global middle_smooth_window_size

	middle_temp_array[middle_count]=middle_alarm_value
	middle_count=middle_count + 1

	if(middle_count >= middle_smooth_window_size):

		temp_array_avg=np.average(middle_temp_array)
		#Publishing smoothened value
		middle_coil_smooth.publish(temp_array_avg)

		middle_count = 0
		middle_temp_array=np.zeros(middle_smooth_window_size)


"""



def right_coil_callback(right_alarm_data):

	right_alarm = right_alarm_data.data
	smooth_right_alarm_data(right_alarm)

def smooth_right_alarm_data(right_alarm_value):


	global right_temp_array
	#Variable for data count
	global right_count

	#Window size of data ,can be put in parameter
	global right_smooth_window_size

	right_temp_array[right_count]=right_alarm_value
	right_count=right_count + 1

	if(right_count >= right_smooth_window_size):

		temp_array_avg=np.average(right_temp_array)
		#Publishing smoothened value
		right_coil_smooth.publish(temp_array_avg)

		right_count = 0
		right_temp_array=np.zeros(right_smooth_window_size)






def Start_smoothing():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('Smoother_node', anonymous=True)




    #Global variable for publisher objects

    global left_coil_smooth
   # global middle_coil_smooth
    global right_coil_smooth


    #Publisher topics for smoothened values
    left_coil_smooth = rospy.Publisher('/left_coil_smooth', Float32, queue_size=1)
    #middle_coil_smooth = rospy.Publisher('/middle_coil_smooth', Float32, queue_size=1)
    right_coil_smooth = rospy.Publisher('/right_coil_smooth', Float32, queue_size=1)



    #Subcriber for alarm value of each coils and there callback
    rospy.Subscriber('/left_coil_alarm', Float32, left_coil_callback)
    #rospy.Subscriber('/middle_coil_alarm', Float32, middle_coil_callback)
    rospy.Subscriber('/right_coil_alarm', Float32, right_coil_callback)


    rospy.loginfo("Start running smoothing filter for metal detector alarm values")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    Start_smoothing()
