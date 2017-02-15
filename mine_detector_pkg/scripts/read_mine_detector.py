#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  read_mine_detector.py
#
#
#
# This code read three coil values and process the three channel value of three coil into three alarm value and send through three topics




import rospy
#Importing Metal detector messages called Coil
from metal_detector_msgs.msg._Coil import Coil

#Importing std_msgs
from std_msgs.msg import Int32
from std_msgs.msg import Float32

#Creating object of Coil() message header
leftCoil = Coil()
#middleCoil = Coil()
rightCoil = Coil()


#Topic of the difference of each channel
left_coil_alarm_topic = rospy.Publisher('left_coil_alarm',Float32, queue_size=1)
#middle_coil_alarm_topic = rospy.Publisher('middle_coil_alarm',Float32, queue_size=1)
right_coil_alarm_topic = rospy.Publisher('right_coil_alarm',Float32, queue_size=1)


# Metal detector raw value callback
def receiveCoilSignal(actualCoil):

	#Receving three coil raw data
    global leftCoil
    leftCoil = actualCoil.left_coil
    global rightCoil
    rightCoil = actualCoil.right_coil

	#Function to compute alarm value for each coil
    compute_coil_alarm_new()


def	compute_coil_alarm_new():
    left_mine_alarm = leftCoil
    left_coil_alarm_topic.publish(left_mine_alarm)
    right_mine_alarm = rightCoil
    right_coil_alarm_topic.publish(right_mine_alarm)

def start_mine_detect():

	#Initializing node
	rospy.init_node('Read_Metal_Detector')
	#Subscribing coil data
	rospy.Subscriber("/coils", Coil, receiveCoilSignal)

	rospy.loginfo('Started reading coil values and converting to alarm values')
	#Spin and wait
	rospy.spin()




if __name__ == '__main__':

	#Start mine detection
	start_mine_detect()
