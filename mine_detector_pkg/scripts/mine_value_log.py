#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  read_mine_detector.py
#
#
#
# This code read three coil values and process the three channel value of three coil into three alarm value and send through three topics




import rospy,tf,csv,sys
#Importing Metal detector messages called Coil
from metal_detector_msgs.msg._Coil import Coil

#Importing std_msgs
from std_msgs.msg import Int32
from std_msgs.msg import Float32



#Module for getting transformation and setting mine position
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped




#Creating object of Coil() message header
leftCoil = Coil()
middleCoil = Coil()
rightCoil = Coil()


transListener = None

#Topic of the difference of each channel
left_coil_alarm_topic = rospy.Publisher('left_coil_alarm',Float32, queue_size=1)
#middle_coil_alarm_topic = rospy.Publisher('middle_coil_alarm',Float32, queue_size=1)
right_coil_alarm_topic = rospy.Publisher('right_coil_alarm',Float32, queue_size=1)


try:
	#Files for plotting Left channel 1,2,3 values
	left_coil_val = open('left_coil_first_value.csv','wt')
	left_coil_val_writer = csv.writer(left_coil_first_val)
#	first_val_writer.writerow( (1, 2, 3) )

	#######################################################################


	right_coil_val = open('right_coil_first_value.csv','wt')
	right_coil_val_writer = csv.writer(right_coil_val)


	#######################################################################

except:
	rospy.logwarn("Unable to open log files")
	sys.exit(0)

def log_left_data(channel_value,odom_x_val):
	global left_coil_val_writer


	rospy.loginfo("Writing left data vs odom")
	left_coil_val_writer.writerow( (odom_x_val,channel_value) )



def log_right_data(channel_value,odom_x_val):
	global right_coil_first_val_writer

	rospy.loginfo("Writing right data vs odom")
	right_coil_val_writer.writerow( (odom_x_val,channel_value) )




def get_coil_transform(coil_name):



        try:
		if(coil_name == 'l'):
			(trans, rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))

		#if(coil_name == 'm'):
		#	(trans, rot) = transListener.lookupTransform('minefield', 'middle_coil', rospy.Time(0))
		#	(trans, rot) = transListener.lookupTransform('minefield', 'middle_coil', rospy.Time(0))

		if(coil_name == 'r'):
			(trans, rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))
			(trans, rot) = transListener.lookupTransform('minefield', 'left_coil', rospy.Time(0))




        except:
		print "Error from transform"
                return 0


        cx, cy, cz = trans
        return cx


# Metal detector raw value callback
def receiveCoilSignal(actualCoil):
	#Receving three coil raw data
    global leftCoil
    leftCoil = actualCoil.left_coil
    global rightCoil
    rightCoil = actualCoil.right_coil

	#Function to compute alarm value for each coil
	compute_coil_alarm_new()

def compute_coil_alarm_new():
    left_coil_alarm = left_coil
	left_coil_alarm_topic.publish(left_mine_alarm)
	log_left_data(left_coil,odom_x)

    right_coil_alarm = right_coil
    right_coil_alarm_topic.publish(right_mine_alarm)
	log_right_data(right_coil,odom_x)

def start_mine_detect():
	global transListener
	#Initializing node
	rospy.init_node('Read_Metal_Detector')

	transListener = tf.TransformListener()


	#Subscribing coil data
	rospy.Subscriber("/coils", Coil, receiveCoilSignal)

	rospy.loginfo('Started reading coil values and converting to alarm values')
	#Spin and wait
	rospy.spin()

if __name__ == '__main__':

	#Start mine detection
	start_mine_detect()
