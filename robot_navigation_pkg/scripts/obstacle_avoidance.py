#!/usr/bin/env python
# -*- coding:utf8 -*-

import numpy as np
from numpy import deg2rad, cos, sin
import rospy, os, sys, curses, time, cv2, tf
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan, Imu
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool

'''
global left_threshold_data_limit 
left_threshold_data_limit = 100
global right_threshold_data_limit
right_threshold_data_limit = 100

global front_laser_data 
front_laser_data = []
'''

'''
global left_laser_data 
left_laser_data = []
global right_laser_data 
right_laser_data = []
'''

try:
	global front_threshold_data_limit
	front_threshold_data_limit = rospy.get_param("/front_threshold_data_limit",31)

	global front_laser_range_lower_threshold
	front_laser_range_lower_threshold = rospy.get_param("/front_laser_range_lower_threshold",1.5)

	global front_laser_range_upper_threshold
	front_laser_range_upper_threshold = rospy.get_param("/front_laser_range_upper_threshold",2.5)

except:
	rospy.logwarn("Unable to retrieve obstacle detection parameters")
	pass

'''
global left_laser_range_lower_threshold
left_laser_range_lower_threshold = 1.5

global left_laser_range_upper_threshold
front_laser_range_upper_threshold = 2.5


global right_laser_range_lower_threshold
right_laser_range_lower_threshold = 1.5

global right_laser_range_upper_threshold
right_laser_range_upper_threshold = 2.5
'''

#global obstacle_flag 
#obstacl

global front_laser_topic
front_laser_topic = rospy.Publisher('/front_scan', LaserScan)


# Laser range-finder callback
def receiveLaser(LaserNow):
    global laserInfo 
    global obstacle_flag 
    global front_laser_topic
    global front_laser_data

    global obstacle_detect
    obstacle_flag = False


    try:
	    #Assigning laser scan to a variable
	    laserInfo = LaserNow
	    #copy another variable for extracting front
	    front_laser = LaserNow


	    laser_data = laserInfo.ranges
	    laser_data_length = len(laserInfo.ranges)
	    laser_data_avg_length = int(laser_data_length /2)

	 #   rospy.loginfo("Laser count %d",laser_data_avg_length)
	#    rospy.loginfo("Angle min %d",laserInfo.angle_min)

	    front_laser_low = laser_data_avg_length - front_threshold_data_limit
	    front_laser_high = laser_data_avg_length + front_threshold_data_limit

	    front_laser_data = list(laser_data[front_laser_low:front_laser_high])
	    zero_low_pad = list(0 for i in range(0,front_laser_low)) 
	    zero_high_pad = list(0 for i in range(laser_data_avg_length,front_laser_high)) 

	    #Padding zero to front and back of laser data
	    zero_low_pad.extend(front_laser_data)
	    zero_low_pad.extend(zero_high_pad)

	    #Assigning new ranges to another message 
	    front_laser.ranges = zero_low_pad
	    front_laser_topic.publish(front_laser)
	 

	    obstacle_detect.publish(False)
	    for i in range(0, len(front_laser_data)):
	    	if(front_laser_data[i] > front_laser_range_lower_threshold and front_laser_data[i] < front_laser_range_upper_threshold):
			obstacle_flag = True
#	 		rospy.logwarn(front_laser_data[i])		   
	    
	    if(obstacle_flag == True):
	    	rospy.logwarn("Obstacle detected")
		rospy.set_param("/is_obstacle",True)
		obstacle_detect.publish(True)
		rospy.sleep(0.02)	
		obstacle_flag = False
		rospy.set_param("/is_obstacle",False)
		obstacle_detect.publish(False)

    except:
	    rospy.loginfo("Exception in obstacle avoidance loop")
	    pass


    	
    front_laser_data = 0    


if __name__ == '__main__':
    # Initialize client node

#    obstacle_flag = False

    rospy.init_node('obstacle_avoidance')
    rospy.loginfo("Start running Obstacle avoidance node")

    rospy.set_param("/is_obstacle",False)


    global obstacle_detect

    obstacle_detect = rospy.Publisher('/obsctacle_detection', Bool)



    rospy.Subscriber("/scan", LaserScan, receiveLaser)
    rospy.spin()


