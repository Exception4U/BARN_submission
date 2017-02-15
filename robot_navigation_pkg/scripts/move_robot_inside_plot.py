#!/usr/bin/env python
import rospy,sys, math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

global gps_value_multi_factor
gps_value_multi_factor = 1000000
#gps_value_multi_factor = 1



global robot_current_angle
global current_robo_angle
global robot_direction

robot_current_angle = 0.0
current_robo_angle = 0.0
robot_direction = 'c'

try:
	gps_corner_1_lat = rospy.get_param("/minefield_static_tf_publisher/minefield/corner1/latitude",-30.060314832074897851 ) * gps_value_multi_factor
	gps_corner_1_long = rospy.get_param("/minefield_static_tf_publisher/minefield/corner1/longitude",-51.173918920793525444 ) * gps_value_multi_factor


	gps_corner_2_lat = rospy.get_param("/minefield_static_tf_publisher/minefield/corner2/latitude",-30.060314969251294315) * gps_value_multi_factor

	gps_corner_2_long = rospy.get_param("/minefield_static_tf_publisher/minefield/corner2/longitude",-51.173815175327305838) * gps_value_multi_factor



	gps_corner_3_lat = rospy.get_param("/minefield_static_tf_publisher/minefield/corner3/latitude",-30.060224724386614525) * gps_value_multi_factor
	gps_corner_3_long = rospy.get_param("/minefield_static_tf_publisher/minefield/corner3/longitude", -51.173815017676766104)* gps_value_multi_factor


	gps_corner_4_lat = rospy.get_param("/minefield_static_tf_publisher/minefield/corner4/latitude",-30.060224587210715441 ) * gps_value_multi_factor
	gps_corner_4_long = rospy.get_param("/minefield_static_tf_publisher/minefield/corner4/longitude",-51.173918763048902747 ) * gps_value_multi_factor


	rospy.loginfo("Retrieved GPS Limts")

except:
	rospy.logwarn("Unable to retrieved the GPS Limits, you should run simulation before running this node")
	pass

def Find_Slope_Angle(robot_pose,center_plot):
	try:
		y_diff = center_plot[1] - robot_pose[1]
		x_diff = center_plot[0] - robot_pose[0]


		if(x_diff == 0):
			if(y_diff >= 0):
				return 90
			elif(y_diff < 0):
				return -90

		slope = float(y_diff / x_diff)

		slope_angle = math.degrees(math.atan(slope))

#		rospy.logwarn("Slope %d",slope_angle)
		return slope_angle






	except:
		rospy.loginfo("Exception in calculating slope of robot and center of plot")
		pass

#Will update robot current angle

def update_robot_angle(angle_dat):
	global robot_current_angle

	if(angle_dat.data > 0 and angle_dat.data < 180):
		robot_current_angle = angle_dat.data

	if(angle_dat.data < 0 and angle_dat.data > -180):
		robot_current_angle = (180 + angle_dat.data)+180

#	rospy.loginfo("Robot current angle %f",robot_current_angle)

def Find_plot_center():

	global gps_corner_1_lat
	global gps_corner_1_long

	global gps_corner_2_lat
	global gps_corner_2_long

	global gps_corner_3_lat
	global gps_corner_3_long

	global gps_corner_4_lat
	global gps_corner_4_long

	global gps_value_multi_factor

	try:

		center_lat = (gps_corner_1_lat + gps_corner_3_lat)/2

#		center_lat = center_lat /gps_value_multi_factor
		center_lat = center_lat


		center_long = (gps_corner_1_long + gps_corner_3_long)/2
#		center_long = center_long /gps_value_multi_factor

		center_long = center_long


		center_gps_coord = [center_lat,center_long]

		return center_gps_coord

	except:
		rospy.loginfo("Exception in computing plot center")
		pass

def Detect_Left_or_Side(robot_gps_data,plot_center):
	global robot_direction
	global rotate_out_direction

#Left will be latitude is higher than right
	temp_diff = float(robot_gps_data[1] - plot_center[1])
#	rospy.loginfo(temp_diff)
	if(temp_diff >= 0):
#		rospy.loginfo("Robot is on right from center")
		robot_direction = 'r'
		rotate_out_direction.publish(robot_direction)

	if(temp_diff <= 0):
#		rospy.loginfo("Robot is on left from center")
		robot_direction = 'l'
		rotate_out_direction.publish(robot_direction)

	if(temp_diff == 0):
#		rospy.loginfo("Robot is on Middle")
		robot_direction = 'c'
		rotate_out_direction.publish(robot_direction)

	return robot_direction

def Find_rotate_angle(side, angle_bw_robot_plot, robot_gps_data):
	global robot_current_angle
	global rotate_angle_out


	try:

		if(side == 'l'):
			if(angle_bw_robot_plot <= 90 and angle_bw_robot_plot >= 0):

				rotate_angle = float(90 - angle_bw_robot_plot)



#				rospy.loginfo("Robot on left bottom, need to rotate %f , robot angle %f",rotate_angle,angle_bw_robot_plot)
				rospy.set_param("/robot_out_turn_angle",rotate_angle)
				rotate_angle_out.publish(rotate_angle)


			if(angle_bw_robot_plot < 0 and angle_bw_robot_plot >= -90):


				rotate_angle = 270 - angle_bw_robot_plot

#				rospy.loginfo("Robot on left top, need to rotate  %f , robot angle %f",rotate_angle,robot_current_angle)
				rospy.set_param("/robot_out_turn_angle",rotate_angle)
				rotate_angle_out.publish(rotate_angle)


		if(side == 'r'):
			if(angle_bw_robot_plot <= 90 and angle_bw_robot_plot >= 0):

				rotate_angle = 270 - angle_bw_robot_plot


#				rospy.loginfo("Robot on right top, need to rotate %f , robot angle %f",rotate_angle,robot_current_angle)
				rospy.set_param("/robot_out_turn_angle",rotate_angle)
				rotate_angle_out.publish(rotate_angle)



			if(angle_bw_robot_plot < 0 and angle_bw_robot_plot >= -90):

				rotate_angle = 90 - (angle_bw_robot_plot)

#				rospy.loginfo("Robot on right bottom, need to rotate  %f , robot angle %f",rotate_angle,robot_current_angle)
				rospy.set_param("/robot_out_turn_angle",rotate_angle)
				rotate_angle_out.publish(rotate_angle)


	except:
		rospy.loginfo("Unable to set robot rotation angle")
		pass

def receive_gps_data(gps_msg):
	global robot_gps_data
	global gps_value_multi_factor
	global rotate_angle_out

	robot_gps_data = (gps_msg.latitude * gps_value_multi_factor,gps_msg.longitude * gps_value_multi_factor)
#	rospy.loginfo("Robot GPS data")
#	rospy.loginfo(robot_gps_data)
#	rospy.loginfo("Plot center GPS data")
	plot_center = Find_plot_center()
	#Finding slope angle
	angle_bw_robot_plot = Find_Slope_Angle(robot_gps_data,plot_center)

	side = Detect_Left_or_Side(robot_gps_data,plot_center)
#	rospy.loginfo(side)

	Find_rotate_angle(side, angle_bw_robot_plot, robot_gps_data)

#	rospy.loginfo("Slope angle")
#	rospy.loginfo(angle_bw_robot_plot)

#	rospy.loginfo("Center of plot")
#	rospy.loginfo(plot_center)


if __name__ == '__main__':

	rospy.init_node('Move_Robot_in')

	rospy.set_param("/robot_out_turn_angle",0)

	rospy.Subscriber("/gps/fix",NavSatFix, receive_gps_data)

	global rotate_angle_out

	rotate_angle_out = rospy.Publisher('/rotate_angle_out', Float32)

	global rotate_out_direction

	rotate_out_direction = rospy.Publisher('/rotate_out_direction', String)


	rospy.Subscriber("/robot_angle",Float32, update_robot_angle)


	rospy.loginfo("Started Move_Robot_in")

	rospy.spin()

