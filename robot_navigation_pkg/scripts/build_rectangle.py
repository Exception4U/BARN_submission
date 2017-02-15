#!/usr/bin/env python
import rospy,sys
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix



global gps_value_multi_factor

#gps_value_multi_factor = 1000000
gps_value_multi_factor = 1

#Retrieve GPS corner values
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
        rospy.loginfo("Providing static and preloaded location values ")
        gps_corner_1_lat = rospy.get_param(-30.060314832074897851)* gps_value_multi_factor
        gps_corner_1_lat = rospy.get_param(-51.173918920793525444 )* gps_value_multi_factor
        gps_corner_2_lat = rospy.get_param(-30.060314969251294315)* gps_value_multi_factor
        gps_corner_2_long = rospy.get_param(-51.173815175327305838)* gps_value_multi_factor
        gps_corner_3_lat = rospy.get_param(-30.060224724386614525)* gps_value_multi_factor
        gps_corner_3_long = rospy.get_param(-51.173815017676766104)* gps_value_multi_factor
        gps_corner_4_lat = rospy.get_param(-30.060224587210715441)* gps_value_multi_factor
        gps_corner_4_long = rospy.get_param(-51.173918763048902747)* gps_value_multi_factor
	pass

def Find_Rectangle_Points():

#     A                   B
#
#     C                  D
#

	global gps_corner_1_lat
	global gps_corner_1_long

	global gps_corner_2_lat
	global gps_corner_2_long

	global gps_corner_3_lat
	global gps_corner_3_long

	global gps_corner_4_lat
	global gps_corner_4_long


#	left_x = max(gps_corner_1_lat,gps_corner_3_lat)
#	top_y = min(gps_corner_1_long,gps_corner_2_long)
#	right_x = min(gps_corner_2_lat, gps_corner_4_lat)
#	bot_y = max(gps_corner_3_long, gps_corner_4_long)


	left_x = max(gps_corner_1_lat,gps_corner_3_lat)
	top_y = min(gps_corner_1_long,gps_corner_2_long)
	right_x = min(gps_corner_2_lat, gps_corner_4_lat)
	bot_y = max(gps_corner_3_long, gps_corner_4_long)



	rectA = [left_x,top_y]
	rectB = [right_x,top_y]
	rectC = [left_x,bot_y]
	rectD = [right_x,bot_y]

	rospy.loginfo("C1")
	rospy.loginfo(rectA)


	rospy.loginfo("C2")
	rospy.loginfo(rectB)


	rospy.loginfo("C3")
	rospy.loginfo(rectC)


	rospy.loginfo("C4")
	rospy.loginfo(rectD)






if __name__ == '__main__':

	rospy.init_node("Build_rectangle_node")

#	rospy.Subscriber("/gps/fix",NavSatFix, receive_gps_data)

	Find_Rectangle_Points()

	rospy.loginfo("Started Rectangle node")

	rospy.spin()

