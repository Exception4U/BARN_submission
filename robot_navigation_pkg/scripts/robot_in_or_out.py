#!/usr/bin/env python
import rospy,sys
from collections import namedtuple
from pprint import pprint as pp
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix

Pt = namedtuple('Pt', 'x, y')               # Point
Edge = namedtuple('Edge', 'a, b')           # Polygon edge from a to b
Poly = namedtuple('Poly', 'name, edges')    # Polygon

_eps = 0.00001
_huge = sys.float_info.max
_tiny = sys.float_info.min

global gps_value_multi_factor
gps_value_multi_factor = 1



global gps_corner_1_lat
global gps_corner_1_long

global gps_corner_2_lat
global gps_corner_2_long

global gps_corner_3_lat
global gps_corner_3_long

global gps_corner_4_lat
global gps_corner_4_long


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


def rayintersectseg(p, edge):
    ''' takes a point p=Pt() and an edge of two endpoints a,b=Pt() of a line segment returns boolean
    '''
    try:
	    a,b = edge
	    if a.y > b.y:
		a,b = b,a
	    if p.y == a.y or p.y == b.y:
		p = Pt(p.x, p.y + _eps)

	    intersect = False

	    if (p.y > b.y or p.y < a.y) or (
		p.x > max(a.x, b.x)):
		return False

	    if p.x < min(a.x, b.x):
		intersect = True
	    else:
		if abs(a.x - b.x) > _tiny:
		    m_red = (b.y - a.y) / float(b.x - a.x)
		else:
		    m_red = _huge
		if abs(a.x - p.x) > _tiny:
		    m_blue = (p.y - a.y) / float(p.x - a.x)
		else:
		    m_blue = _huge
		intersect = m_blue >= m_red
	    return intersect
    except:
	rospy.loginfo("Exception in raycasting algorithm")
	pass

def _odd(x): return x%2 == 1

def ispointinside(p, poly):
    ln = len(poly)
    return _odd(sum(rayintersectseg(p, edge)
                    for edge in poly.edges ))

def receive_gps_data(gps_msg):
	global robot_gps_data
	global robot_in_out

	robot_gps_data = (gps_msg.latitude * gps_value_multi_factor,gps_msg.longitude * gps_value_multi_factor)


    	testpoints = (Pt(x=robot_gps_data[0], y=robot_gps_data[1]), Pt(x=5, y=8),
                  Pt(x=-10, y=5), Pt(x=0, y=5),
                  Pt(x=10, y=5), Pt(x=8, y=5),
                  Pt(x=10, y=10))


#    	for poly in polys:
#        	print '   ', '\t'.join("%s: %s" % (p, ispointinside(p, poly))
#                	               for p in testpoints[:1])

	try:
		is_robot_in =  ispointinside(testpoints[0],polys[0])
		rospy.set_param("/is_robot_in",is_robot_in)
#		temp_msg = Bool()
#		temp_msg.data = is_robot_in
		robot_in_out.publish(is_robot_in)
#	    	rospy.loginfo("Robot IN  = %d",is_robot_in)
		rospy.sleep(0.01)

	except:
		rospy.loginfo("Unable to set robot in out parameter")
		pass

if __name__ == '__main__':

	rospy.init_node('GPS_Checking_node')

	rospy.set_param("/is_robot_in",False)

	rospy.Subscriber("/gps/fix",NavSatFix, receive_gps_data)

	global robot_in_out

	robot_in_out = rospy.Publisher("/robot_in_out", Bool)


	global polys

        polys = [
      	Poly(name='strange', edges=(
        Edge(a=Pt(x=gps_corner_1_lat, y=gps_corner_1_long), b=Pt(x=gps_corner_2_lat, y=gps_corner_2_long)),
        Edge(a=Pt(x=gps_corner_2_lat, y=gps_corner_2_long), b=Pt(x=gps_corner_3_lat, y=gps_corner_3_long)),
        Edge(a=Pt(x=gps_corner_3_lat, y=gps_corner_3_long), b=Pt(x=gps_corner_4_lat, y=gps_corner_4_long)),
        Edge(a=Pt(x=gps_corner_4_lat, y=gps_corner_4_long), b=Pt(x=gps_corner_1_lat, y=gps_corner_1_long)),
        ))
      	]

	rospy.loginfo("Started GPS check")

	rospy.spin()

