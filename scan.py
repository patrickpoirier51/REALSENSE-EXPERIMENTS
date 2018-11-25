#! /usr/bin/env python
 
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
 
def callback(msg):

	scan = np.array (msg.ranges)
	left = np.nanmin (scan[1:200:8])
	center = np.nanmin (scan[220:420:8])
	right = np.nanmin (scan[440:640:8])	
	
	print "%.2f" %  left , 
	print "%.2f" %  center ,	
	print "%.2f" %  right



rospy.init_node('scan_values')
sub = rospy.Subscriber('/camera/scan', LaserScan, callback)
rospy.spin()

