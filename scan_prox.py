#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from dronekit import connect, VehicleMode
import time
import numpy as np
import math


def send_distance_message(dist,orient):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        1000,       # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        orient,     # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def callback(msg):
        scan = np.array (msg.ranges)
        left = (np.nanmin (scan[1:200:8]))*100
        center = (np.nanmin (scan[220:420:8]))*100
        right = np.nanmin ((scan[440:640:8]))*100	
        send_distance_message(left,7)
        send_distance_message(center,0)
        send_distance_message(right,1)	
        #print "%.2f" %  left , 
        #print "%.2f" %  center ,	
        #print "%.2f" %  right



# Connect to the Vehicle. 
print("\nConnecting to vehicle")
#vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True)
vehicle = connect('tcp:192.168.2.18:5763', wait_ready=True)

rospy.init_node('scan_values')
sub = rospy.Subscriber('/camera/scan', LaserScan, callback)
rospy.spin()


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

#https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION
#Value	Field Name	Description
#0	MAV_SENSOR_ROTATION_NONE	Roll: 0, Pitch: 0, Yaw: 0
#1	MAV_SENSOR_ROTATION_YAW_45	Roll: 0, Pitch: 0, Yaw: 45
#2	MAV_SENSOR_ROTATION_YAW_90	Roll: 0, Pitch: 0, Yaw: 90
#3	MAV_SENSOR_ROTATION_YAW_135	Roll: 0, Pitch: 0, Yaw: 135
#4	MAV_SENSOR_ROTATION_YAW_180	Roll: 0, Pitch: 0, Yaw: 180
#5	MAV_SENSOR_ROTATION_YAW_225	Roll: 0, Pitch: 0, Yaw: 225
#6	MAV_SENSOR_ROTATION_YAW_270	Roll: 0, Pitch: 0, Yaw: 270
#7	MAV_SENSOR_ROTATION_YAW_315	Roll: 0, Pitch: 0, Yaw: 315


