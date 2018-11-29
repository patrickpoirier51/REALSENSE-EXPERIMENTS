## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

from dronekit import connect, VehicleMode
import pyrealsense2 as rs
import numpy as np
import time
import math

#Assign alternative path for OpenCV under ROS
import sys
sys.path.append("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2


#https://mavlink.io/en/messages/common.html#MAV_SENSOR_ORIENTATION
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

    
# Connect to the Vehicle. 
print("\nConnecting to vehicle")
#vehicle = connect('udpin:0.0.0.0:14551', wait_ready=True)
vehicle = connect('tcp:192.168.2.18:5763', wait_ready=True)



# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
scale = 0.1

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        #color_frame = frames.get_color_frame()
        if not depth_frame:  #or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        #color_image = np.asanyarray(color_frame.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.05), cv2.COLORMAP_HSV)

        l_top = depth_image[0:160,0:212]*scale
        l_top_mask = np.ma.masked_equal(l_top, 0.00, copy=False)
        l_top_min = l_top_mask.min()     
        l_center = depth_image[161:320,0:212]*scale
        l_center_mask = np.ma.masked_equal(l_center, 0.00, copy=False)
        l_center_min = l_center_mask.min()           
        l_bottom = depth_image[320:480,0:212]*scale
        l_bottom_mask = np.ma.masked_equal(l_bottom, 0.00, copy=False)
        l_bottom_min = l_bottom_mask.min()

           
        c_top = depth_image[0:160,213:426]*scale
        c_top_mask = np.ma.masked_equal(c_top, 0.00, copy=False)
        c_top_min = c_top_mask.min()     
        c_center = depth_image[161:320,213:426]*scale
        c_center_mask = np.ma.masked_equal(c_center, 0.00, copy=False)
        c_center_min = c_center_mask.min()
        c_bottom = depth_image[320:480,213:426]*scale
        c_bottom_mask = np.ma.masked_equal(c_bottom, 0.00, copy=False)
        c_bottom_min = c_bottom_mask.min()

     
        r_top = depth_image[0:160,427:640]*scale
        r_top_mask = np.ma.masked_equal(r_top, 0.00, copy=False)
        r_top_min = r_top_mask.min()                  
        r_center = depth_image[161:320,427:640]*scale
        r_center_mask = np.ma.masked_equal(r_center, 0.00, copy=False)
        r_center_min = r_center_mask.min()          
        r_bottom = depth_image[320:480,427:640]*scale
        r_bottom_mask = np.ma.masked_equal(r_bottom, 0.00, copy=False)
        r_bottom_min = r_bottom_mask.min()

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


        # Send vectors to Mavlink according to distance and azimuth (above)
        send_distance_message (l_center_min,7) 
        send_distance_message (c_center_min,0)
        send_distance_message (r_center_min,1)

        #print min matrix
        #print ("%.2f %.2f %.2f ") % (l_top_min , c_top_min , r_top_min)
        print ("%.2f %.2f %.2f ") % (l_center_min , c_center_min , r_center_min)
        #print ("%.2f %.2f %.2f ") % (l_bottom_min , c_bottom_min , r_bottom_min)
        #print ""
        #print ""

        
        # Stack both images horizontally
        #l_images = np.vstack((l_top,l_center,l_bottom))
        #c_images = np.vstack((c_top,c_center,c_bottom))
        #r_images = np.vstack((r_top,r_center,r_bottom))
        #full_images = np.hstack ((l_images,c_images,r_images))
        
        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense',depth_colormap )
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()

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
