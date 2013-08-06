#!/usr/bin/env python

"""
    object_tracker2.py - Version 1.0 2013-07-18
    
    Follow a target published on the /roi topic using depth from the registered depth image.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('rbx1_apps')
import rospy
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Twist
from math import copysign, isnan
from cv2 import cv as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ObjectTracker():
    def __init__(self):
        rospy.init_node("object_tracker")
                        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # How often should we update the robot's motion?
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate) 
        
        # The maximum rotation speed in radians per second
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.5)
        
        # The x threshold (% of image width) indicates how far off-center
        # the ROI needs to be in the x-direction before we react
        self.x_threshold = rospy.get_param("~x_threshold", 0.1)
        
        # The maximum distance a target can be from the robot for us to track
        self.max_z = rospy.get_param("~max_z", 2.0)
        
        # Initialize the global ROI
        self.roi = RegionOfInterest()
        
        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.6)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.x_threshold = rospy.get_param("~x_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight x-displacement of the person when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 2.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        # Intialize the movement command
        self.move_cmd = Twist()
        
        # We will get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0
        
        # We need cv_bridge to convert the ROS depth image to an OpenCV array
        self.cv_bridge = CvBridge()
        self.depth_array = None
        
        # Set flag to indicate when the ROI stops updating
        self.target_visible = False
        
        # Wait for the camera_info topic to become available
        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message('camera_info', CameraInfo)
        
        # Subscribe to the camera_info topic to get the image width and height
        rospy.Subscriber('camera_info', CameraInfo, self.get_camera_info)

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
                    
        # Subscribe to the registered depth image
        rospy.Subscriber("depth_image", Image, self.convert_depth_image)
        
        # Wait for the depth image to become available
        rospy.wait_for_message('depth_image', Image)
        
        # Subscribe to the ROI topic and set the callback to update the robot's motion
        rospy.Subscriber('roi', RegionOfInterest, self.set_cmd_vel)
        
        # Wait until we have an ROI to follow
        rospy.loginfo("Waiting for an ROI to track...")
        rospy.wait_for_message('roi', RegionOfInterest)
        
        rospy.loginfo("ROI messages detected. Starting tracker...")
        
        # Begin the tracking loop
        while not rospy.is_shutdown():
            # If the target is not visible, stop the robot
            if not self.target_visible:
                self.move_cmd = Twist()
            else:
                # Reset the flag to False by default
                self.target_visible = False
                
            # Send the Twist command to the robot
            self.cmd_vel_pub.publish(self.move_cmd)
            
            # Sleep for 1/self.rate seconds
            r.sleep()

    def set_cmd_vel(self, msg):
        # If the ROI has a width or height of 0, we have lost the target
        if msg.width == 0 or msg.height == 0:
            return
        
        # If the ROI stops updating this next statement will not happen
        self.target_visible = True
        
        self.roi = msg
        
        # Compute the displacement of the ROI from the center of the image
        target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2

        try:
            percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
        except:
            percent_offset_x = 0
            
        # Intialize the movement command
        self.move_cmd = Twist()
        
        # Rotate the robot only if the displacement of the target exceeds the threshold
        if abs(percent_offset_x) > self.x_threshold:
            # Set the rotation speed proportional to the displacement of the target
            try:
                speed = percent_offset_x * self.x_scale
                self.move_cmd.angular.z = -copysign(max(self.min_rotation_speed,
                                            min(self.max_rotation_speed, abs(speed))), speed)
            except:
                pass
            
        # Now compute the depth component
        n_z = sum_z = mean_z = 0
         
        # Get the min/max x and y values from the ROI
        min_x = self.roi.x_offset
        max_x = min_x + self.roi.width
        min_y = self.roi.y_offset
        max_y = min_y + self.roi.height
        
        # Get the average depth value over the ROI
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                try:
                    z = self.depth_array[y, x]
                except:
                    continue
                
                # Depth values can be NaN which should be ignored
                if isnan(z) or z > self.max_z:
                    continue
                else:
                    sum_z = sum_z + z
                    n_z += 1   
        try:
            mean_z = sum_z / n_z
            
            if mean_z < self.max_z and (abs(mean_z - self.goal_z) > self.z_threshold):
                speed = (mean_z - self.goal_z) * self.z_scale
                self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
        except:
            pass
                    
    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.cv_bridge.imgmsg_to_cv(ros_image, "32FC1")
        except CvBridgeError, e:
            print e

        # Convert the depth image to a Numpy array
        self.depth_array = np.array(depth_image, dtype=np.float32)

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     

if __name__ == '__main__':
    try:
        ObjectTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object tracking node terminated.")

