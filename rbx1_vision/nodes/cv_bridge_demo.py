#!/usr/bin/env python

""" cv_bridge_demo.py - Version 1.1 2013-12-20

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

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

import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"
        
        rospy.init_node(self.node_name)
        
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        
        self.rgb_image = None
        self.depth_image = None
        
        # Create the OpenCV display window for the RGB image
        self.rgb_window_name = self.node_name
        cv2.namedWindow(self.rgb_window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.rgb_window_name, 25, 75)
                
        # And one for the depth image
        self.depth_window_name = "Depth Image"

        cv2.namedWindow(self.depth_window_name, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.depth_window_name, 25, 350)
        
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        rospy.loginfo("Waiting for image topics...")
        rospy.wait_for_message("input_rgb_image", Image)
        
        # Subscribe to the camera image and depth topics and set he appropriate callbacks
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("input_depth_image", Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo("Ready.")
        
        while not rospy.is_shutdown():
            if not self.rgb_image is None:
                cv2.imshow(self.rgb_window_name, self.rgb_image)
                      
                # Process any keyboard commands
                self.keystroke = cv2.waitKey(5)
                         
                if self.keystroke != -1:
                    cc = chr(self.keystroke & 255).lower()
                    if cc == 'q':
                        # The user has press the q key, so exit
                        rospy.signal_shutdown("User hit q key to quit.")
                
            if not self.depth_image is None:
                cv2.imshow(self.depth_window_name, self.depth_image)                

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        
        # Process the frame using the process_image() function
        self.rgb_image = self.process_image(frame)        
                
                
    def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError, e:
            print e
                
        # Normalize the depth image to fall between 0 (black) and 1 (white)
        depth_image = cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX, dtype=cv2.CV_32FC1)
        
        # Process the depth image
        self.depth_image = self.process_depth_image(depth_image)

          
    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        
        return edges
    
    def process_depth_image(self, frame):
        # Just return the raw image for this demo
        return frame
    
    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   
    
def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    