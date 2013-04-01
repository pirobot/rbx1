#!/usr/bin/env python

""" match_template_simple.py - Version 1.0 2012-02-27

"""

import roslib; roslib.load_manifest('rbx1_vision')
import rospy
from ros2opencv2 import ROS2OpenCV2
from sensor_msgs.msg import Image, RegionOfInterest
import sys
import cv2.cv as cv
import cv2
import numpy as np

class MatchTemplate(ROS2OpenCV2):
    def __init__(self, node_name):
        ROS2OpenCV2.__init__(self, node_name)
        
        self.node_name = node_name

        self.use_depth_for_detection = rospy.get_param("~use_depth_for_detection", False)
        self.fov_width = rospy.get_param("~fov_width", 1.094)
        self.fov_height = rospy.get_param("~fov_height", 1.094)
        self.max_object_size = rospy.get_param("~max_object_size", 0.28)
        
        # Intialize the detection box
        self.detect_box = None
        
        # Initialize a couple of intermediate image variables
        self.grey = None
        self.small_image = None  
        
        # What kind of detector do we want to load
        self.detector_type = "template"
        self.detector_loaded = False
        
        rospy.loginfo("Waiting for video topics to become available...")

        # Wait until the image topics are ready before starting
        rospy.wait_for_message("input_rgb_image", Image)
        
        if self.use_depth_for_detection:
            rospy.wait_for_message("input_depth_image", Image)
            
        rospy.loginfo("Ready.")

    def process_image(self, cv_image):        
        # STEP 1. Load a detector if one is specified
        if self.detector_type and not self.detector_loaded:
            self.detector_loaded = self.load_detector(self.detector_type)
            
        # STEP 2: Detect the object
        self.detect_box = self.detect_roi(self.detector_type, cv_image)
                
        return cv_image
    
    def load_detector(self, detector):
        if detector == "template":
            #try:
            """ Read in the template image """              
            template_file = rospy.get_param("~template_file", "")
            
            #self.template =  cv2.equalizeHist(cv2.cvtColor(cv2.imread(template_file, cv.CV_LOAD_IMAGE_COLOR), cv2.COLOR_BGR2GRAY))
            #self.template = cv2.imread(template_file, cv.CV_LOAD_IMAGE_GRAYSCALE)
            #self.template = cv2.Sobel(self.template_image, cv.CV_32F, 1, 1)
            
            self.template = cv2.imread(template_file, cv.CV_LOAD_IMAGE_COLOR)
            
            cv2.imshow("Template", self.template)
                            
            return True
            #except:
                #rospy.loginfo("Exception loading face detector!")
                #return False
        else:
            return False
        
    def detect_roi(self, detector, cv_image):
        if detector == "template":
            detect_box = self.match_template(cv_image)
        
        return detect_box
    
    def match_template(self, cv_image):        
        frame = np.array(cv_image, dtype=np.uint8)
        
        #grey = cv2.equalizeHist(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))  
        #edges = cv2.Sobel(grey, cv.CV_32F, 1, 1)
        
        W,H = frame.shape[1], frame.shape[0]
        w,h = self.template.shape[1], self.template.shape[0]
        width = W - w + 1
        height = H - h + 1
        
        result = cv.CreateMat(height, width, cv.CV_32FC1)
        result_array = np.array(result, dtype = np.float32)

        cv2.matchTemplate(frame, self.template, cv.CV_TM_CCOEFF_NORMED, result_array)
        
        (min_score, max_score, minloc, maxloc) = cv2.minMaxLoc(result_array)
        
        #if max_score > 0.7:
            #return None
        (x, y) = maxloc

        match_box = (x, y, w, h)
        cv2.imshow("Match Result", result_array)
        #cv.Rectangle(self.marker_image, (x, y), (x + w, y + h),(255, 255, 0), 3, 0)
        return match_box

def main(args):
      FD = MatchTemplate("match_template")
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down match template node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    