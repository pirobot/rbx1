#!/usr/bin/env python

""" object_detector.py - Version 1.0 2012-02-11

    Based on the OpenCV findobj.py demo code 
"""

import roslib; roslib.load_manifest('rbx1_vision')
import rospy
from ros2opencv2 import ROS2OpenCV2
import sys
import numpy as np
import cv2
import cv
from common import anorm
from functools import partial

help_message = '''SURF image match 

USAGE: find_object.py [ <image1> <image2> ]
'''

FLANN_INDEX_KDTREE = 1  # bug: flann enums are missing

flann_params = dict(algorithm = FLANN_INDEX_KDTREE,
                    trees = 4)

class FindObject(ROS2OpenCV2):
    def __init__(self, node_name):
        ROS2OpenCV2.__init__(self, node_name)
        
        self.node_name = node_name
        
        """ Read in the template image """              
        template_file = rospy.get_param("~template_file", "")
        
        self.template = cv2.imread(template_file, cv.CV_LOAD_IMAGE_COLOR)
        
        cv2.imshow("Template", self.template)
        
        self.template = cv2.cvtColor(self.template, cv2.COLOR_RGB2GRAY)
        self.surf = cv2.SURF(1000)
        self.kp1, self.desc1 = self.surf.detect(self.template, None, False)
        
        self.frame_index = 0

    def process_image(self, cv_image):
        cv2.waitKey(5)
        
        self.frame_index += 1
        
        # Create a numpy array version of the image as required by many of the cv2 functions
        cv_array = np.array(cv_image, dtype=np.uint8)
                
        # Create a greyscale version of the image
        self.img2 = cv2.cvtColor(cv_array, cv2.COLOR_BGR2GRAY)

        if self.frame_index % 20 == 0:
            self.kp2, self.desc2 = self.surf.detect(self.img2, None, False)
            self.desc1.shape = (-1, self.surf.descriptorSize())
            self.desc2.shape = (-1, self.surf.descriptorSize())
            print 'Template - %d features, img2 - %d features' % (len(self.kp1), len(self.kp2))
        
            #print 'bruteforce match:',
            #vis_brute = self.match_and_draw( self.match_bruteforce, 0.75 )
            print 'flann match:',
            vis_flann = self.match_and_draw(self.match_flann, 0.6) # flann tends to find more distant second
                                                                   # neighbours, so r_threshold is decreased
            #cv2.imshow('find_object SURF', vis_brute)
            if vis_flann is not None:
                cv2.imshow('find_object SURF flann', vis_flann)
        
        return cv_image

    
    def match_bruteforce(self, desc1, desc2, r_threshold = 0.75):
        res = []
        for i in xrange(len(desc1)):
            dist = anorm( desc2 - desc1[i] )
            n1, n2 = dist.argsort()[:2]
            r = dist[n1] / dist[n2]
            if r < r_threshold:
                res.append((i, n1))
        return np.array(res)
    
    def match_flann(self, desc1, desc2, r_threshold = 0.6):
        flann = cv2.flann_Index(desc2, flann_params)
        idx2, dist = flann.knnSearch(desc1, 2, params = {}) # bug: need to provide empty dict
        mask = dist[:,0] / dist[:,1] < r_threshold
        idx1 = np.arange(len(desc1))
        pairs = np.int32( zip(idx1, idx2[:,0]) )
        return pairs[mask]
    
    def draw_match(self, img1, img2, p1, p2, status = None, H = None):
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
        vis[:h1, :w1] = img1
        vis[:h2, w1:w1+w2] = img2
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    
        if H is not None:
            corners = np.float32([[0, 0], [w1, 0], [w1, h1], [0, h1]])
            corners = np.int32( cv2.perspectiveTransform(corners.reshape(1, -1, 2), H).reshape(-1, 2) + (w1, 0) )
            cv2.polylines(vis, [corners], True, (255, 255, 255))
        
        if status is None:
            status = np.ones(len(p1), np.bool_)
        green = (0, 255, 0)
        red = (0, 0, 255)
        for (x1, y1), (x2, y2), inlier in zip(np.int32(p1), np.int32(p2), status):
            col = [red, green][inlier]
            if inlier:
                cv2.line(vis, (x1, y1), (x2+w1, y2), col)
                cv2.circle(vis, (x1, y1), 2, col, -1)
                cv2.circle(vis, (x2+w1, y2), 2, col, -1)
            else:
                r = 2
                thickness = 3
                cv2.line(vis, (x1-r, y1-r), (x1+r, y1+r), col, thickness)
                cv2.line(vis, (x1-r, y1+r), (x1+r, y1-r), col, thickness)
                cv2.line(vis, (x2+w1-r, y2-r), (x2+w1+r, y2+r), col, thickness)
                cv2.line(vis, (x2+w1-r, y2+r), (x2+w1+r, y2-r), col, thickness)
        return vis
    
    def draw_matches(self, img1, img2, p1, p2):
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        vis = np.zeros((max(h1, h2), w1+w2), np.uint8)
        vis[:h1, :w1] = img1
        vis[:h2, w1:w1+w2] = img2
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        
        if status is None:
            status = np.ones(len(p1), np.bool_)
        green = (0, 255, 0)
        red = (0, 0, 255)
        for (x1, y1), (x2, y2), inlier in zip(np.int32(p1), np.int32(p2), status):
            col = [red, green][inlier]
            if inlier:
                cv2.line(vis, (x1, y1), (x2+w1, y2), col)
                cv2.circle(vis, (x1, y1), 2, col, -1)
                cv2.circle(vis, (x2+w1, y2), 2, col, -1)
            else:
                r = 2
                thickness = 3
                cv2.line(vis, (x1-r, y1-r), (x1+r, y1+r), col, thickness)
                cv2.line(vis, (x1-r, y1+r), (x1+r, y1-r), col, thickness)
                cv2.line(vis, (x2+w1-r, y2-r), (x2+w1+r, y2+r), col, thickness)
                cv2.line(vis, (x2+w1-r, y2+r), (x2+w1+r, y2-r), col, thickness)
        return vis
    
    def match_and_draw(self, match, r_threshold):
        m = match(self.desc1, self.desc2, r_threshold)
        matched_p1 = np.array([self.kp1[i].pt for i, j in m])
        matched_p2 = np.array([self.kp2[j].pt for i, j in m])
        try:
            H, status = cv2.findHomography(matched_p1, matched_p2, cv2.RANSAC, 5.0)
            print '%d / %d  inliers/matched' % (np.sum(status), len(status))
            vis = self.draw_match(self.template, self.img2, matched_p1, matched_p2, status, H)
            #vis = self.draw_matches(self.template, self.img2, matched_p1, matched_p2)
        except:
            vis = None

        return vis

def main(args):
      FO = FindObject("find_object")
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down Find Object node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    