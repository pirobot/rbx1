#!/usr/bin/env python

"""
    head_tracker.py - Version 1.0 2010-12-28
    
    Move the head to track a target published on the /roi topic.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

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

import roslib; roslib.load_manifest('rbx1_dynamixels')
import rospy
from sensor_msgs.msg import JointState, RegionOfInterest, CameraInfo
from dynamixel_controllers.srv import *
from std_msgs.msg import Float64
from math import radians

class HeadTracker():
    def __init__(self):
        rospy.init_node("head_tracker")
        
        rospy.on_shutdown(self.shutdown)
        
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        
        # The namespace and joints parameter needs to be set by the servo controller
        # (The namespace is usually null.)
        namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/joints', '')
        
        # What are the names of the pan and tilt joint in the list of dynamixels?
        self.head_pan_joint = 'head_pan_joint'
        self.head_tilt_joint = 'head_tilt_joint'
        
        # Joint speeds are given in radians per second
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.max_joint_speed = rospy.get_param('~max_joint_speed', 0.5)
        
        # How far ahead or behind the target (in radians) should we aim for?
        self.lead_target_angle = rospy.get_param('~lead_target_angle', 0.5)
        
        # The pan/tilt thresholds indicate what percentage of the image window
        # the ROI needs to be off-center before we make a movement
        self.pan_threshold = int(rospy.get_param("~pan_threshold", 0.05))
        self.tilt_threshold = int(rospy.get_param("~tilt_threshold", 0.05))
        
        # The gain_pan and gain_tilt parameter determine how responsive the servo movements are.
        # If these are set too high, oscillation can result.
        self.gain_pan = rospy.get_param("~gain_pan", 0.5)
        self.gain_tilt = rospy.get_param("~gain_tilt", 0.5)
        
        # Set limits on the pan and tilt angles
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
        
        # Initialize the servo services and publishers
        self.init_servos()
                
        # Center the pan and tilt servos at the start
        rospy.loginfo("Centering servos...")
        self.center_head_servos()

        # Set a flag to indicate when the target has been lost
        self.target_visible = False
        
        # Set a counter to determine when we should give up on the target
        self.target_lost_count = 0
        
        # How long we are willing to wait before giving up
        self.max_target_lost_count = self.rate * 5
        
        self.last_tilt_speed = 0
        self.last_pan_speed = 0
        
        # Wait for messages on the three topics we need to monitor
        rospy.loginfo("Waiting for roi and camera_info topics...")
        rospy.wait_for_message('camera_info', CameraInfo)
        rospy.wait_for_message('joint_states', JointState)
        rospy.wait_for_message('roi', RegionOfInterest)
        
        # Monitor the joint states of the pan and tilt servos
        self.joint_state = JointState()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)
        
        # Wait until we actually have joint state values
        while self.joint_state == JointState():
            rospy.sleep(1)

        # Subscribe to camera_info topics and set the callback
        self.image_width = self.image_height = 0
        rospy.Subscriber('camera_info', CameraInfo, self.get_camera_info)
        
        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
            
        # Subscribe to roi topics and set the callback
        rospy.Subscriber('roi', RegionOfInterest, self.set_joint_cmd)
        
        rospy.loginfo("Ready to track target.")
                
        while not rospy.is_shutdown():
            # If we have lost the target, stop the servos
            if not self.target_visible:
                self.pan_speed = 0
                self.tilt_speed = 0
                # Keep track of how long the target is lost
                self.target_lost_count += 1
            else:
                self.target_visible = False
                self.target_lost_count = 0
            
            # If the target is lost long enough, center the servos
            if self.target_lost_count > self.max_target_lost_count:
                self.center_head_servos()
            else:               
                try:
                    # Only update the pan speed if it differs from the last value
                    if self.last_pan_speed != self.pan_speed:
                        self.set_servo_speed(self.head_pan_joint, self.pan_speed)
                        self.last_pan_speed = self.pan_speed
                    self.set_servo_position(self.head_pan_joint, self.pan_position)
                except:
                    # If there are exceptions, momentarily stop the head movement by setting
                    # the target pan position to the current position
                    try:
                        current_pan_position = self.tilt_speedlf.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
                        self.set_servo_position(self.head_pan_joint, current_pan_position)
                        rospy.logerr("Servo SetSpeed Exception!")
                        rospy.logerr(sys.exc_info())
                    except:
                        pass
                                 
                try:
                    # Only update the tilt speed if it differs from the last value
                    if self.last_tilt_speed != self.tilt_speed:
                        self.set_servo_speed(self.head_tilt_joint, self.tilt_speed)
                        self.last_tilt_speed = self.tilt_speed
                    self.set_servo_position(self.head_tilt_joint, self.tilt_position)
                except:
                    # If there are exceptions, momentarily stop the head movement by setting
                    # the target tilt position to the current position
                    try:
                        current_tilt_position = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
                        self.set_servo_position(self.head_tilt_joint, current_tilt_position)
                        rospy.logerr("Servo SetSpeed Exception!")
                        rospy.logerr(sys.exc_info())
                    except:
                        pass
                                    
            r.sleep()
            
    def init_servos(self):
        # Create dictionaries to hold the speed, position and torque controllers
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        # Connect to the set_speed and torque_enable services and
        # define a position publisher for each servo
        rospy.loginfo("Waiting for joint controllers services...")
        
        for controller in sorted(self.joints):
            try:
                # The set_speed services
                set_speed_service = '/' + controller + '/set_speed'
                rospy.wait_for_service(set_speed_service)  
                self.servo_speed[controller] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)
                
                # Initialize the servo speed to the default_joint_speed
                self.servo_speed[controller](self.default_joint_speed)
                
                # Torque enable/disable control for each servo
                torque_service = '/' + controller + '/torque_enable'
                rospy.wait_for_service(torque_service) 
                self.torque_enable[controller] = rospy.ServiceProxy(torque_service, TorqueEnable)
                
                # Start each servo in the disabled state so we can move them by hand
                self.torque_enable[controller](False)

                # The position controllers
                self.servo_position[controller] = rospy.Publisher('/' + controller + '/command', Float64)
            except:
                rospy.logerr("Can't contact servo services!")
        
        self.pan_position = 0
        self.tilt_position = 0
        self.pan_speed = 0
        self.tilt_speed = 0
        
        self.last_tilt_speed = 0
        self.last_pan_speed = 0
        
    def set_servo_speed(self, servo, speed):
        """ A speed of exactly 0 has a special meaning for Dynamixel servos--namely, "move as fast as you can".
            This can have some very undesirable consequences since it is the complete opposite of what 0 normally
            means.  So we define a very small speed value to represent zero speed. """
        if speed == 0:
            speed = 0.0001
        
        self.servo_speed[servo](speed)
        
    def set_servo_position(self, servo, position):
        self.servo_position[servo].publish(position)
        
    def set_joint_cmd(self, msg):
        # If the ROI stops updating this next statement will not happen
        self.target_visible = True

        # Compute the displacement of the ROI from the center of the image
        target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2
        target_offset_y = msg.y_offset + msg.height / 2 - self.image_height / 2
        
        try:
            percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
            percent_offset_y = float(target_offset_y) / (float(self.image_height) / 2.0)
        except:
            percent_offset_x = 0
            percent_offset_y = 0
          
        # Pan the camera only if the x target offset exceeds the threshold
        if abs(percent_offset_x) > self.pan_threshold:
            # Set the pan speed proportion to the target offset
            self.pan_speed = trunc(min(self.max_joint_speed, max(0, self.gain_pan * abs(percent_offset_x))), 2)
            
            # Set the target position ahead or behind the current position
            current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]

            if target_offset_x > 0:
                self.pan_position = max(self.min_pan, current_pan - self.lead_target_angle)
            else:
                self.pan_position = min(self.max_pan, current_pan + self.lead_target_angle)
        else:
            self.pan_speed = 0
        
        # Tilt the camera only if the y target offset exceeds the threshold
        if abs(percent_offset_y) > self.tilt_threshold:
            # Set the tilt speed proportion to the target offset
            self.tilt_speed = trunc(min(self.max_joint_speed, max(0, self.gain_tilt * abs(percent_offset_y))), 2)

            # Set the target position ahead or behind the current position
            current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]

            if target_offset_y < 0:
                self.tilt_position = max(self.min_tilt, current_tilt - self.lead_target_angle)
            else:
                self.tilt_position = min(self.max_tilt, current_tilt + self.lead_target_angle)
        else:
            self.tilt_speed = 0
            
    def center_head_servos(self):
        try:
            self.servo_speed[self.head_pan_joint](self.default_joint_speed)
            self.servo_speed[self.head_tilt_joint](self.default_joint_speed)
            for i in range(3):
                self.servo_position[self.head_pan_joint].publish(0)
                self.servo_position[self.head_tilt_joint].publish(0)
                rospy.sleep(1)
        except:
            pass
            
    def update_joint_state(self, msg):
        self.joint_state = msg
        
    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
    def shutdown(self):
        rospy.loginfo("Shutting down head tracking node...")
        self.center_head_servos()
        
        # Relax all servos to give them a rest.
        for servo in self.joints:
            self.torque_enable[servo](False)
        
def trunc(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

            
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
    def shutdown(self):
        rospy.loginfo("Shutting down head tracking node...")         
                   
if __name__ == '__main__':
    try:
        HeadTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node terminated.")




