#!/usr/bin/env python

import roslib; roslib.load_manifest("rbx1_dynamixels")
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("fake_pub")
p = rospy.Publisher('joint_states', JointState)

msg = JointState()
msg.name = ["front_castor_joint", "left_wheel_joint", "rear_castor_joint", "right_wheel_joint"]
msg.position = [0.0 for name in msg.name]
msg.velocity = [0.0 for name in msg.name]

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    p.publish(msg)
    rospy.sleep(0.1)

