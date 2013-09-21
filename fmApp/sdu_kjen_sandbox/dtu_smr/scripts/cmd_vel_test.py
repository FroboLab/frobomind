#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

topic_cmd_vel = '/cmd_vel'
update_interval = 0.1 # s
vel_lin = -0.0
vel_ang = 0.0

rospy.init_node('cmd_vel_test')
pub = rospy.Publisher(topic_cmd_vel, Twist)

while not rospy.is_shutdown():
	pub.publish(Twist(Vector3(vel_lin, 0, 0),Vector3(0, 0, vel_ang)))
	rospy.sleep(update_interval)
 

