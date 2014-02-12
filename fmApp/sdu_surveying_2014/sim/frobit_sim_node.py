#!/usr/bin/env python
#/****************************************************************************
# frobit simulator
# Copyright (c) 2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#	* Neither the name FroboMind nor the
#	  names of its contributors may be used to endorse or promote products
#	  derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
A simple differentially steered robot (frobit) simulator. 

2014-01-20 KJ First version
"""

import rospy
from math import sin, cos, pi
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from differential_kinematics import differential_kinematics

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines
		self.count = 0

		# static parameters
		self.update_rate = 50 # set update frequency [Hz]
		self.deadman_tout_duration = 0.2 # [s]
		self.cmd_vel_tout_duration = 0.2 # [s]

		# get parameters
		self.w_dist = rospy.get_param("/diff_steer_wheel_distance", 0.2) # [m]
		self.ticks_per_meter_left = rospy.get_param("/ticks_per_meter_left", 500)
		self.ticks_per_meter_right = rospy.get_param("/ticks_per_meter_right", 500)

		acc_lin_max = rospy.get_param("~max_linear_acceleration", 1.0) # [m/s^2]
		acc_ang_max = rospy.get_param("~max_angular_acceleration", 0.1) # [rad/s^2]

		# get topic names
		self.topic_deadman = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		self.topic_cmd_vel = rospy.get_param("~cmd_vel_sub",'/fmCommand/cmd_vel')
		self.topic_pose = rospy.get_param("~pose_pub",'/fmKnowledge/pose')

		# initialize internal variables
		self.update_interval = 1/(self.update_rate * 1.0)
		self.pi2 = 2*pi
		self.cmd_vel_tout_active = True
		self.pose = [0.0, 0.0, 0.0]
		self.deadman_tout = 0.0
		self.vel_lin_desired = 0.0
		self.vel_ang_desired = 0.0
		self.acc_lin_max_step = acc_lin_max*self.update_interval		
		self.acc_ang_max_step = acc_ang_max*self.update_interval		
		self.vel_lin = 0.0
		self.vel_ang = 0.0
		self.ref_vel_left = 0.0
		self.ref_vel_right = 0.0

		self.dk = differential_kinematics(self.w_dist)

		# setup topic publishers
		self.pose_pub = rospy.Publisher(self.topic_pose, Odometry)
		self.pose_msg = Odometry()

		# setup subscription topic callbacks
		rospy.Subscriber(self.topic_deadman, Bool, self.on_deadman_message)
		rospy.Subscriber(self.topic_cmd_vel, TwistStamped, self.on_cmd_vel_message)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def accelerate_vel (self, vel_actual, vel_desired, max_step):
		# update velocity while respecting max acceleration
		# print '%.4f, %.4f, %.4f' % (vel_actual, vel_desired, max_step)
		if vel_actual < vel_desired:
			vel_actual += max_step
			if vel_actual > vel_desired:
				vel_actual = vel_desired
		elif vel_actual > vel_desired:
			vel_actual -= max_step
			if vel_actual < vel_desired:
				vel_actual = vel_desired
		return vel_actual

	def update_vel (self):
		if self.deadman_tout < rospy.get_time():
			self.vel_lin_desired = 0.0
			self.vel_ang_desired = 0.0

		self.vel_lin = self.accelerate_vel (self.vel_lin, self.vel_lin_desired, self.acc_lin_max_step)
		self.vel_ang = self.accelerate_vel (self.vel_ang, self.vel_ang_desired, self.acc_ang_max_step)

		# calculate coresponding left and right wheel speed [m/s]
		(self.ref_vel_left, self.ref_vel_right) = self.dk.inverse(self.vel_lin, self.vel_ang)

		# convert to [ticks/update_interval]
		# self.ref_ticks_left = self.ref_vel_left*self.ticks_per_meter_left;
		# self.ref_ticks_right = self.ref_vel_right*self.ticks_per_meter_right;

	def update_sim (self):
		(sim_vel_lin, sim_vel_ang) = self.dk.forward(self.ref_vel_left, self.ref_vel_right)

		ds = [sim_vel_lin*self.update_interval, 0]
		da = sim_vel_ang* self.update_interval
		rot_ds = self.vec2d_rot(ds, self.pose[2] + da/2.0)	
		self.pose[0] += rot_ds[0]
		self.pose[1] += rot_ds[1]
		self.pose[2] += da
	

	def on_deadman_message(self, msg):
		if msg.data == True:
			self.deadman_tout = rospy.get_time() + self.deadman_tout_duration
		else:
			self.deadman_tout = 0

	def on_cmd_vel_message(self, msg):
		# update timeout
		self.cmd_vel_tout = rospy.get_time() + self.cmd_vel_tout_duration

		# retrieve linear and angular velocity from the message
		self.vel_lin_desired = msg.twist.linear.x
		self.vel_ang_desired = msg.twist.angular.z

		if self.cmd_vel_tout_active:
			self.cmd_vel_tout_active = False
			rospy.loginfo (rospy.get_name() + ': Receiving cmd_vel')

	def publish_pose_message(self):
		self.pose_msg.header.stamp = rospy.Time.now()
		self.pose_msg.pose.pose.position.x = self.pose[0]
		self.pose_msg.pose.pose.position.y = self.pose[1]
		self.pose_msg.pose.pose.position.z = 0.0
		q = quaternion_from_euler (0, 0, self.pose[2])
		self.pose_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
		self.pose_pub.publish(self.pose_msg); # publish the pose message

	def updater(self):
		while not rospy.is_shutdown():
			self.update_vel()
			self.update_sim()
			self.publish_pose_message()

			# check for timeouts
			if self.cmd_vel_tout_active == False:
				if rospy.get_time() > self.cmd_vel_tout:
					self.cmd_vel_tout_active = True
					self.vel_lin_desired = 0.0
					self.vel_ang_desired = 0.0
		 			rospy.logwarn (rospy.get_name() + ': cmd_vel timeout')

			# go back to sleep
			self.r.sleep()

	def vec2d_rot(self, v, theta): # return the vector v rotated by theta
		rot_x = v[0]*cos(theta) - v[1]*sin(theta)
		rot_y = v[0]*sin(theta) + v[1]*cos(theta)
		return ([rot_x, rot_y])
	
	def angle_limit (self, angle): # return angle within [0;2pi[
		while angle < 0:
			angle += self.pi2
		while angle >= self.pi2:
			angle -= self.pi2
		return angle


# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('frobit_sim')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

