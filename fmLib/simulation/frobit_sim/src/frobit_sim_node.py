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
from msgs.msg import FloatArrayStamped, PropulsionModuleFeedback
from tf.transformations import quaternion_from_euler
from differential_ifk_py.differential_kinematics import differential_kinematics
import numpy as np
from numpy import random

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
		acc_ang_max = rospy.get_param("~max_angular_acceleration", 1.8) # [rad/s^2]
		self.wheel_speed_variance = 0.001
		self.wheel_speed_delay = 0.05 # [s]
		self.wheel_speed_delay_variance = 0.05
		self.wheel_speed_error = 0.03 # [m/s]
		self.wheel_speed_minimum = 0.0

		pub_fb_rate = rospy.get_param("~publish_wheel_feedback_rate", 0)
		if pub_fb_rate != 0:
			self.pub_fb_interval = int(self.update_rate/pub_fb_rate)
		else:
			self.pub_fb_interval = 0

		# get topic names
		self.topic_deadman = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		self.topic_cmd_vel = rospy.get_param("~cmd_vel_sub",'/fmCommand/cmd_vel')
		self.topic_odom_reset = rospy.get_param("~odom_reset_sub",'/fmInformation/odom_reset')
		self.topic_pose = rospy.get_param("~pose_pub",'/fmKnowledge/pose')
		self.topic_w_fb_left_pub = rospy.get_param("~wheel_feedback_left_pub",'/fmInformation/wheel_feedback_left')
		self.topic_w_fb_right_pub = rospy.get_param("~wheel_feedback_right_pub",'/fmInformation/wheel_feedback_right')

		# initialize internal variables
		self.update_interval = 1/(self.update_rate * 1.0)
		self.pi2 = 2*pi
		self.cmd_vel_tout_active = True
		self.pose = [0.0, 0.0, 0.0]
		self.deadman_tout = 0.0
		self.cmd_vel_msgs = []
		self.vel_lin_desired = 0.0
		self.vel_ang_desired = 0.0
		self.acc_lin_max_step = acc_lin_max*self.update_interval		
		self.acc_ang_max_step = acc_ang_max*self.update_interval		
		self.vel_lin = 0.0
		self.vel_ang = 0.0
		self.ref_vel_left = 0.0
		self.ref_vel_right = 0.0
		self.sim_vel_left = 0.0
		self.sim_vel_right = 0.0
		self.wheel_speed_err_left = 0.0	
		self.wheel_speed_err_right = 0.0

		self.dk = differential_kinematics(self.w_dist)

		# setup topic publishers
		self.pose_pub = rospy.Publisher(self.topic_pose, Odometry)
		self.pose_msg = Odometry()

		# setup wheel feedback topic publisher
		if self.pub_fb_interval > 0:
			self.wl_fb_vel_set = 0.0
			self.wr_fb_vel_set = 0.0
			self.w_fb_left_pub = rospy.Publisher(self.topic_w_fb_left_pub, PropulsionModuleFeedback)
			self.w_fb_right_pub = rospy.Publisher(self.topic_w_fb_right_pub, PropulsionModuleFeedback)
			self.w_fb = PropulsionModuleFeedback()

		# setup subscription topic callbacks
		rospy.Subscriber(self.topic_deadman, Bool, self.on_deadman_message)
		rospy.Subscriber(self.topic_cmd_vel, TwistStamped, self.on_cmd_vel_message)
		rospy.Subscriber(self.topic_odom_reset, FloatArrayStamped, self.on_odom_reset_message)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def accelerate_vel (self, vel_actual, vel_desired, max_step):
		# update velocity while respecting max acceleration
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
		now = rospy.get_time()

		wheel_update = now - self.wheel_speed_delay + np.random.randn()*self.wheel_speed_delay_variance
		while len(self.cmd_vel_msgs) > 0 and self.cmd_vel_msgs[0][0] < wheel_update:
			self.vel_lin_desired = self.cmd_vel_msgs[0][1]
			self.vel_ang_desired = self.cmd_vel_msgs[0][2]
			(self.wl_fb_vel_set, self.wr_fb_vel_set) = self.dk.inverse(self.vel_lin_desired, self.vel_ang_desired)
			del(self.cmd_vel_msgs[0])

		if self.deadman_tout < now:
			self.vel_lin_desired = 0.0
			self.vel_ang_desired = 0.0
	
		self.vel_lin = self.accelerate_vel (self.vel_lin, self.vel_lin_desired, self.acc_lin_max_step)
		self.vel_ang = self.accelerate_vel (self.vel_ang, self.vel_ang_desired, self.acc_ang_max_step)

		# calculate coresponding left and right wheel speed [m/s]
		(self.ref_vel_left, self.ref_vel_right) = self.dk.inverse(self.vel_lin, self.vel_ang)

		# convert to [ticks/update_interval]
		# self.ref_ticks_left = self.ref_vel_left*self.ticks_per_meter_left;
		# self.ref_ticks_right = self.ref_vel_right*self.ticks_per_meter_right;

	def keep_within (self, var, maximum):
		if var > maximum:
			var = maximum
		elif var < -maximum:
			var = -maximum
		return var
		
	def update_sim (self):
		if self.ref_vel_left > 0.0 and self.ref_vel_left < self.wheel_speed_minimum:
			self.ref_vel_left = 0.0
		elif self.ref_vel_left < 0.0 and self.ref_vel_left > -self.wheel_speed_minimum:
			self.ref_vel_left = 0.0

		if self.ref_vel_right > 0.0 and self.ref_vel_right < self.wheel_speed_minimum:
			self.ref_vel_right = 0.0
		elif self.ref_vel_right < 0.0 and self.ref_vel_right > -self.wheel_speed_minimum:
			self.ref_vel_right = 0.0

		if self.ref_vel_left != 0 or self.ref_vel_right != 0:
			#self.sim_vel_left = self.ref_vel_left + np.random.randn()* self.wheel_speed_variance
			#self.sim_vel_right = self.ref_vel_right + np.random.randn()* self.wheel_speed_variance

			self.wheel_speed_err_left += np.random.randn()*0.008
			self.wheel_speed_err_left = self.keep_within (self.wheel_speed_err_left, self.wheel_speed_error)

			self.wheel_speed_err_right += np.random.randn()*0.008
			self.wheel_speed_err_right = self.keep_within (self.wheel_speed_err_right, self.wheel_speed_error)

			#self.sim_vel_left = self.ref_vel_left + self.wheel_speed_err_left*self.ref_vel_left + np.random.randn()* self.wheel_speed_variance
			#self.sim_vel_right = self.ref_vel_right + self.wheel_speed_err_right*self.ref_vel_right + np.random.randn()* self.wheel_speed_variance

			self.sim_vel_left = self.ref_vel_left + self.wheel_speed_err_left + np.random.randn()* self.wheel_speed_variance
			self.sim_vel_right = self.ref_vel_right + self.wheel_speed_err_right + np.random.randn()* self.wheel_speed_variance

			(sim_vel_lin, sim_vel_ang) = self.dk.forward(self.sim_vel_left, self.sim_vel_right)
		else:
			self.sim_vel_left = 0.0
			self.sim_vel_right = 0.0
			sim_vel_lin = 0.0
			sim_vel_ang = 0.0

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
		if self.cmd_vel_tout_active:
			self.cmd_vel_tout_active = False
			rospy.loginfo (rospy.get_name() + ': Receiving cmd_vel')

		# save cmd_vel message
		self.cmd_vel_msgs.append([rospy.get_time(), msg.twist.linear.x, msg.twist.angular.z])

	def on_odom_reset_message(self, msg):
		self.pose = [msg.data[0], msg.data[1], msg.data[2]]
		#rospy.loginfo (rospy.get_name() + ': Received odometry reset')

	def publish_pose_message(self):
		self.pose_msg.header.stamp = rospy.Time.now()
		self.pose_msg.pose.pose.position.x = self.pose[0]
		self.pose_msg.pose.pose.position.y = self.pose[1]
		self.pose_msg.pose.pose.position.z = 0.0
		q = quaternion_from_euler (0, 0, self.pose[2])
		self.pose_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
		self.pose_pub.publish(self.pose_msg); # publish the pose message

	def publish_wheel_fb_messages(self):
		self.w_fb.header.stamp = rospy.Time.now()
		#left wheel
		self.w_fb.velocity = self.sim_vel_left 
		self.w_fb.velocity_setpoint = self.wl_fb_vel_set
		self.w_fb.thrust = 0.0
		self.w_fb_left_pub.publish (self.w_fb)
		# right wheel
		self.w_fb.velocity = self.sim_vel_right
		self.w_fb.velocity_setpoint = self.wr_fb_vel_set
		self.w_fb.thrust = 0.0
		self.w_fb_right_pub.publish (self.w_fb)

	def updater(self):
		while not rospy.is_shutdown():
			self.count += 1
			self.update_vel()
			self.update_sim()
			self.publish_pose_message()
			if self.pub_fb_interval != 0:
   				if self.count % self.pub_fb_interval == 0:
					self.publish_wheel_fb_messages()

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

