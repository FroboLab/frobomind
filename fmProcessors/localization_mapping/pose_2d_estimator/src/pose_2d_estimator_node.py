#!/usr/bin/env python
#*****************************************************************************
# FroboMind Pose 2D Estimator Node
# Copyright (c) 2013, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
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
#*****************************************************************************
"""
This file wraps the FroboMind Pose 2D Estimator library into a ROS node.
Most documentation of the library is in pose_2d_estimator.py

Revision
2013-05-16 KJ First version
"""
# ROS imports
import rospy,tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from msgs.msg import gpgga_tranmerc
from math import pi, sqrt, atan2
from pose_2d_estimator import pose_2d_preprocessor, pose_2d_ekf, yaw_ekf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Pose2DEstimatorNode():
	def __init__(self):
		self.update_rate = 20 # set update frequency [Hz]
		self.pose_msg = Odometry()
		self.quaternion = np.empty((4, ), dtype=np.float64) 
		self.odom_topic_received = False
		self.odometry_x_prev = 0.0
		self.odometry_y_prev = 0.0
		self.odometry_yaw_prev = 0.0

		# Get parameters
		self.pose_msg.header.frame_id = rospy.get_param("~frame_id", "base_link")
		self.pose_msg.child_frame_id = rospy.get_param("~child_frame_id", "odom")

		robot_max_velocity = rospy.get_param("~/robot_max_velocity", "1.0") # Robot maximum velocity [m/s]
		ekf_init_guess_easting = rospy.get_param("~ekf_initial_guess_easting", "0.0")
		ekf_init_guess_northing = rospy.get_param("~ekf_initial_guess_northing", "0.0")
		ekf_init_guess_yaw = rospy.get_param("~ekf_initial_guess_yaw", "0.0")

		self.odometry_var_dist = rospy.get_param("~odometry_distance_variance", "0.0")
		self.odometry_var_angle = rospy.get_param("~odometry_angular_variance", "0.0")

		# Get topic names
		self.odom_topic = rospy.get_param("~odom_sub",'/fmKnowledge/encoder_odom')
		self.imu_topic = rospy.get_param("~imu_sub",'/fmInformation/imu')
		self.gga_topic = rospy.get_param("~gga_sub",'/fmInformation/gpgga_tranmerc')
		self.pose_topic = rospy.get_param("~pose_pub",'/fmKnowledge/pose')

		# Setup subscription topic callbacks
		rospy.Subscriber(self.odom_topic, Odometry, self.on_odom_topic)
		rospy.Subscriber(self.imu_topic, Imu, self.on_imu_topic)
		rospy.Subscriber(self.gga_topic, gpgga_tranmerc, self.on_gga_topic)

		# setup publish topics
		self.pose_pub = rospy.Publisher(self.pose_topic, Odometry)
		self.br = tf.TransformBroadcaster()

		# initialize estimator (preprocessing)
		self.pp = pose_2d_preprocessor (robot_max_velocity)

		# initialize estimator (EKF)
		self.ekf = pose_2d_ekf()
		self.pose = [ekf_init_guess_easting, ekf_init_guess_northing, ekf_init_guess_yaw]
		self.ekf.set_initial_guess(self.pose)
		self.yawekf = yaw_ekf() # !!! TEMPORARY HACK

		# Call updater function
		rospy.loginfo(rospy.get_name() + ": Start")
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_odom_topic(self, msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		self.quaternion[0] = msg.pose.pose.orientation.x
		self.quaternion[1] = msg.pose.pose.orientation.y
		self.quaternion[2] = msg.pose.pose.orientation.z
		self.quaternion[3] = msg.pose.pose.orientation.w
		(roll,pitch,yaw) = euler_from_quaternion(self.quaternion)

		if self.odom_topic_received == True: # if we have received a first odom message

			# EKF system update (odometry)
			time_recv = msg.header.stamp.secs + msg.header.stamp.nsecs*1e-9
			delta_dist =  sqrt((x-self.odometry_x_prev)**2 + (y-self.odometry_y_prev)**2)
			delta_angle = self.angle_diff (yaw, self.odometry_yaw_prev)
			self.pp.odometry_new_feedback (time_recv, delta_dist, delta_angle)
			self.pose = self.ekf.system_update (delta_dist, self.odometry_var_dist, delta_angle, self.odometry_var_angle)
			self.pose[2] = self.yawekf.system_update (delta_angle, self.odometry_var_angle) # !!! TEMPORARY HACK

			# publish the estimated pose	
			self.publish_pose()

		# housekeeping
		self.odom_topic_received = True
		self.odometry_x_prev = x
		self.odometry_y_prev = y
		self.odometry_yaw_prev = yaw

	def on_imu_topic(self, msg):
		self.quaternion[0] = msg.orientation.x
		self.quaternion[1] = msg.orientation.y
		self.quaternion[2] = msg.orientation.z
		self.quaternion[3] = msg.orientation.w
		#rot = self.quat(0,1,0,pi/3)
		#self.quaternion = self.multiply(self.quaternion, rot)
		#rot = self.quat(1,0,0,pi/3)
		#self.quaternion = self.multiply(self.quaternion, rot)
		#rot = self.quat(0,0,1,(2*pi)/3)
		#self.quaternion = self.multiply(self.quaternion, rot)
		(roll,pitch,yaw) = euler_from_quaternion(self.quaternion)
		# self.pose[2] = yaw # NOT ACCURATE ENOUGH FROM THE VECTORNAV !!!

	def on_gga_topic(self, msg):
		if msg.fix > 0: # if satellite fix
			# GNSS data preprocessing
			time_recv = msg.time_recv.secs + msg.time_recv.nsecs*1e-9
			valid = self.pp.gnss_new_measurement (time_recv, msg.easting, msg.northing, msg.fix, msg.sat, msg.hdop)
			if valid == True: # if we have a valid position 
				var_pos = self.pp.gnss_estimate_variance_pos()

				# EKF measurement update (GNSS)
				self.pose = self.ekf.measurement_update_pos ([msg.easting, msg.northing], var_pos)
				(valid, yaw) = self.pp.estimate_absolute_orientation()
				if valid == True:
					var_yaw = 0.1
					self.pose[2] = self.yawekf.measurement_update (yaw, var_yaw) # !!! TEMPORARY HACK
	
	def publish_pose(self):
		self.pose_msg.header.stamp = rospy.Time.now()
		self.pose_msg.pose.pose.position.x = self.pose[0]
		self.pose_msg.pose.pose.position.y = self.pose[1]
		self.pose_msg.pose.pose.position.z = 0
		q = quaternion_from_euler (0, 0, self.pose[2])
		self.pose_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
		self.pose_pub.publish(self.pose_msg); # publish the pose message
		self.br.sendTransform((self.pose[0],self.pose[1],0), q, rospy.Time.now(), \
			self.pose_msg.header.frame_id, self.pose_msg.child_frame_id) # publish the transform message

	def updater(self):
		while not rospy.is_shutdown():
				
			# do updating stuff
			if self.odom_topic_received == False:
				self.publish_pose()

			# go back to sleep
			self.r.sleep()

	# return signed difference between new and old angle
	def angle_diff (self, angle_new, angle_old):
		diff = angle_new - angle_old
		while diff < -pi:
			diff += 2*pi
		while diff > pi:
			diff -= 2*pi
		return diff

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pose_2d_estimator_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = Pose2DEstimatorNode()
    except rospy.ROSInterruptException: pass

