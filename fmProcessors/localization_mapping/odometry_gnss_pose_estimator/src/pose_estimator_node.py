#!/usr/bin/env python
#*****************************************************************************
# FroboMind Pose Estimator Node
# Copyright (c) 2013-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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
Most documentation of the library is in pose_estimator.py

Revision
2013-05-16 KJ First version
2014-03-18 KJ Various bug fixes and computation optimizations
2014-04-21 KJ Fixed EKF problems.
2014-06-19 KJ "driving forward" detection now accepts zero encoder updates.
"""
# ROS imports
import rospy,tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from msgs.msg import gpgga_tranmerc, IntArrayStamped
from math import pi, sqrt, atan2, asin
from pose_estimator import odometry_gnss_pose_preprocessor, odometry_pose_ekf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PoseEstimatorNode():
	def __init__(self):
		# defines
		self.pi2 = 2.0*pi

		self.SOL_ODO = 0
		self.SOL_SPS = 1
		self.SOL_DGPS = 2
		self.SOL_FLOAT = 3
		self.SOL_RTK = 4

		self.ERR_NONE = 0
		self.ERR_ODO_TOUT = -1
		self.ERR_IMU_TOUT = -2 
		self.ERR_GNSS_TOUT = -3
		self.ERR_ORIENTATION_TOUT = -4

		# initialization
		rospy.loginfo(rospy.get_name() + ": Start")
		self.update_rate = 5
		self.pose_msg = Odometry()
		self.pose_status_msg = IntArrayStamped()
		self.pose_status_msg.data.append (self.SOL_ODO)
		self.pose_status_msg.data.append (self.ERR_NONE)
		self.quaternion = np.empty((4, ), dtype=np.float64) 
		self.first_odom_topic_received = False
		self.odometry_x_prev = 0.0
		self.odometry_y_prev = 0.0
		self.odometry_yaw_prev = 0.0
		self.estimate_orientation_now = False
		self.first_absolute_pos_update = False
		self.first_absolute_yaw_update = False

		self.latest_odo_update = 0.0
		self.latest_gnss_update = 0.0
		self.latest_imu_update = 0.0
		self.latest_orientation_update = 0.0
	
		self.solution = self.SOL_ODO
		self.error = self.ERR_NONE

		# Get parameters
		self.pose_msg.header.frame_id = rospy.get_param("~frame_id", "base_link")
		self.pose_msg.child_frame_id = rospy.get_param("~child_frame_id", "odom")

		robot_max_velocity = float(rospy.get_param("~/robot_max_velocity", 1.0)) # Robot maximum velocity [m/s]
		self.solution_required = rospy.get_param("~solution_required", self.SOL_ODO)

		self.odometry_var_dist = rospy.get_param("~odometry_distance_variance", 0.000001)
		self.odometry_var_yaw = rospy.get_param("~odometry_angular_variance", 0.000001)
		self.gnss_var_yaw = rospy.get_param("~gnss_angular_variance", 0.0001)


		self.odo_timeout = rospy.get_param("~odometry_timeout", 0.5)
		self.gnss_timeout = rospy.get_param("~gnss_timeout", 2.0)
		self.imu_timeout = rospy.get_param("~imu_timeout", 0.5)
		self.absolute_orientation_timeout = rospy.get_param("~absolute_orientation_timeout", 30.0)

		# Get topic names
		self.odom_topic = rospy.get_param("~odom_sub",'/fmKnowledge/odometry')
		self.imu_topic = rospy.get_param("~imu_sub",'/fmInformation/imu')
		self.gga_topic = rospy.get_param("~gga_sub",'/fmInformation/gpgga_tranmerc')
		self.pose_topic = rospy.get_param("~pose_pub",'/fmKnowledge/pose')
		self.pose_status_topic = rospy.get_param("~pose_status_pub",'/fmKnowledge/pose_status')

		# Setup subscription topic callbacks
		rospy.Subscriber(self.odom_topic, Odometry, self.on_odom_topic)
		rospy.Subscriber(self.imu_topic, Imu, self.on_imu_topic)
		rospy.Subscriber(self.gga_topic, gpgga_tranmerc, self.on_gga_topic)

		# setup publish topics
		self.pose_pub = rospy.Publisher(self.pose_topic, Odometry)
		self.br = tf.TransformBroadcaster()
		self.pose_status_pub = rospy.Publisher(self.pose_status_topic, IntArrayStamped)

		# initialize estimator (preprocessing)
		self.pp = odometry_gnss_pose_preprocessor (robot_max_velocity)
		self.acc_roll = 0.0
		self.acc_pitch = 0.0

		# initialize EKF
		self.ekf = odometry_pose_ekf()
		self.pose = [0.0, 0.0, 0.0]
		self.ekf.initial_guess (self.pose, self.odometry_var_dist, self.odometry_var_yaw)

		# Call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_odom_topic(self, msg):
		self.latest_odo_update = rospy.get_time()
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		self.quaternion[0] = msg.pose.pose.orientation.x
		self.quaternion[1] = msg.pose.pose.orientation.y
		self.quaternion[2] = msg.pose.pose.orientation.z
		self.quaternion[3] = msg.pose.pose.orientation.w
		(roll,pitch,yaw) = euler_from_quaternion(self.quaternion)

		# driving forwards or backwards?
		if (msg.twist.twist.linear.x >= 0):
			forward = True
		else:
			forward = False

		if self.first_odom_topic_received == True: # if we have received a first odom message
			# EKF system update (odometry)
			delta_dist =  sqrt((x-self.odometry_x_prev)**2 + (y-self.odometry_y_prev)**2)
			delta_angle = self.angle_diff (yaw, self.odometry_yaw_prev)
			self.pp.odometry_new_data (self.latest_odo_update, delta_dist, delta_angle, forward)
			self.pose = self.ekf.system_update (delta_dist, self.odometry_var_dist, delta_angle, self.odometry_var_yaw)

			# publish the estimated pose	
			self.publish_pose()

		# housekeeping
		self.first_odom_topic_received = True
		self.odometry_x_prev = x
		self.odometry_y_prev = y
		self.odometry_yaw_prev = yaw

	def on_imu_topic(self, msg):
		self.latest_imu_update = rospy.get_time()
		self.pp.imu_new_data (self.latest_imu_update, msg.angular_velocity.z)

		# determine pitch and roll based on the accelerometers
		ax = msg.linear_acceleration.x
		ay = msg.linear_acceleration.y
		az = msg.linear_acceleration.z
		self.acc_pitch = atan2(-ay, sqrt(ax**2 + az**2))
		self.acc_roll = atan2(ax, az)

	def on_gga_topic(self, msg):
		self.latest_gnss_update = rospy.get_time()
		
		# evaluate the satellite solution
		if msg.fix == 0:
			sol = self.SOL_ODO
		elif msg.fix == 1:
			sol = self.SOL_SPS
		elif msg.fix == 2:
			sol  = self.SOL_DGPS
		elif msg.fix == 5:
			sol = self.SOL_FLOAT
		elif msg.fix == 4:
			sol = self.SOL_RTK

		if sol >= self.SOL_SPS and sol >= self.solution_required:
			self.solution = sol
		else:
			self.solution = self.SOL_ODO

		if self.solution >= self.SOL_SPS:
			# GNSS data preprocessing
			pos_valid = self.pp.gnss_new_data (self.latest_gnss_update, msg.easting, msg.northing, msg.fix, msg.sat, msg.hdop)
			if pos_valid == True: # if we have a valid position 
				if self.first_absolute_pos_update == False:
					self.ekf.initial_guess ([msg.easting, msg.northing, self.pose[2]], self.odometry_var_dist, pi)
					self.first_absolute_pos_update = True

				# orientation update
				yaw = self.pose[2]
				if self.estimate_orientation_now == True:
					self.estimate_orientation_now = False
					(yaw_valid, yaw_test) = self.pp.estimate_absolute_orientation()
					if yaw_valid == True:
						self.latest_orientation_update = self.latest_gnss_update
						yaw = yaw_test
						if self.first_absolute_yaw_update == False:
							self.ekf.initial_guess ([msg.easting, msg.northing, yaw], self.odometry_var_dist, self.odometry_var_yaw)
							self.first_absolute_yaw_update = True
							rospy.loginfo(rospy.get_name() + ': First absolute orientation update')

				# update EKF
				var_pos = self.pp.gnss_estimate_variance_pos()
				yaw_diff = self.angle_diff (yaw, self.pose[2])
				self.pose = self.ekf.measurement_update ([msg.easting, msg.northing, self.pose[2]+yaw_diff], var_pos, self.gnss_var_yaw)

	def publish_pose(self):
		if self.solution_required == self.SOL_ODO or (self.solution >= self.solution_required and self.first_absolute_yaw_update == True):
			self.pose_msg.header.stamp = rospy.Time.now()
			self.pose_msg.pose.pose.position.x = self.pose[0]
			self.pose_msg.pose.pose.position.y = self.pose[1]
			self.pose_msg.pose.pose.position.z = 0
			q = quaternion_from_euler (self.acc_roll, self.acc_pitch, self.pose[2])
			self.pose_msg.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
			self.pose_pub.publish(self.pose_msg); # publish the pose message
			self.br.sendTransform((self.pose[0],self.pose[1],0), q, rospy.Time.now(), \
				self.pose_msg.header.frame_id, self.pose_msg.child_frame_id) # publish the transform message

	def publish_pose_status(self):
		self.pose_status_msg.header.stamp = rospy.Time.now()

		# update solution status
		self.pose_status_msg.data[0] = self.solution

		# update error status
		time_now = rospy.get_time()
		err = self.ERR_NONE
		if self.latest_odo_update + self.odo_timeout < time_now:
			err = self.ERR_ODO_TOUT 
		elif self.latest_imu_update + self.imu_timeout < time_now:
			err = self.ERR_IMU_TOUT
		elif self.latest_gnss_update + self.gnss_timeout < time_now and self.solution_required > self.SOL_ODO:
			err = self.ERR_GNSS_TOUT
		elif self.latest_orientation_update + self.absolute_orientation_timeout < time_now  and self.solution_required > self.SOL_ODO:
			err = self.ERR_ORIENTATION_TOUT
		self.pose_status_msg.data[1] = err

		# publish the status
		self.pose_status_pub.publish (self.pose_status_msg)

	def updater(self):
		while not rospy.is_shutdown(): # updated at the rate defined by self.update_rate
			if self.first_odom_topic_received == False: # publish pose from here until odometry is received.
				self.publish_pose()
			self.publish_pose_status() # publish the current pose status
			self.estimate_orientation_now = True # request a new orientation update

			# go back to sleep
			self.r.sleep()

	# return signed difference between new and old angle
	def angle_diff (self, angle_new, angle_old):
		diff = angle_new - angle_old
		while diff < -pi:
			diff += self.pi2
		while diff > pi:
			diff -= self.pi2
		return diff

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('odometry_gnss_pose_estimator_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = PoseEstimatorNode()
    except rospy.ROSInterruptException: pass

