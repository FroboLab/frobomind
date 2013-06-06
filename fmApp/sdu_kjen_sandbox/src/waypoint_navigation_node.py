#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation
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
#****************************************************************************/
"""
2013-06-06 KJ First version
"""

# imports
lass Pose2DEstimatorNode():
	def __init__(self):
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
		self.r = rospy.Rate(10) # set updater frequency [Hz]
		self.updater()

	def on_odom_topic(self, msg):
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		#yaw = 0.0 # !!!!! OI OI NEEDS ATTENTION!!! MUST GET THIS FROM QUARTERNION
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
	
	def publish_cmd_vel(self):
		pass

	def updater(self):
		while not rospy.is_shutdown():
				
			# do updating stuff
			if self.odom_topic_received == False:
				self.publish_pose()

			# go back to sleep
			self.r.sleep()


# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('waypoint_navigation_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = WaypointNavigationNode()
    except rospy.ROSInterruptException: pass

