#!/usr/bin/env python
#/****************************************************************************
# FroboScout Interface
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
2013-10-01 KJ First version
"""

# imports
import rospy
from msgs.msg import nmea, IntStamped
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
from differential_kinematics import differential_kinematics

class FroboScoutInterfaceNode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines
		self.update_rate = 20 # set update frequency [Hz]
		self.pid_update_interval = 20 # [ms]

		# locally defined parameters
		self.deadman_tout_duration = 0.2 # [s]
		self.cmd_vel_tout_duration = 0.2 # [s]

		# get parameters
		self.wheel_dist = rospy.get_param("/diff_steer_wheel_distance",'1.0') # [m]
		self.ticks_per_meter_left = rospy.get_param("/ticks_per_meter_left",'1000')
		self.ticks_per_meter_right = rospy.get_param("/ticks_per_meter_right",'1000')
		self.wheel_left_kp = rospy.get_param("~wheel_left_kp",'1.0')
		self.wheel_left_ki = rospy.get_param("~wheel_left_ki",'0.0')
		self.wheel_left_kd = rospy.get_param("~wheel_left_kd",'0.0')
		self.wheel_right_kp = rospy.get_param("~wheel_right_kp",'1.0')
		self.wheel_right_ki = rospy.get_param("~wheel_right_ki",'0.0')
		self.wheel_right_kd = rospy.get_param("~wheel_right_kd",'0.0')
		rospy.loginfo (rospy.get_name() + ': Differential wheel distance %.2f' % self.wheel_dist)
		rospy.loginfo (rospy.get_name() + ': Ticks per meter left %d' % self.ticks_per_meter_left)
		rospy.loginfo (rospy.get_name() + ': Ticks per meter right %d' % self.ticks_per_meter_right)

		# instantiate differntial kinemaics class
		self.dk = differential_kinematics(self.wheel_dist)

		# get topic names
		self.deadman_topic = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_sub",'/fmCommand/cmd_vel')
		self.enc_left_topic = rospy.get_param("~enc_left_pub",'/fmInformation/enc_left')
		self.enc_right_topic = rospy.get_param("~enc_right_pub",'/fmInformation/enc_right')
		self.wheel_left_sub_topic = rospy.get_param("~wheel_left_sub",'/fmData/wheel_left_nmea_in')
		self.wheel_left_pub_topic = rospy.get_param("~wheel_left_pub",'/fmSignal/wheel_left_nmea_out')
		self.wheel_right_sub_topic = rospy.get_param("~wheel_right_sub",'/fmData/wheel_right_nmea_in')
		self.wheel_right_pub_topic = rospy.get_param("~wheel_right_pub",'/fmSignal/wheel_right_nmea_out')

		# setup wheel ctrl topic publisher
		self.wheel_left_pub = rospy.Publisher(self.wheel_left_pub_topic, nmea)
		self.wheel_right_pub = rospy.Publisher(self.wheel_right_pub_topic, nmea)
		self.nmea = nmea()
		self.nmea.data.append('0')
		self.nmea.valid = True

		# setup encoder topic publisher
		self.state_left = 0
		self.enc_left = 0
		self.volt_left = 0
		self.state_right = 0
		self.enc_right = 0
		self.volt_right = 0
		self.enc_left_pub = rospy.Publisher(self.enc_left_topic, IntStamped)
		self.enc_right_pub = rospy.Publisher(self.enc_right_topic, IntStamped)
		self.intstamp = IntStamped()

		# setup subscription topic callbacks
		self.deadman_tout = 0
		rospy.Subscriber(self.deadman_topic, Bool, self.on_deadman_message)
		self.reset_vel() # set velocity to zero
		self.cmd_vel_tout = 0
		self.cmd_vel_tout_active = True
		rospy.Subscriber(self.cmd_vel_topic, TwistStamped, self.on_cmd_vel_message)
		rospy.Subscriber(self.wheel_left_sub_topic, nmea, self.on_wheel_left_message)
		rospy.Subscriber(self.wheel_right_sub_topic, nmea, self.on_wheel_right_message)

		# send parameters to wheel modules		
		self.publish_wheel_parameter_message()
		self.publish_wheel_parameter_message()
		self.publish_wheel_parameter_message() # necessary due to error in serial driver?

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_deadman_message(self, msg):
		if msg.data == True:
			self.deadman_tout = rospy.get_time() + self.deadman_tout_duration
		else:
			self.deadman_tout = 0

	def on_cmd_vel_message(self, msg):
		# update timeout
		self.cmd_vel_tout = rospy.get_time() + self.cmd_vel_tout_duration

		# retrieve linear and angular velocity from the message
		self.vel_lin = msg.twist.linear.x
		self.vel_ang = msg.twist.angular.z

		# calculate coresponding left and right wheel speed [m/s]
		(self.ref_vel_left, self.ref_vel_right) = self.dk.inverse (self.vel_lin, self.vel_ang)
		#print 'lin %.1f ang %.2f left %.1f right %.1f' % (self.vel_lin, self.vel_ang, self.ref_vel_left, self.ref_vel_right)

		# convert to [ticks/update_interval]
		self.ref_ticks_left = self.ref_vel_left*self.ticks_per_meter_left;
		self.ref_ticks_right = self.ref_vel_right*self.ticks_per_meter_right;

		if self.cmd_vel_tout_active:
			self.cmd_vel_tout_active = False
			rospy.loginfo (rospy.get_name() + ': Receiving cmd_vel')

	def on_wheel_left_message(self, msg):
		if msg.valid == True:
			if msg.type == 'PFSST':
				self.state_left = int(msg.data[0])
				self.enc_left += int(msg.data[1])
				self.volt_left = int(msg.data[2])
			elif msg.type == 'PFSHI':
				rospy.logwarn (rospy.get_name() + ': Left wheel module reset')
				self.publish_wheel_parameter_message()

	def on_wheel_right_message(self, msg):
		if msg.valid == True:
			if msg.type == 'PFSST':
				self.state_right = int(msg.data[0])
				self.enc_right += int(msg.data[1])
				self.volt_right = int(msg.data[2])
			elif msg.type == 'PFSHI':
				rospy.logwarn (rospy.get_name() + ': Right wheel module reset')
				self.publish_wheel_parameter_message()

	def publish_enc_messages(self):
		self.intstamp.header.stamp = rospy.Time.now()
		self.intstamp.data = self.enc_left
		#self.enc_left = 0
		self.enc_left_pub.publish (self.intstamp)
		self.intstamp.data = self.enc_right
		#self.enc_right = 0
		self.enc_right_pub.publish (self.intstamp)

	def publish_wheel_ctrl_messages(self):
		self.nmea.header.stamp = rospy.Time.now()
		self.nmea.type = 'PFSCT'
		self.nmea.length = 1
		if self.deadman_tout > rospy.get_time():
			self.nmea.data[0] = ('%d' % (self.ref_ticks_left + 0.5))
			self.wheel_left_pub.publish (self.nmea)
			self.nmea.data[0] = ('%d' % (self.ref_ticks_right + 0.5))
			self.wheel_right_pub.publish (self.nmea)
		else:
			self.nmea.data[0] = '0'
			self.wheel_left_pub.publish (self.nmea)
			self.wheel_right_pub.publish (self.nmea)

	def publish_wheel_parameter_message(self):
		nmea_pid = nmea()
		nmea_pid.header.stamp = rospy.Time.now()
		nmea_pid.type = 'PFSWP'
		nmea_pid.length = 5
		nmea_pid.data.append('1') # enable closed loop PID
		nmea_pid.data.append('%d' % self.pid_update_interval) 
		nmea_pid.data.append('%d' % self.wheel_left_kp) # Kp
		nmea_pid.data.append('%d' % self.wheel_left_ki) # Ki
		nmea_pid.data.append('%d' % self.wheel_left_kd) # Kd
		self.wheel_left_pub.publish (nmea_pid)
		nmea_pid.length = 5
		nmea_pid.data.append('1') # enable closed loop PID
		nmea_pid.data.append('%d' % self.wheel_right_kp) # Kp
		nmea_pid.data.append('%d' % self.wheel_right_ki) # Ki
		nmea_pid.data.append('%d' % self.wheel_right_kd) # Kd
		self.wheel_right_pub.publish (nmea_pid)

	def publish_wheel_parameter_open_loop_message(self):
		nmea_pid = nmea()
		nmea_pid.header.stamp = rospy.Time.now()
		nmea_pid.type = 'PFSWP'
		nmea_pid.length = 1
		nmea_pid.data.append('0') # enable open loop
		self.wheel_left_pub.publish (nmea_pid)
		self.wheel_right_pub.publish (nmea_pid)

	def reset_vel (self):
		self.vel_lin = 0.0 # [m/s]
		self.vel_ang = 0.0 # [rad/s]
		self.ref_vel_left = 0 # [m/s]
		self.ref_vel_right = 0 # [m/s]
		self.ref_ticks_left = 0 # [ticks/update_interval]
		self.ref_ticks_right = 0 # [ticks/update_interval]

	def updater(self):
		while not rospy.is_shutdown():
			self.publish_wheel_ctrl_messages()
			self.publish_enc_messages()
				
			# check for cmd_vel timeout
			if self.cmd_vel_tout_active == False:
				if rospy.get_time() > self.cmd_vel_tout:
					self.cmd_vel_tout_active = True
					self.reset_vel() # set velocity to zero
		 			rospy.logwarn (rospy.get_name() + ': cmd_vel timeout')

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('froboscout_interface_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = FroboScoutInterfaceNode()
    except rospy.ROSInterruptException:
		pass


