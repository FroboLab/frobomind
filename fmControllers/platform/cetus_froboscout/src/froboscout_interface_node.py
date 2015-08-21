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
2013-11-18 KJ Added support for publish wheel status
"""

# imports
import rospy
from msgs.msg import nmea, IntStamped, PropulsionModuleStatus, PropulsionModuleFeedback
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
from differential_ifk_py.differential_kinematics import differential_kinematics
from time import sleep

class FroboScoutInterfaceNode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines
		self.update_rate = 50 # set update frequency [Hz]
		self.pid_update_interval = 20 # [ms]
		self.w_STATE_ERR_NO_CONFIG = 3
		self.w_STATE_ERR = 3
		self.count = 0

		# locally defined parameters
		self.deadman_tout_duration = 0.2 # [s]
		self.cmd_vel_tout_duration = 0.2 # [s]
		self.w_tout_duration = 0.2 # [s]

		# variables
		self.w_voltage_conv = 5.0*(1800.0 + 700.0)/(700.0*1023.0) #(voltage divider 1800/700 ohm)
		self.supply_voltage_ok = False
		self.w_current_conv = 5.0/(1023.0*0.075) # according to simple-h controller datasheet

		self.wl_stat_curr = 0.0
		self.wl_stat_volt = 0.0
		self.wl_stat_power = 0.0
		self.wr_stat_curr = 0.0
		self.wr_stat_volt = 0.0
		self.wr_stat_power = 0.0

		# get parameters
		self.w_dist = rospy.get_param("/diff_steer_wheel_distance", 1.0) # [m]
		self.ticks_per_meter_left = rospy.get_param("/ticks_per_meter_left", 1000)
		self.ticks_per_meter_right = rospy.get_param("/ticks_per_meter_right", 1000)
		acc_lin_max = rospy.get_param("~max_linear_acceleration", 1.0) # [m/s^2]
		acc_ang_max = rospy.get_param("~max_angular_acceleration", 0.1) # [rad/s^2]
		self.acc_lin_max_step = acc_lin_max/(self.update_rate * 1.0)		
		self.acc_ang_max_step = acc_ang_max/(self.update_rate * 1.0)		
		self.wl_kp = rospy.get_param("~wheel_left_kp", 1.0)
		self.wl_ki = rospy.get_param("~wheel_left_ki", 0.0)
		self.wl_kd = rospy.get_param("~wheel_left_kd", 0.0)
		self.wl_max_integral_output = rospy.get_param("~wheel_left_max_integral_output",0.0)
		self.wr_kp = rospy.get_param("~wheel_right_kp", 1.0)
		self.wr_ki = rospy.get_param("~wheel_right_ki", 0.0)
		self.wr_kd = rospy.get_param("~wheel_right_kd", 0.0)
		self.wr_max_integral_output = rospy.get_param("~wheel_right_max_integral_output", 0.0)
		self.min_supply_voltage = rospy.get_param("~min_supply_voltage", 12.0)

		pub_status_rate = rospy.get_param("~publish_wheel_status_rate", 5)
		if pub_status_rate != 0:
			self.pub_status_interval = int(self.update_rate/pub_status_rate)
		else:
			self.pub_status_interval = 0

		pub_fb_rate = rospy.get_param("~publish_wheel_feedback_rate", 0)
		if pub_fb_rate != 0:
			self.pub_fb_interval = int(self.update_rate/pub_fb_rate)
		else:
			self.pub_fb_interval = 0

		show_volt_interval = rospy.get_param("~show_voltage_interval", 60)
		if show_volt_interval != 0:
			self.show_volt_interval = int(show_volt_interval) * self.update_rate
		else:
			self.show_volt_interval = 0

		#rospy.loginfo (rospy.get_name() + ': Differential wheel distance %.2f' % self.w_dist)
		#rospy.loginfo (rospy.get_name() + ': Ticks per meter left %d' % self.ticks_per_meter_left)
		#rospy.loginfo (rospy.get_name() + ': Ticks per meter right %d' % self.ticks_per_meter_right)

		# instantiate differntial kinemaics class
		self.dk = differential_kinematics(self.w_dist)

		# get topic names
		self.deadman_topic = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_sub",'/fmCommand/cmd_vel')
		self.enc_left_topic = rospy.get_param("~enc_left_pub",'/fmInformation/enc_left')
		self.enc_right_topic = rospy.get_param("~enc_right_pub",'/fmInformation/enc_right')
		self.w_fb_left_pub_topic = rospy.get_param("~wheel_feedback_left_pub",'/fmInformation/wheel_feedback_left')
		self.w_fb_right_pub_topic = rospy.get_param("~wheel_feedback_right_pub",'/fmInformation/wheel_feedback_right')

		self.w_stat_left_pub_topic = rospy.get_param("~wheel_status_left_pub",'/fmInformation/wheel_status_left')
		self.w_stat_right_pub_topic = rospy.get_param("~wheel_status_right_pub",'/fmInformation/wheel_status_right')
		self.wl_sub_topic = rospy.get_param("~wheel_left_sub",'/fmData/wheel_left_nmea_in')
		self.wl_pub_topic = rospy.get_param("~wheel_left_pub",'/fmSignal/wheel_left_nmea_out')
		self.wr_sub_topic = rospy.get_param("~wheel_right_sub",'/fmData/wheel_right_nmea_in')
		self.wr_pub_topic = rospy.get_param("~wheel_right_pub",'/fmSignal/wheel_right_nmea_out')

		# setup wheel ctrl topic publisher
		self.wl_pub = rospy.Publisher(self.wl_pub_topic, nmea)
		self.wr_pub = rospy.Publisher(self.wr_pub_topic, nmea)
		self.nmea = nmea()
		self.nmea.data.append('0')
		self.nmea.valid = True
		self.vel_lin_desired = 0.0
		self.vel_ang_desired = 0.0
		self.vel_lin = 0.0
		self.vel_ang = 0.0

		# setup encoder topic publisher
		self.wl_fb_state = 0
		self.wr_fb_state = 0
		self.enc_left = 0
		self.enc_right = 0
		self.enc_left_pub = rospy.Publisher(self.enc_left_topic, IntStamped)
		self.enc_right_pub = rospy.Publisher(self.enc_right_topic, IntStamped)
		self.intstamp = IntStamped()

		# setup wheel status topic publisher
		if self.pub_status_interval > 0:
			self.w_stat_left_pub = rospy.Publisher(self.w_stat_left_pub_topic, PropulsionModuleStatus)
			self.w_stat_right_pub = rospy.Publisher(self.w_stat_right_pub_topic, PropulsionModuleStatus)
			self.w_stat = PropulsionModuleStatus()

		# setup wheel feedback topic publisher
		if self.pub_fb_interval > 0:
			self.wl_fb_vel = 0.0
			self.wl_fb_vel_set = 0.0
			self.wl_fb_thrust = 0.0
			self.wr_fb_vel = 0.0
			self.wr_fb_vel_set = 0.0
			self.wr_fb_thrust = 0.0
			self.w_fb_left_pub = rospy.Publisher(self.w_fb_left_pub_topic, PropulsionModuleFeedback)
			self.w_fb_right_pub = rospy.Publisher(self.w_fb_right_pub_topic, PropulsionModuleFeedback)
			self.w_fb = PropulsionModuleFeedback()

		# setup subscription topic callbacks
		self.deadman_tout = 0
		rospy.Subscriber(self.deadman_topic, Bool, self.on_deadman_message)
		self.cmd_vel_tout = 0
		self.cmd_vel_tout_active = True
		self.wl_tout = 0
		self.wl_tout_active = True
		self.wr_tout = 0
		self.wr_tout_active = True
		rospy.Subscriber(self.cmd_vel_topic, TwistStamped, self.on_cmd_vel_message)
		rospy.Subscriber(self.wl_sub_topic, nmea, self.on_wheel_left_message)
		rospy.Subscriber(self.wr_sub_topic, nmea, self.on_wheel_right_message)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def check_voltage(self):
		if self.wl_stat_volt >= self.min_supply_voltage and self.wr_stat_volt >= self.min_supply_voltage: 
			self.supply_voltage_ok = True
		else:
			self.supply_voltage_ok = False

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
		if self.deadman_tout < rospy.get_time() or self.wl_fb_state >= self.w_STATE_ERR or self.wr_fb_state >= self.w_STATE_ERR:
			self.vel_lin_desired = 0.0
			self.vel_ang_desired = 0.0

		self.vel_lin = self.accelerate_vel (self.vel_lin, self.vel_lin_desired, self.acc_lin_max_step)
		self.vel_ang = self.accelerate_vel (self.vel_ang, self.vel_ang_desired, self.acc_ang_max_step)

		# calculate coresponding left and right wheel speed [m/s]
		(self.ref_vel_left, self.ref_vel_right) = self.dk.inverse(self.vel_lin, self.vel_ang)

		# convert to [ticks/s]
		self.ref_ticks_left = self.ref_vel_left*self.ticks_per_meter_left;
		self.ref_ticks_right = self.ref_vel_right*self.ticks_per_meter_right;

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

	def on_wheel_left_message(self, msg):
		if msg.valid == True:
			if msg.type == 'PFSST': # state, ticks, current, voltage, velocity_set, velocity, thrust, p, i, d 
				self.wl_tout = rospy.get_time() + self.w_tout_duration

				self.wl_fb_state = int(msg.data[0])
				self.enc_left += int(msg.data[1])
				self.wl_stat_curr = int(msg.data[2])*self.w_current_conv
				self.wl_stat_volt = int(msg.data[3])*self.w_voltage_conv
				if self.pub_fb_interval != 0:
					self.wl_fb_vel_set = int(msg.data[4]) / (1.0*self.ticks_per_meter_left)
					self.wl_fb_vel = int(msg.data[5]) / (1.0*self.ticks_per_meter_left)
					self.wl_fb_thrust = int(msg.data[6])
				if self.wl_fb_state == self.w_STATE_ERR_NO_CONFIG: # wheel module not configured since reset?
					self.publish_wheel_parameter_message()
			elif msg.type == 'PFSHI':
				rospy.logwarn (rospy.get_name() + ': Left wheel module reset')

			if self.wl_tout_active:
				self.wl_tout_active = False
				rospy.loginfo (rospy.get_name() + ': Receiving data from left wheel')
				self.publish_wheel_parameter_message()

	def on_wheel_right_message(self, msg):
		if msg.valid == True:
			if msg.type == 'PFSST':
				self.wr_tout = rospy.get_time() + self.w_tout_duration
				self.wr_fb_state = int(msg.data[0])
				self.enc_right += int(msg.data[1])
				self.wr_stat_curr = int(msg.data[2])*self.w_current_conv
				self.wr_stat_volt = int(msg.data[3])*self.w_voltage_conv
				if self.pub_fb_interval != 0:
					self.wr_fb_vel_set = int(msg.data[4]) / (1.0*self.ticks_per_meter_left)
					self.wr_fb_vel = int(msg.data[5]) / (1.0*self.ticks_per_meter_left)
					self.wr_fb_thrust = int(msg.data[6])
				if self.wr_fb_state == self.w_STATE_ERR_NO_CONFIG: # wheel module not configured since reset?
					self.publish_wheel_parameter_message()
			elif msg.type == 'PFSHI':
				rospy.logwarn (rospy.get_name() + ': Right wheel module reset')

			if self.wr_tout_active:
				self.wr_tout_active = False
				rospy.loginfo (rospy.get_name() + ': Receiving data from right wheel')
				self.publish_wheel_parameter_message()

	def publish_enc_messages(self):
		self.intstamp.header.stamp = rospy.Time.now()
		self.intstamp.data = self.enc_left
		#self.enc_left = 0
		self.enc_left_pub.publish (self.intstamp)
		self.intstamp.data = self.enc_right
		#self.enc_right = 0
		self.enc_right_pub.publish (self.intstamp)

	def publish_wheel_fb_messages(self):
		self.w_fb.header.stamp = rospy.Time.now()
		#left wheel
		self.w_fb.velocity = self.wl_fb_vel 
		self.w_fb.velocity_setpoint = self.wl_fb_vel_set
		self.w_fb.thrust = self.wl_fb_thrust
		self.w_fb_left_pub.publish (self.w_fb)
		# right wheel
		self.w_fb.velocity = self.wr_fb_vel 
		self.w_fb.velocity_setpoint = self.wr_fb_vel_set
		self.w_fb.thrust = self.wr_fb_thrust
		self.w_fb_right_pub.publish (self.w_fb)

	def publish_wheel_status_messages(self):
		self.w_stat.header.stamp = rospy.Time.now()
		#left wheel
		self.w_stat.voltage = self.wl_stat_volt		
		self.w_stat.current = self.wl_stat_curr
		self.w_stat.power =  self.wl_stat_volt*self.wl_stat_curr
		self.w_stat_left_pub.publish (self.w_stat)
		# right wheel
		self.w_stat.voltage = self.wr_stat_volt		
		self.w_stat.current = self.wr_stat_curr
		self.w_stat.power =  self.wl_stat_volt*self.wl_stat_curr
		self.w_stat_right_pub.publish (self.w_stat)

	def publish_wheel_ctrl_messages(self):
		self.nmea.header.stamp = rospy.Time.now()
		self.nmea.type = 'PFSCT'
		self.nmea.length = 1
		if self.deadman_tout > rospy.get_time() and self.wl_tout_active == False and self.wr_tout_active == False:
			self.nmea.data[0] = ('%d' % (self.ref_ticks_left + 0.5))
			self.wl_pub.publish (self.nmea)
			self.nmea.data[0] = ('%d' % (self.ref_ticks_right + 0.5))
			self.wr_pub.publish (self.nmea)
		else:
			self.nmea.data[0] = '0'
			self.wl_pub.publish (self.nmea)
			self.wr_pub.publish (self.nmea)

	def publish_wheel_parameter_message(self):
		# configure minimum supply voltage
		nmea_sys_par = nmea()
		nmea_sys_par.header.stamp = rospy.Time.now()
		nmea_sys_par.type = 'PFSSP'
		nmea_sys_par.length = 1
		nmea_sys_par.data.append('%d' % (self.min_supply_voltage/self.w_voltage_conv + 0.5)) 
		self.wl_pub.publish (nmea_sys_par)
		self.wr_pub.publish (nmea_sys_par)
		#print self.min_supply_voltage, self.min_supply_voltage/self.w_voltage_conv, nmea_sys_par.data[0]

		# configure PID parameters
		nmea_pid = nmea()
		nmea_pid.header.stamp = rospy.Time.now()
		nmea_pid.type = 'PFSWP'
		nmea_pid.length = 6
		nmea_pid.data.append('1') # enable closed loop PID
		nmea_pid.data.append('%d' % self.pid_update_interval) 
		nmea_pid.data.append('%d' % self.wl_kp) # Kp
		nmea_pid.data.append('%d' % self.wl_ki) # Ki
		nmea_pid.data.append('%d' % self.wl_kd) # Kd
		nmea_pid.data.append('%d' % self.wl_max_integral_output) # Maximum output (integrator anti-windup)
		self.wl_pub.publish (nmea_pid)
		nmea_pid.data = []
		nmea_pid.data.append('1') # enable closed loop PID
		nmea_pid.data.append('%d' % self.pid_update_interval) 
		nmea_pid.data.append('%d' % self.wr_kp) # Kp
		nmea_pid.data.append('%d' % self.wr_ki) # Ki
		nmea_pid.data.append('%d' % self.wr_kd) # Kd
		nmea_pid.data.append('%d' % self.wr_max_integral_output) # Maximum output (integrator anti-windup)
		self.wr_pub.publish (nmea_pid)


	def publish_wheel_parameter_open_loop_message(self):
		nmea_pid = nmea()
		nmea_pid.header.stamp = rospy.Time.now()
		nmea_pid.type = 'PFSWP'
		nmea_pid.length = 1
		nmea_pid.data.append('0') # enable open loop
		self.wl_pub.publish (nmea_pid)
		self.wr_pub.publish (nmea_pid)

	def updater(self):
		while not rospy.is_shutdown():
			self.count += 1
			self.update_vel()
			self.publish_wheel_ctrl_messages()
			self.publish_enc_messages()
			if self.count % 5 == 0:
				self.check_voltage()

			# wheel status
			if self.pub_status_interval != 0:
				if self.count % self.pub_status_interval == 0:
					self.publish_wheel_status_messages()
			
			# wheel feedback
			if self.pub_fb_interval != 0:
   				if self.count % self.pub_fb_interval == 0:
					self.publish_wheel_fb_messages()

			# show voltage
			if self.show_volt_interval != 0:
   				if self.count % self.show_volt_interval == 0:
					s = rospy.get_name() + ': Voltage left %.1f right %.1f' % (self.wl_stat_volt, self.wr_stat_volt)
					if self.supply_voltage_ok:
			 			rospy.loginfo(s)
					else:
						rospy.logwarn(s)					

			# check for timeouts
			if self.cmd_vel_tout_active == False:
				if rospy.get_time() > self.cmd_vel_tout:
					self.cmd_vel_tout_active = True
					self.vel_lin_desired = 0.0
					self.vel_ang_desired = 0.0
		 			rospy.logwarn (rospy.get_name() + ': cmd_vel timeout')

			if self.wl_tout_active == False:
				if rospy.get_time() > self.wl_tout:
					self.wl_tout_active = True
		 			rospy.logwarn (rospy.get_name() + ': Left wheel data timeout')

			if self.wr_tout_active == False:
				if rospy.get_time() > self.wr_tout:
					self.wr_tout_active = True
		 			rospy.logwarn (rospy.get_name() + ': Right wheel data timeout')

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


