#!/usr/bin/env python
#/****************************************************************************
# Frobobit V2 Interface
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
#****************************************************************************/
"""
2013-10-01 KJ First version
2013-11-18 KJ Added support for publish wheel status
2014-04-17 KJ Migrated from Cetus FroboScout to SDU Frobit V2 Interface
2014-05-04 KJ Added seperate PID parameters for driving and turning about own
              center.
2014-08-21 KJ Added supply_voltage_scale_factor parameter to support the
              FroboMind Controller
"""

# imports
import rospy
from msgs.msg import nmea, IntStamped, FloatArrayStamped, PropulsionModuleStatus, PropulsionModuleFeedback
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
from differential_ifk_py.differential_kinematics import differential_kinematics
#from time import sleep

class FrobitInterfaceNode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines
		self.update_rate = 50 # set update frequency [Hz]
		self.pid_update_interval = 50 # [ms]
		self.pfbst_update_interval = 20 # [ms]
		self.STATE_OK = 1
		self.STATE_WARN_NMEA_CS = 2
		self.STATE_ERR_NO_CONFIG = 3
		self.STATE_ERR_WATCHDOG = 4
		self.STATE_ERR_LOWBAT = 5
		self.STATE_ERR = 3
		self.count = 0
		self.MOTION_DRIVE = 0
		self.MOTION_TURN = 1
		self.motion = self.MOTION_DRIVE

		# locally defined parameters
		self.deadman_tout_duration = 0.2 # [s]
		self.cmd_vel_tout_duration = 0.2 # [s]
		self.frobit_tout_duration = 0.2 # [s]

		# variables
		self.frobit_state = 0
		self.frobit_state_prev = -1
		self.robocard_voltage_divider = 0.03747 # 5.0*(22000 + 3300)/(3300*1023) because of 22000/3300 ohm v-divider
		self.frobit_voltage_ok = False
		self.frobit_volt = 0.0
		self.send_cfg_tout = 0.0

		# get parameters
		self.w_dist = rospy.get_param("/diff_steer_wheel_distance", 1.0) # [m]
		self.ticks_per_meter_left = rospy.get_param("/ticks_per_meter_left", 1000)
		self.ticks_per_meter_right = rospy.get_param("/ticks_per_meter_right", 1000)
		self.castor_front = rospy.get_param("~castor_front", "false")
		acc_lin_max = rospy.get_param("~max_linear_acceleration", 1.0) # [m/s^2]
		acc_ang_max = rospy.get_param("~max_angular_acceleration", 0.1) # [rad/s^2]
		self.acc_lin_max_step = acc_lin_max/(self.update_rate * 1.0)		
		self.acc_ang_max_step = acc_ang_max/(self.update_rate * 1.0)		
		self.w_drive_ff = rospy.get_param("~wheel_drive_feed_forward", 1.0)
		self.w_drive_kp = rospy.get_param("~wheel_drive_kp", 1.0)
		self.w_drive_ki = rospy.get_param("~wheel_drive_ki", 0.0)
		self.w_drive_kd = rospy.get_param("~wheel_drive_kd", 0.0)
		self.w_drive_max_int_output = rospy.get_param("~wheel_drive_max_integral_output", 0.0)
		self.w_turn_ff = rospy.get_param("~wheel_turn_feed_forward", 1.0)
		self.w_turn_kp = rospy.get_param("~wheel_turn_kp", 1.0)
		self.w_turn_ki = rospy.get_param("~wheel_turn_ki", 0.0)
		self.w_turn_kd = rospy.get_param("~wheel_turn_kd", 0.0)
		self.w_turn_max_int_output = rospy.get_param("~wheel_turn_max_integral_output", 0.0)
		self.supply_voltage_scale_factor = rospy.get_param("~supply_voltage_scale_factor", self.robocard_voltage_divider)
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

		pub_pid_rate = rospy.get_param("~publish_wheel_pid_rate", 0)
		if pub_pid_rate != 0:
			self.pub_pid_interval = int(self.update_rate/pub_pid_rate)
		else:
			self.pub_pid_interval = 0

		show_volt_interval = rospy.get_param("~show_voltage_interval", 60)
		if show_volt_interval != 0:
			self.show_volt_interval = int(show_volt_interval) * self.update_rate
		else:
			self.show_volt_interval = 0

		# instantiate differntial kinemaics class
		self.dk = differential_kinematics(self.w_dist)

		# get topic names
		self.deadman_topic = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_sub",'/fmCommand/cmd_vel')
		self.enc_l_topic = rospy.get_param("~enc_left_pub",'/fmInformation/enc_left')
		self.enc_r_topic = rospy.get_param("~enc_right_pub",'/fmInformation/enc_right')
		self.w_fb_left_pub_topic = rospy.get_param("~wheel_feedback_left_pub",'/fmInformation/wheel_feedback_left')
		self.w_fb_right_pub_topic = rospy.get_param("~wheel_feedback_right_pub",'/fmInformation/wheel_feedback_right')
		self.w_stat_left_pub_topic = rospy.get_param("~wheel_status_left_pub",'/fmInformation/wheel_status_left')
		self.w_stat_right_pub_topic = rospy.get_param("~wheel_status_right_pub",'/fmInformation/wheel_status_right')
		self.w_pid_left_pub_topic = rospy.get_param("~wheel_pid_left_pub",'/fmInformation/wheel_pid_left')
		self.w_pid_right_pub_topic = rospy.get_param("~wheel_pid_right_pub",'/fmInformation/wheel_pid_right')
		self.frobit_sub_topic = rospy.get_param("~nmea_from_frobit_sub",'/fmSignal/nmea_from_frobit')
		self.frobit_pub_topic = rospy.get_param("~nmea_to_frobit_pub",'/fmSignal/nmea_to_frobit')

		# setup frobit NMEA topic publisher
		self.frobit_pub = rospy.Publisher(self.frobit_pub_topic, nmea)

		# setup NMEA $PFBCT topic publisher
		self.nmea_pfbct = nmea()
		self.nmea_pfbct.type = 'PFBCT'
		self.nmea_pfbct.length = 2
		self.nmea_pfbct.valid = True
		self.nmea_pfbct.data.append('0')
		self.nmea_pfbct.data.append('0')
		self.vel_lin_desired = 0.0
		self.vel_ang_desired = 0.0
		self.vel_lin = 0.0
		self.vel_ang = 0.0

		# setup encoder topic publisher
		self.w_fb_state = 0
		self.enc_l = 0
		self.enc_r = 0
		self.enc_l_buf = [0]*5
		self.enc_r_buf = [0]*5

		self.enc_l_pub = rospy.Publisher(self.enc_l_topic, IntStamped)
		self.enc_r_pub = rospy.Publisher(self.enc_r_topic, IntStamped)
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
			self.tick_to_vel_factor = 1000.0/(1.0*self.ticks_per_meter_left*self.pfbst_update_interval)

		# setup wheel pid topic publisher
		if self.pub_pid_interval > 0:
			self.wl_pid_err = 0.0
			self.wl_pid_out = 0.0
			self.wl_pid_p = 0.0
			self.wl_pid_i = 0.0
			self.wl_pid_d = 0.0
			self.wr_pid_err = 0.0
			self.wr_pid_out = 0.0
			self.wr_pid_p = 0.0
			self.wr_pid_i = 0.0
			self.wr_pid_d = 0.0
			self.w_pid_left_pub = rospy.Publisher(self.w_pid_left_pub_topic, FloatArrayStamped)
			self.w_pid_right_pub = rospy.Publisher(self.w_pid_right_pub_topic, FloatArrayStamped)
			self.w_pid = FloatArrayStamped()
			self.nmea_pid = nmea()
			self.nmea_pid.type = 'PFBWP'
			self.nmea_pid.length = 7
			self.nmea_pid.valid = True
			self.nmea_pid.data.append('1') # enable closed loop PID
			self.nmea_pid.data.append(str(self.pid_update_interval)) 
			self.nmea_pid.data.append('0') 
			self.nmea_pid.data.append('0') 
			self.nmea_pid.data.append('0') 
			self.nmea_pid.data.append('0') 
			self.nmea_pid.data.append('0') 

		# setup subscription topic callbacks
		self.deadman_tout = 0
		rospy.Subscriber(self.deadman_topic, Bool, self.on_deadman_message)
		self.cmd_vel_tout = 0
		self.cmd_vel_tout_active = True
		self.frobit_tout = 0
		self.frobit_tout_active = True
		rospy.Subscriber(self.cmd_vel_topic, TwistStamped, self.on_cmd_vel_message)
		rospy.Subscriber(self.frobit_sub_topic, nmea, self.on_nmea_from_frobit)

		# call updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def check_voltage(self):
		if self.frobit_volt >= self.min_supply_voltage: 
			self.frobit_voltage_ok = True
		else:
			self.frobit_voltage_ok = False

	def accelerate_vel (self, vel_actual, vel_desired, max_step):
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
		if self.deadman_tout < rospy.get_time() or self.frobit_state >= self.STATE_ERR:
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

	def estimate_vel (self): # weighted average 
		lv =  (3*self.enc_l_buf[-1] + 2*self.enc_l_buf[-2] + 1*self.enc_l_buf[-3])/6.0 * self.tick_to_vel_factor
		rv =  (3*self.enc_r_buf[-1] + 2*self.enc_r_buf[-2] + 1*self.enc_r_buf[-3])/6.0 * self.tick_to_vel_factor
		return (lv, rv)

	def on_nmea_from_frobit(self, msg):
		if msg.valid == True:
			if msg.type == 'PFBST': # state, ticks, current, voltage, velocity_set, velocity, thrust, p, i, d 
				self.frobit_tout = rospy.get_time() + self.frobit_tout_duration

				self.frobit_state = int(msg.data[0])
				if self.frobit_tout_active:
					self.frobit_tout_active = False
					rospy.loginfo (rospy.get_name() + ': Receiving status from Frobit')				

				if self.frobit_state != self.frobit_state_prev:
					self.frobit_state_prev = self.frobit_state
					if self.frobit_state == self.STATE_OK:
						self.show_voltage()
						rospy.loginfo (rospy.get_name() + ': Frobit says: All systems go!')
					elif self.frobit_state == self.STATE_WARN_NMEA_CS:
						rospy.logwarn (rospy.get_name() + ': Frobit says: Erroneous NMEA message received.')
					elif self.frobit_state == self.STATE_ERR_NO_CONFIG:
						rospy.logwarn (rospy.get_name() + ': Frobit says: Waiting for configuration')
					elif self.frobit_state == self.STATE_ERR_WATCHDOG:
						rospy.logwarn (rospy.get_name() + ': Frobit says: Not receiving status messages')
					elif self.frobit_state == self.STATE_ERR_LOWBAT:
						rospy.logwarn (rospy.get_name() + ': Frobit says: Low battery')

				el =  int(msg.data[1]);
				self.enc_l += el
				self.enc_l_buf.append (el)
				del self.enc_l_buf[0]
				er =  int(msg.data[2]);
				self.enc_r += er
				self.enc_r_buf.append (er)
				del self.enc_r_buf[0]

				self.frobit_volt = int(msg.data[3])*self.supply_voltage_scale_factor
				if self.pub_fb_interval != 0:
					self.wl_fb_thrust = int(msg.data[4])
					self.wr_fb_thrust = int(msg.data[5])

				if self.pub_pid_interval != 0:
					self.wl_pid_err = int(msg.data[6])
					self.wl_pid_out = int(msg.data[7])
					self.wl_pid_p = int(msg.data[8])
					self.wl_pid_i = int(msg.data[9])
					self.wl_pid_d = int(msg.data[10])
					self.wr_pid_err = int(msg.data[11])
					self.wr_pid_out = int(msg.data[12])
					self.wr_pid_p = int(msg.data[13])
					self.wr_pid_i = int(msg.data[14])
					self.wr_pid_d = int(msg.data[15])

				if self.frobit_state == self.STATE_ERR_NO_CONFIG: # wheel module not configured since reset?
					self.publish_configuration_messages()
			elif msg.type == 'PFBHI':
				if msg.data[0] == '1':
					hardware_type = 'RoboCard'
				elif msg.data[0] == '2':
					hardware_type = 'FroboMind Controller'
				else:
					hardware_type = 'Unknown hardware'

				if msg.length >= 4:
					software_ver = msg.data[1] + '.' + msg.data[2]

					if msg.data[3] == '0':
						reset_cause = 'Power on reset'
					elif msg.data[3] == '1':
						reset_cause = 'Reset'
					elif msg.data[3] == '2':
						reset_cause = 'Brown-out reset'
					elif msg.data[3] == '3':
						reset_cause = 'Watchdog reset'
					elif msg.data[3] == '4':
						reset_cause = 'JTAG reset'
					else:
						reset_cause = 'Unknown reset'
				else:
					reset_cause = 'Unknown reset'
					software_ver = msg.data[1]

				rospy.logwarn (rospy.get_name() + ': Frobit says: %s' % (reset_cause))
				rospy.loginfo (rospy.get_name() + ': Frobit says: %s with firmware %s detected' % (hardware_type, software_ver))

	def publish_enc_messages(self):
		self.intstamp.header.stamp = rospy.Time.now()
		self.intstamp.data = self.enc_l
		#self.enc_l = 0
		self.enc_l_pub.publish (self.intstamp)
		self.intstamp.data = self.enc_r
		#self.enc_r = 0
		self.enc_r_pub.publish (self.intstamp)

	def publish_wheel_fb_messages(self):
		self.w_fb.header.stamp = rospy.Time.now()

		(vl, vr) = self.estimate_vel ()
		#left wheel
		self.w_fb.velocity = vl
		self.w_fb.velocity_setpoint = self.ref_vel_left
		self.w_fb.thrust = self.wl_fb_thrust
		self.w_fb_left_pub.publish (self.w_fb)
		# right wheel
		self.w_fb.velocity = vr
		self.w_fb.velocity_setpoint = self.ref_vel_right
		self.w_fb.thrust = self.wr_fb_thrust
		self.w_fb_right_pub.publish (self.w_fb)

	def publish_wheel_status_messages(self):
		self.w_stat.header.stamp = rospy.Time.now()
		#left wheel
		self.w_stat.voltage = self.frobit_volt		
		self.w_stat.current = 0.0
		self.w_stat.power =  0.0
		self.w_stat_left_pub.publish (self.w_stat)
		# right wheel
		self.w_stat.voltage = self.frobit_volt		
		self.w_stat.current = 0.0
		self.w_stat.power =  0.0
		self.w_stat_right_pub.publish (self.w_stat)

	def publish_wheel_pid_messages(self):
		self.w_pid.header.stamp = rospy.Time.now()
		#left wheel
		self.w_pid.data = [self.wl_pid_err, self.wl_pid_out, self.wl_pid_p, self.wl_pid_i, self.wl_pid_d]
		self.w_pid_left_pub.publish (self.w_pid)
		# right wheel
		self.w_pid.data = [self.wr_pid_err, self.wr_pid_out, self.wr_pid_p, self.wr_pid_i, self.wr_pid_d]
		self.w_pid_right_pub.publish (self.w_pid)

	def publish_wheel_ctrl_messages(self):
		if (self.ref_ticks_left < 0 and self.ref_ticks_right > 0) \
			or (self.ref_ticks_left > 0 and self.ref_ticks_right < 0):
			if self.motion == self.MOTION_DRIVE:
				self.publish_pid_turn_param_message()
				self.motion = self.MOTION_TURN
		if (self.ref_ticks_left < 0 and self.ref_ticks_right < 0) \
			or (self.ref_ticks_left > 0 and self.ref_ticks_right > 0):
			if self.motion == self.MOTION_TURN:
				self.publish_pid_drive_param_message()
				self.motion = self.MOTION_DRIVE

		self.nmea_pfbct.header.stamp = rospy.Time.now()
		if self.deadman_tout > rospy.get_time() and self.frobit_tout_active == False:
			self.nmea_pfbct.data[0] = ('%d' % (self.ref_ticks_left))
			self.nmea_pfbct.data[1] = ('%d' % (self.ref_ticks_right))
		else:
			self.nmea_pfbct.data[0] = '0'
			self.nmea_pfbct.data[1] = '0'
		self.frobit_pub.publish (self.nmea_pfbct)

	def publish_configuration_messages(self):
		if rospy.get_time() > self.send_cfg_tout:
			self.send_cfg_tout = rospy.get_time() + 1.0 # 1 second timeout

			# configure minimum supply voltage
			nmea_sys_par = nmea()
			nmea_sys_par.header.stamp = rospy.Time.now()
			nmea_sys_par.type = 'PFBSP'
			nmea_sys_par.length = 1
			nmea_sys_par.data.append('%d' % (self.min_supply_voltage/self.supply_voltage_scale_factor + 0.5)) 
			self.frobit_pub.publish (nmea_sys_par)
			self.publish_pid_drive_param_message()
			rospy.loginfo (rospy.get_name() + ': Sending configuration to Frobit')

	def publish_pid_drive_param_message(self):
			self.nmea_pid.header.stamp = rospy.Time.now()
			self.nmea_pid.data[2] = str(self.w_drive_ff) # feed forward
			self.nmea_pid.data[3] = str(self.w_drive_kp) # Kp
			self.nmea_pid.data[4] = str(self.w_drive_ki) # Ki
			self.nmea_pid.data[5] = str(self.w_drive_kd) # Kd
			self.nmea_pid.data[6] = str(self.w_drive_max_int_output) # anti-windup
			self.frobit_pub.publish (self.nmea_pid)

	def publish_pid_turn_param_message(self):
			self.nmea_pid.header.stamp = rospy.Time.now()
			self.nmea_pid.data[2] = str(self.w_turn_ff) # feed forward
			self.nmea_pid.data[3] = str(self.w_turn_kp) # Kp
			self.nmea_pid.data[4] = str(self.w_turn_ki) # Ki
			self.nmea_pid.data[5] = str(self.w_turn_kd) # Kd
			self.nmea_pid.data[6] = str(self.w_turn_max_int_output) # anti-windup
			self.frobit_pub.publish (self.nmea_pid)

	def publish_wheel_parameter_open_loop_message(self):
		nmea_pid = nmea()
		nmea_pid.header.stamp = rospy.Time.now()
		nmea_pid.type = 'PFBWP'
		nmea_pid.length = 1
		nmea_pid.data.append('0') # enable open loop
		self.frobit_pub.publish (nmea_pid)

	def show_voltage(self):
		s = rospy.get_name() + ': Frobit says: Battery %.1f Volt' % (self.frobit_volt)
		if self.frobit_voltage_ok:
 			rospy.loginfo(s)
		else:
			rospy.logwarn(s)					

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

			# wheel pid
			if self.pub_pid_interval != 0:
   				if self.count % self.pub_pid_interval == 0:
					self.publish_wheel_pid_messages()

			# show voltage
			if self.show_volt_interval != 0:
   				if self.count % self.show_volt_interval == 0:
					self.show_voltage()

			# check for timeouts
			if self.cmd_vel_tout_active == False:
				if rospy.get_time() > self.cmd_vel_tout:
					self.cmd_vel_tout_active = True
					self.vel_lin_desired = 0.0
					self.vel_ang_desired = 0.0
		 			rospy.logwarn (rospy.get_name() + ': cmd_vel timeout')

			if self.frobit_tout_active == False:
				if rospy.get_time() > self.frobit_tout:
					self.frobit_tout_active = True
		 			rospy.logwarn (rospy.get_name() + ': Not receiving data from Frobit')

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Frobit_interface_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = FrobitInterfaceNode()
    except rospy.ROSInterruptException:
		pass


