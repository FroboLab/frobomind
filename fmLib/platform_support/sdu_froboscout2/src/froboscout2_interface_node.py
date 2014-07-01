#!/usr/bin/env python
#/****************************************************************************
# FroboScout2 Interface
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
2014-06-40 KJ Migrated to FroboScout2 using wheel hub motors
"""

# imports
import rospy
from msgs.msg import serial, IntStamped, PropulsionModuleStatus, PropulsionModuleFeedback
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped
from differential_ifk_py.differential_kinematics import differential_kinematics
from time import sleep

class wheel_interface():
	def __init__(self, ident, w_pub_topic, w_sub_topic):
		self.STATE_NO_INIT = 0
		self.STATE_WAIT_RESP_SUB_RATE = 1
		self.STATE_WAIT_RESP_SUB_ADDR = 2
		self.STATE_WAIT_EN_PUB_SERVICE = 3
		self.STATE_WAIT_RUN = 4
		self.STATE_OK = 10

		self.state = self.STATE_NO_INIT

		self.ident = ident
		self.now = 0.0
		self.vel_set = 0.0
		self.encoder_ticks = 0
		self.encoder_latest_update = 0.0
		self.encoder_update_tout = 0.3 # [s]
		self.vel = 0.0

		self.ser_send = serial()
		self.ser_use_checksum = False
		self.ser_send_tout = 0.0
		self.ser_send_tout_duration = 1.0 # [s]

		# set up serial publisher
		self.w_pub = rospy.Publisher(w_pub_topic, serial)
		
		# set up serial subscriber
		rospy.Subscriber(w_sub_topic, serial, self.parse_serial_string)

	def set_velocity(self, vel):
		self.vel_set = vel

	def send_cfg_sub_rate(self):
		print self.ident+'Set subscription rate'
		self.publish_serial_string('WSR:%06x' % (20)) # Write Subscription Rate
		#self.publish_serial_string('RSI:01') # Write Subscription Rate
		self.state = self.STATE_WAIT_RESP_SUB_RATE

	def send_cfg_sub_addr(self):
		print self.ident+'Subscribing to encoder ticks, velocity and torque'
		self.publish_serial_string('WSA:0003010203') # Write Subscription Address (pos, vel, torque)
		self.state = self.STATE_WAIT_RESP_SUB_ADDR

	def send_cfg_en_pub_service(self):
		print self.ident+'Enabling publish service'
		self.publish_serial_string('EPS\n') # Enable Publish Service
		self.state = self.STATE_WAIT_EN_PUB_SERVICE

	def send_cfg_run(self):
		print self.ident+'Send run command'
		self.publish_serial_string('W:0400000001\n') # Run
		self.state = self.STATE_WAIT_RUN


	def hex_str_to_signed_int(self, s):
		i = int(s, 16)
		if i > 0x7FFFFFFF:
			i -= 0x100000000
		return i

	def signed_int_to_hex_str(self, i, bytes):
		if i < 0:
			i += 0x100000000
		if bytes == 4:
			s = '%08x' % i
		return s

	def parse_serial_string(self, msg):
		#print 'got data', msg.data
		if self.state == self.STATE_OK:
			if msg.data[:3] == '%P:':
				self.encoder_ticks = self.hex_str_to_signed_int(msg.data[7:15])
				vel = self.hex_str_to_signed_int(msg.data[15:23])
				torque = self.hex_str_to_signed_int(msg.data[23:31])
				#print 'got data', self.encoder_ticks, vel, torque
				self.encoder_latest_update = self.now
				pass
		elif self.state == self.STATE_WAIT_RESP_SUB_RATE:
			if msg.data == '#W':
				self.send_cfg_sub_addr()				
		elif self.state == self.STATE_WAIT_RESP_SUB_ADDR:
			if msg.data == '#W':
				self.send_cfg_en_pub_service()				
		elif self.state == self.STATE_WAIT_EN_PUB_SERVICE:
			if msg.data[:3] == '%P:':
				self.send_cfg_run()				
		elif self.state == self.STATE_WAIT_RUN:
			if msg.data == '#W':
				self.state = self.STATE_OK
				self.encoder_latest_update = self.now # fake 'encoder ok' so we don't get an immediate timeout			
	'''

nmeaCalcCS(s) == int(s[-2:], 16)

def nmeaCalcCS(s):
    cs = 0
    for c in s[1:-3]:
        cs = cs ^ ord(c)
    return cs


def nmea_checksum_ok(msg):
    msg = msg.strip('$')
    msg = msg.strip('\n')
    data,cs_recv = msg.split('*', 1)
    cs = reduce(operator.xor, (ord(s) for s in data), 0)
    return (cs == int(cs_recv,16))
	'''
		
	def publish_serial_string(self, s):
		self.ser_send.header.stamp = rospy.Time.now()
		if self.ser_use_checksum == True:
			cs = 0
			for c in s[:]:
				print c
				cs = cs ^ ord(c)
			self.ser_send.data = ('$%s*%02x\n' % (s, cs)).upper()
		else:
			self.ser_send.data = '#%s\n' % s

		#print 'sending', self.ser_send.data
		self.w_pub.publish (self.ser_send)
		self.ser_send_tout = self.now + self.ser_send_tout_duration

	def check_timeout(self):
		if self.state == self.STATE_OK:
			if self.now > self.encoder_latest_update + self.encoder_update_tout:
				print 'FPGA timeout'
				self.state = self.STATE_NO_INIT
		elif self.state == self.STATE_WAIT_RESP_SUB_RATE or self.state == self.STATE_WAIT_RESP_SUB_ADDR or self.state == self.STATE_WAIT_EN_PUB_SERVICE or self.state == self.STATE_WAIT_RUN:
			if self.now > self.ser_send_tout:
				self.state = self.STATE_NO_INIT

	def update(self):
		self.now = rospy.get_rostime().to_sec()
		self.check_timeout()
		if self.state == self.STATE_OK:
			pwm_set = self.vel_set * 10000
			vel = 'W:06%s\n' % self.signed_int_to_hex_str(pwm_set, 4)
			#print 'send velocity', vel
			self.publish_serial_string(vel) # HASTIGHED

		elif self.state == self.STATE_NO_INIT:
			self.send_cfg_sub_rate()

class FroboScout2InterfaceNode():
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

		# get topic names
		self.deadman_topic = rospy.get_param("~deadman_sub",'/fmCommand/deadman')
		self.cmd_vel_topic = rospy.get_param("~cmd_vel_sub",'/fmCommand/cmd_vel')
		self.enc_left_topic = rospy.get_param("~enc_left_pub",'/fmInformation/enc_left')
		self.enc_right_topic = rospy.get_param("~enc_right_pub",'/fmInformation/enc_right')
		self.w_fb_left_pub_topic = rospy.get_param("~wheel_feedback_left_pub",'/fmInformation/wheel_feedback_left')
		self.w_fb_right_pub_topic = rospy.get_param("~wheel_feedback_right_pub",'/fmInformation/wheel_feedback_right')

		self.w_stat_left_pub_topic = rospy.get_param("~wheel_status_left_pub",'/fmInformation/wheel_status_left')
		self.w_stat_right_pub_topic = rospy.get_param("~wheel_status_right_pub",'/fmInformation/wheel_status_right')
		wl_sub_topic = rospy.get_param("~wheel_left_sub",'/fmData/wheel_left_serial_in')
		wl_pub_topic = rospy.get_param("~wheel_left_pub",'/fmSignal/wheel_left_serial_out')
		wr_sub_topic = rospy.get_param("~wheel_right_sub",'/fmData/wheel_right_serial_in')
		wr_pub_topic = rospy.get_param("~wheel_right_pub",'/fmSignal/wheel_right_serial_out')

		# instantiate wheel interface classes
		self.wl = wheel_interface('L: ', wl_pub_topic, wl_sub_topic)
		self.wr = wheel_interface('R: ', wr_pub_topic, wr_sub_topic)

		# instantiate differntial kinematics class
		self.dk = differential_kinematics(self.w_dist)

		# setup encoder topic publisher
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

		self.vel_lin_desired = 0.0
		self.vel_ang_desired = 0.0
		self.vel_lin = 0.0
		self.vel_ang = 0.0

		rospy.Subscriber(self.cmd_vel_topic, TwistStamped, self.on_cmd_vel_message)

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
		if self.deadman_tout < rospy.get_time() or self.wl.state != self.wl.STATE_OK or self.wr.state != self.wl.STATE_OK:
			#if self.deadman_tout < rospy.get_time() or self.wr.state != self.wl.STATE_OK:
			self.vel_lin_desired = 0.0
			self.vel_ang_desired = 0.0

		self.vel_lin = self.accelerate_vel (self.vel_lin, self.vel_lin_desired, self.acc_lin_max_step)
		self.vel_ang = self.accelerate_vel (self.vel_ang, self.vel_ang_desired, self.acc_ang_max_step)

		# calculate coresponding left and right wheel speed [m/s]
		(self.ref_vel_left, self.ref_vel_right) = self.dk.inverse(self.vel_lin, self.vel_ang)
		self.wl.set_velocity (-self.ref_vel_left)  # !!!!!!!!!!!!!!! minus
		self.wr.set_velocity (self.ref_vel_right) 

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

	def publish_enc_messages(self):
		self.intstamp.header.stamp = rospy.Time.now()
		self.intstamp.data = self.wl.encoder_ticks
		self.enc_left_pub.publish (self.intstamp)
		self.intstamp.data = self.wl.encoder_ticks
		self.enc_right_pub.publish (self.intstamp)

	def publish_wheel_fb_messages(self):
		self.w_fb.header.stamp = rospy.Time.now()
		#left wheel
		self.w_fb.velocity = 0.0 
		self.w_fb.velocity_setpoint = 0.0
		self.w_fb.thrust = 0.0
		self.w_fb_left_pub.publish (self.w_fb)
		# right wheel
		self.w_fb.velocity = 0.0 
		self.w_fb.velocity_setpoint = 0.0
		self.w_fb.thrust = 0.0
		self.w_fb_right_pub.publish (self.w_fb)

	def publish_wheel_status_messages(self):
		self.w_stat.header.stamp = rospy.Time.now()
		#left wheel
		self.w_stat.voltage = 0.0	
		self.w_stat.current = 0.0
		self.w_stat.power =  0.0
		self.w_stat_left_pub.publish (self.w_stat)
		# right wheel
		self.w_stat.voltage = 0.0		
		self.w_stat.current = 0.0
		self.w_stat.power =  0.0
		self.w_stat_right_pub.publish (self.w_stat)

	def updater(self):
		while not rospy.is_shutdown():
			self.count += 1
			self.update_vel()
			self.wl.update()
			self.wr.update()
			self.publish_enc_messages()

			# wheel status
			if self.pub_status_interval != 0:
				if self.count % self.pub_status_interval == 0:
					self.publish_wheel_status_messages()
			
			# wheel feedback
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

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('froboscout2_interface_node')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = FroboScout2InterfaceNode()
    except rospy.ROSInterruptException:
		pass


