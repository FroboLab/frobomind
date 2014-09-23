#!/usr/bin/env python
#/****************************************************************************
# FroboMind route plan socket daemon
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

2014-09-17 KJ First version
"""
from navigation_globals import *
import rospy
from msgs.msg import StringArrayStamped, RoutePt, waypoint_navigation_status
from socket import *

class socketd():
	def  __init__(self, socket_addr, socket_port, socket_max_idle, socket_password, debug):
		self.MSG_TYPE_INFO = 0
		self.MSG_TYPE_WARNING = 1
		self.MSG_TYPE_ERROR = 2
		self.max_idle = socket_max_idle
		self.socket_password = socket_password
		if socket_password == '':
			self.socket_password_received = True
		else:	
			self.socket_password_received = False
		self.socket_timeout = 0.0
		self.socket_packets = []
		self.debug = debug
		self.tcpSerSock = socket(AF_INET, SOCK_STREAM)
		self.msg_text = ""
		self.msg_type = self.MSG_TYPE_INFO
		self.socket_ok = True
		try:
			self.tcpSerSock.bind((socket_addr, socket_port))
		except:
			self.socket_ok = False
		if self.socket_ok:	
			self.tcpSerSock.listen(5)
			self.socket_open = False
		else:
			self.msg_type = self.MSG_TYPE_ERROR
			self.msg_text = "Unable to bind to %s port %d" % (socket_addr, socket_port)

	def close_socket(self):
		self.tcpCliSock.close()
		self.socket_open = False

	def message(self):
		msg_text = self.msg_text
		self.msg_text = ""
		return (self.msg_type, msg_text)

	def send_str(self, s):
		self.tcpCliSock.send('%s\r\n' % s)
	'''
	def send_status(self, x, y):
		self.tcpCliSock.send('$S,%.9f,%.9f\r\n' % (x,y))
	'''

	def packets_available(self):
		if len(self.socket_packets) > 0:
			return True
		else:
			return False

	def get_packet(self):
		if len(self.socket_packets) > 0:
			packet = self.socket_packets[0]
			self.socket_packets.pop(0)
		else:
			packet = ''
		return packet
	
	def update(self, time_now):
		close_socket_now = False
		if self.socket_open:
			d = ''
			try:
				d = self.tcpCliSock.recv(1024)
				if not d:
					if self.debug:
						self.msg_type = self.MSG_TYPE_INFO
						self.msg_text = "Connection closed"
					close_socket_now = True
				else:
					if d[0]=='$' and d[1]=='P' and d[2]=='F' and d[3]=='M' and d[-1]=='\n':
						self.socket_packets.append (d.rstrip())
						self.socket_timeout = time_now + self.max_idle
			except:
				if rospy.get_time() > self.socket_timeout:
					if self.debug:
						self.msg_type = self.MSG_TYPE_WARNING
						self.msg_text = "Socket timeout, closing"
					close_socket_now = True
		else:
			if self.debug:
				self.msg_type = self.MSG_TYPE_INFO
				self.msg_text = "Waiting for incoming connection"
			socket_err = False
			try: # the 'try' is required to be able to ctrl-c without errors
				self.tcpCliSock, addr = self.tcpSerSock.accept()
			except:
				socket_err = True
			if socket_err == False:
				#self.tcpCliSock.settimeout(None)
				self.tcpCliSock.setblocking(0)			
				self.socket_open = True
				self.socket_timeout = rospy.get_time() + self.max_idle
				if self.debug:
					self.msg_type = self.MSG_TYPE_INFO
					self.msg_text = "Connection established from %s port %d" % (addr[0],addr[1])
				outdata = "$PFMRH,1.0,FroboMind Route Plan Server\r\n"
				self.tcpCliSock.send(outdata)

		if close_socket_now:
			self.close_socket()

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")

		# static parameters
		self.update_rate = 50 # set update frequency [Hz]
		self.deadman_tout_duration = 0.2 # [s]
		self.cmd_vel_tout_duration = 0.2 # [s]

		# variables
		self.send_status_interval = 0.0
		self.send_status_timeout = 0.0
		self.pose_e = 0.0
		self.pose_n = 0.0
		self.pose_orientation = 0.0
		self.status_linear_vel = 0.0
		self.status_angular_vel = 0.0
		self.status_mode = 0
		self.status_task = 0
		self.status_b_id = ""

		# get parameters
		socket_addr = rospy.get_param("~socket_address",'localhost')
		socket_port = rospy.get_param("~socket_port",'8080')
		socket_timeout = rospy.get_param("~socket_timeout",'5')
		self.socket_password = rospy.get_param("~socket_password",'')
		self.debug = rospy.get_param("~debug",'false')

		# get topic names
		topic_status_sub =  rospy.get_param("~status_sub",'/fmInformation/wptnav_status')
		topic_hmi_pub = rospy.get_param("~hmi_pub",'/fmDecision/hmi')
		topic_routept = rospy.get_param("~routept_pub",'/fmPlan/route_point')

		# setup HMI publisher
		self.hmi_pub = rospy.Publisher(topic_hmi_pub, StringArrayStamped)
		self.hmi_msg = StringArrayStamped()
		self.hmi_msg.data = ['', ''] # initialize string array

		# setup routept publisher
		self.routept_pub = rospy.Publisher(topic_routept, RoutePt)
		self.routept_msg = RoutePt()

		# setup subscription topic callbacks
		rospy.Subscriber(topic_status_sub, waypoint_navigation_status, self.on_status_message)

		# setup socket
		self.sd = socketd(socket_addr, socket_port, socket_timeout, self.socket_password, self.debug)
		if self.sd.socket_ok == True:
			# call updater function
			self.r = rospy.Rate(self.update_rate)
			self.updater()
		else:
			(msg_type, msg_text) = self.sd.message()
			rospy.logerr(rospy.get_name() + ": %s", msg_text)

	def on_status_message(self, msg):
		self.pose_e = msg.easting
		self.pose_n = msg.northing
		self.pose_orientation = msg.orientation
		self.status_linear_vel = msg.linear_speed
		self.status_angular_vel = msg.angular_speed
		self.status_b_id = msg.b_id
		if msg.state == 1 or msg.state == 2: # automode = True
			self.status_mode = 1
		else:
			self.status_mode = 0
		self.status_task = msg.task

	def publish_routept_message(self):
		self.routept_msg.header.stamp = rospy.Time.now()
		self.routept_pub.publish(self.routept_msg)

	def publish_hmi_message(self, id_str, value_str):
		self.hmi_msg.header.stamp = rospy.Time.now()
		self.hmi_msg.data[0] = id_str
		self.hmi_msg.data[1] = value_str
		self.hmi_pub.publish (self.hmi_msg)

	def socket_updater(self):
		self.sd.update(rospy.get_time())
		(msg_type, msg_text) = self.sd.message()
		if msg_text != "":
			if msg_type == self.sd.MSG_TYPE_INFO:
				rospy.loginfo(rospy.get_name() + ": %s", msg_text)
			elif msg_type == self.sd.MSG_TYPE_WARNING:
				rospy.logwarn(rospy.get_name() + ": %s", msg_text)
			elif msg_type == self.sd.MSG_TYPE_ERROR:
				rospy.logerr(rospy.get_name() + ": %s", msg_text)

	def updater(self):
		while not rospy.is_shutdown():
			self.socket_updater()
			while self.sd.packets_available():
				p = self.sd.get_packet()
				if p[4]=='R' and p[5]=='Q': # quit
					if self.debug:
						rospy.loginfo(rospy.get_name() + ": Closing socket on user request")
					self.sd.close_socket()
				elif p[4]=='R' and p[5]=='K': # keep socket alive
					pass
				elif p[4]=='H' and p[5]=='M': # mode select
					if self.sd.socket_password_received:
						data = p.split (',') # split into comma separated list
						if len(data) > 1 and data[1] != '':
							value = data[1]
							if value == '0' or value == '1':
								if value == '0':
									self.publish_hmi_message (str(HMI_ID_MODE), str(HMI_MODE_MANUAL))
								elif value == '1':
									self.publish_hmi_message (str(HMI_ID_MODE), str(HMI_MODE_AUTO))
								self.sd.send_str('$PFMHM,ok')
								if self.debug:
									if value == '0':
										rospy.loginfo(rospy.get_name() + ": Switching to manual mode")
									else:
										rospy.loginfo(rospy.get_name() + ": Switching to autonomous mode")
							else:
								self.sd.send_str('$PFMHM,err')
						else:
							self.sd.send_str('$PFMHM,err')
					else:
						self.sd.send_str('$PFMHM,passwd')			

				elif p[4]=='R' and p[5]=='S': # subscribe to status
					if self.sd.socket_password_received:
						message_ok = False
						data = p.split (',') # split into comma separated list
						if len(data) > 1 and data[1] != '':
							value = data[1].rstrip()
							if value != '':
								time = float(value)
								if time >= 0.0:
									message_ok = True
									self.send_status_interval = time
									self.send_status_timeout = 0.0
						if message_ok == True:
							self.sd.send_str('$PFMRS,ok')
							if self.debug:
								rospy.loginfo(rospy.get_name() + ": Subscribing to status")
						else:
							self.sd.send_str('$PFMRS,err')
							if self.debug:
								rospy.loginfo(rospy.get_name() + ": Error subscribing to status")
					else:
						self.sd.send_str('$PFMRS,passwd')			

				elif p[4]=='R' and p[5]=='D': # delete route plan
					if self.sd.socket_password_received:
						self.routept_msg.cmd = ROUTEPT_CMD_DELETE
						self.publish_routept_message()
						self.sd.send_str('$PFMRD,ok')
						if self.debug:
							rospy.loginfo(rospy.get_name() + ": Deleting route plan")
					else:
						self.sd.send_str('$PFMRD,passwd')			
				elif p[4]=='R' and p[5]=='G': # wpt using geodetic coordinates
					pass
				elif p[4]=='R' and p[5]=='E': # wpt using ENU coordinates
					if self.sd.socket_password_received:
						self.sd.send_str('$PFMRE,ok')
						data = p.split (',') # split into comma separated list
						self.routept_msg.cmd = ROUTEPT_CMD_APPEND
						self.routept_msg.easting = float(data[1])
						self.routept_msg.northing = float(data[2])
						if len(data) > 3 and data[3] != '':
							self.routept_msg.heading = float(data[3])
						else:
							self.routept_msg.heading = ROUTEPT_INVALID_DATA
						if len(data) > 4 and data[4] != '':
							self.routept_msg.id = data[4]
						else:
							self.routept_msg.id = ''
						if len(data) > 5 and data[5] != '':
							if data[5]=='PP':
								self.routept_msg.nav_mode = ROUTEPT_NAV_MODE_PP
							else:
								self.routept_msg.nav_mode = ROUTEPT_NAV_MODE_AB
						else:
							self.routept_msg.nav_mode = ROUTEPT_INVALID_DATA
						if len(data) > 6 and data[6] != '':
							self.routept_msg.linear_vel = float(data[6])
						else:
							self.routept_msg.linear_vel = ROUTEPT_INVALID_DATA
						if len(data) > 7 and data[7] != '':
							self.routept_msg.angular_vel = float(data[7])
						else:
							self.routept_msg.angular_vel = ROUTEPT_INVALID_DATA
						if len(data) > 8 and data[8] != '':
							self.routept_msg.pause = float(data[8])
						else:
							self.routept_msg.pause = ROUTEPT_INVALID_DATA
						if len(data) > 9 and data[9] != '':
							self.routept_msg.task = int(data[9])
						else:
							self.routept_msg.task = ROUTEPT_INVALID_DATA
						
						self.publish_routept_message()
						if self.debug:
							rospy.loginfo(rospy.get_name() + ": RoutePt (ENU) %s E%.3f N%.3f received" % (self.routept_msg.id, self.routept_msg.easting, self.routept_msg.northing))
					else:
						self.sd.send_str('$PFMRE,passwd')				
				else:
					rospy.logwarn(rospy.get_name() + ": Unknown packet: %s" % p)	

			# is it time to send a status?
			if self.sd.socket_open and self.send_status_interval > 0 and self.send_status_timeout < rospy.get_time():
				self.send_status_timeout = rospy.get_time() + self.send_status_interval
				self.sd.send_str('$PFMRS,%.3f,%.3f,%.3f,%.3f,%.3f,%ld,%ld,%s,,,,,,' % (self.pose_e, self.pose_n, self.pose_orientation, self.status_linear_vel, self.status_angular_vel, self.status_mode, self.status_task, self.status_b_id))
			elif self.sd.socket_open == False:
				self.send_status_interval = 0.0

			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    rospy.init_node('route_plan_server')
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

