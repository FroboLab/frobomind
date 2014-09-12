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

2014-09-10 KJ First version
"""

import rospy
from msgs.msg import StringArrayStamped, RoutePt
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
		self.send_status_interval = 1.0
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

	def send_status(self, x, y):
		self.tcpCliSock.send('$S,%.9f,%.9f\r\n' % (x,y))

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
						self.socket_packets.append (d)
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
				outdata = "$PFMRH,1.0,FroboMind Route Plan Socket Interface\r\n"
				self.tcpCliSock.send(outdata)

		if close_socket_now:
			self.close_socket()

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")
		# defines
		self.ROUTEPT_CMD_DELETE = 0
		self.ROUTEPT_CMD_DELETE_THEN_APPEND = 1
		self.ROUTEPT_CMD_APPEND = 2
		self.ROUTEPT_MODE_PP = 0
		self.ROUTEPT_MODE_MCTE = 1
		self.ROUTEPT_INVALID_DATA = -1000000
		self.count = 0

		# static parameters
		self.update_rate = 50 # set update frequency [Hz]
		self.deadman_tout_duration = 0.2 # [s]
		self.cmd_vel_tout_duration = 0.2 # [s]

		# get parameters
		socket_addr = rospy.get_param("~socket_address",'localhost')
		socket_port = rospy.get_param("~socket_port",'8080')
		socket_timeout = rospy.get_param("~socket_timeout",'5')
		self.socket_password = rospy.get_param("~socket_password",'')
		self.debug = rospy.get_param("~debug",'false')

		# get topic names
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

		# setup socket
		self.sd = socketd(socket_addr, socket_port, socket_timeout, self.socket_password, self.debug)
		if self.sd.socket_ok == True:
			# call updater function
			self.r = rospy.Rate(self.update_rate)
			self.updater()
		else:
			(msg_type, msg_text) = self.sd.message()
			rospy.logerr(rospy.get_name() + ": %s", msg_text)

	def publish_routept_message(self):
		self.routept_msg.header.stamp = rospy.Time.now()
		self.routept_pub.publish(self.routept_msg)

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
				elif p[4]=='R' and p[5]=='D': # delete route plan
					if self.sd.socket_password_received:
						self.routept_msg.cmd = self.ROUTEPT_CMD_DELETE
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
						data = p.split (',') # split into comma separated list
						self.routept_msg.cmd = self.ROUTEPT_CMD_APPEND
						self.routept_msg.easting = float(data[1])
						self.routept_msg.northing = float(data[2])
						if len(data) > 3 and data[3] != '':
							self.routept_msg.heading = float(data[3])
						else:
							self.routept_msg.heading = self.ROUTEPT_INVALID_DATA
						if len(data) > 4 and data[4] != '':
							self.routept_msg.id = data[4]
						else:
							self.routept_msg.id = ''
						if len(data) > 5 and data[5] != '':
							if data[5]=='PP':
								self.routept_msg.nav_mode = self.ROUTEPT_MODE_PP
							else:
								self.routept_msg.nav_mode = self.ROUTEPT_MODE_MCTE
						else:
							self.routept_msg.nav_mode = self.ROUTEPT_MODE_MCTE
						if len(data) > 6 and data[6] != '':
							self.routept_msg.linear_vel = float(data[6])
						else:
							self.routept_msg.linear_vel = self.ROUTEPT_INVALID_DATA
						if len(data) > 7 and data[7] != '':
							self.routept_msg.angular_vel = float(data[7])
						else:
							self.routept_msg.angular_vel = self.ROUTEPT_INVALID_DATA
						if len(data) > 8 and data[8] != '':
							self.routept_msg.pause = float(data[8])
						else:
							self.routept_msg.pause = self.ROUTEPT_INVALID_DATA
						if len(data) > 9 and data[9] != '':
							self.routept_msg.task = int(data[9])
						else:
							self.routept_msg.task = self.ROUTEPT_INVALID_DATA
						
						self.publish_routept_message()
						self.sd.send_str('$PFMRE,ok')
						if self.debug:
							rospy.loginfo(rospy.get_name() + ": RoutePt (ENU) E%.3f N%.3f received" % (self.routept_msg.easting, self.routept_msg.northing))
					else:
						self.sd.send_str('$PFMRE,passwd')				
				else:
					rospy.logwarn(rospy.get_name() + ": Unknown packet: %s" % p)	

			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    rospy.init_node('route_plan_socketd')
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

