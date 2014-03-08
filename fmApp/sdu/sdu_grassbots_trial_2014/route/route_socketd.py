#!/usr/bin/env python
#/****************************************************************************
# route plan socket daemon
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

2014-01-24 KJ First version
"""

import rospy
from msgs.msg import RoutePt
from socket import *

class socketd():
	def  __init__(self, addr, port, max_idle, debug):
		self.MSG_TYPE_INFO = 0
		self.MSG_TYPE_WARNING = 1
		self.MSG_TYPE_ERROR = 2
		self.max_idle = max_idle
		self.socket_timeout = 0.0
		self.last_recv = 0.0
		self.debug = debug
		self.tcpSerSock = socket(AF_INET, SOCK_STREAM)
		self.socket_ok = True
		self.msg_text = ""
		self.msg_type = self.MSG_TYPE_INFO
		try:
			self.tcpSerSock.bind((addr, port))
		except:
			self.socket_ok = False
		if self.socket_ok:	
			self.tcpSerSock.listen(5)
			self.socket_error = False
			self.socket_open = False
		else:
			self.msg_type = self.MSG_TYPE_ERROR
			self.msg_text = "Unable to bind to %s port %d" % (addr, port)

	def close_socket(self):
		self.tcpCliSock.close()
		self.socket_open = False

	def message(self):
		msg_text = self.msg_text
		self.msg_text = ""
		return (self.msg_type, msg_text)

	def send_ack(self, ack):
		if ack:
			self.tcpCliSock.send('+\r\n')
		else:
			self.tcpCliSock.send('-\r\n')

	def send_status(self, x, y):
		self.tcpCliSock.send('$S,%.9f,%.9f\r\n' % (x,y))
	
	def update(self):
		close_socket_now = False
		if self.socket_open:
			try:
				d = self.tcpCliSock.recv(1024)
				if not d:
					self.msg_type = self.MSG_TYPE_INFO
					self.msg_text = "Connection closed"
					close_socket_now = True
				else:
					if d[0]=='$':
						if (d[1]=='k' or d[1]=='K') and d[2]=='\r' and d[3]=='\n': # keep alive
							self.socket_timeout = rospy.get_time() + self.max_idle
							print "Keep alive"
						elif d[1]=='w' or d[1]=='W': # wpt lat/lon
							self.socket_timeout = rospy.get_time() + self.max_idle
							self.send_ack(True)
							print "WPT %d bytes: '%s'" % (len(d), d)
						elif d[1]=='e' or d[1]=='E': # wpt ENU
							self.socket_timeout = rospy.get_time() + self.max_idle
							self.send_ack(True)
							print "WPT ENU %d bytes: '%s'" % (len(d), d)
						elif (d[1]=='r' or d[1]=='R') and d[2]=='\r' and d[3]=='\n': # reset route plan
							self.socket_timeout = rospy.get_time() + self.max_idle
							self.send_ack(True)
							print "Reset route plan!"
						elif (d[1]=='q' or d[1]=='Q') and d[2]=='\r' and d[3]=='\n': # quit
							self.msg_type = self.MSG_TYPE_INFO
							self.msg_text = "Closing socket"
							close_socket_now = True
						else:
							self.send_ack(False)
					else:
						self.send_ack(False)
			except:
				if rospy.get_time() > self.socket_timeout:
					self.msg_type = self.MSG_TYPE_WARNING
					self.msg_text = "Socket timeout, closing"
					close_socket_now = True
		else:
			if self.debug:
				self.msg_type = self.MSG_TYPE_INFO
				self.msg_text = "Waiting for incoming connection"
			self.tcpCliSock, addr = self.tcpSerSock.accept()
			#self.tcpCliSock.settimeout(None)
			self.tcpCliSock.setblocking(0)			
			self.socket_open = True
			self.socket_timeout = rospy.get_time() + self.max_idle
			if self.debug:
				self.msg_type = self.MSG_TYPE_INFO
				self.msg_text = "Connection established from %s port %d" % (addr[0],addr[1])
			outdata = "FroboMind Route Interface\n"
			self.tcpCliSock.send(outdata)

		if close_socket_now:
			self.close_socket()

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
		addr = rospy.get_param("~socket_address",'localhost')
		port = rospy.get_param("~socket_port",'8080')
		timeout = rospy.get_param("~socket_timeout",'5')
		self.debug = rospy.get_param("~debug",'false')

		# get topic names
		self.topic_routept = rospy.get_param("~routept_pub",'/fmPlan/route_point')

		# setup topic publishers
		self.routept_pub = rospy.Publisher(self.topic_routept, RoutePt)
		self.routept_msg = RoutePt()

		# setup subscription topic callbacks

		# setup socket
		self.sd = socketd(addr, port, timeout, self.debug)
		if self.sd.socket_ok == True:

			# call updater function
			self.r = rospy.Rate(self.update_rate)
			self.updater()
		else:
			(msg_type, msg_text) = self.sd.message()
			rospy.logerr(rospy.get_name() + ": %s", msg_text)


	def socket_updater(self):
		self.sd.update()
		(msg_type, msg_text) = self.sd.message()
		if msg_text != "":
			if msg_type == self.sd.MSG_TYPE_INFO:
				rospy.loginfo(rospy.get_name() + ": %s", msg_text)
			elif msg_type == self.sd.MSG_TYPE_WARNING:
				rospy.logwarn(rospy.get_name() + ": %s", msg_text)
			elif msg_type == self.sd.MSG_TYPE_ERROR:
				rospy.logerr(rospy.get_name() + ": %s", msg_text)

	def publish_routept_message(self):
		self.routept_msg.header.stamp = rospy.Time.now()
		self.routept_msg.reset = True
		self.routept_msg.easting = 0.0
		self.routept_msg.northing = 0.0
		self.routept_pub.publish(self.routept_msg);

	def updater(self):
		while not rospy.is_shutdown():
			self.socket_updater()
			self.count += 1
			if self.count % self.update_rate == 0:
				self.sd.send_status(10.38, 55.40)
			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    rospy.init_node('route_socketd')
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass

