#!/usr/bin/env python
#/****************************************************************************
# GT-Position serial server node
# Copyright (c) 2017, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#	* Redistributions of source code must retain the above copyright
#	  notice, this list of conditions and the following disclaimer.
#	* Redistributions in binary form must reproduce the above copyright
#	  notice, this list of conditions and the following disclaimer in the
#	  documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
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

2017-06-13 KJ First version for Folkemoedet 2017
2017-09-11 KJ Restructured as a FroboMind component
"""

import rospy
from std_msgs.msg import Int8
from msgs.msg import StringStamped, nmea
import serial
import time
from crc16 import crc16
from slip_protocol import slip_protocol
from struct import pack
from transverse_mercator_py.utm import utmconv
from math import pi, cos, hypot, acos, sin
import datetime

class ROSnode():
	def __init__(self):
		rospy.loginfo(rospy.get_name() + ": Start")

		# static parameters
		self.update_rate = 200 # set update frequency [Hz]
		self.count = 0

		# status parameters
		self.position_ok = Int8()
		self.position_ok = 0
		self.last_heard = 0.0
		self.last_nmea_sent = 0.0
	
		# get parameters
		self.output_format = rospy.get_param("~output_format", "nmea") 
		self.serial_device = rospy.get_param("~serial_device", "/dev/gt_pos_robot")
		self.serial_baudrate = rospy.get_param("~serial_baudrate", 57600)
		self.robot_id = rospy.get_param("~robot_id", "robot")
		self.lat_offset = rospy.get_param("~latitude_offset", 55.47038)
		self.lon_offset = rospy.get_param("~longitude_offset", 10.32966)
		self.alt_offset = rospy.get_param("~altitude_offset", 0.0)

		# get topic names
		self.topic_nmea_from_gt_pos_sub = rospy.get_param("~nmea_from_gt_pos_sub",'/fmData/nmea_from_gt_pos')		
		self.topic_position_ok_pub = rospy.get_param("~position_ok_pub",'/fmData/robot_position_ok')
		self.topic_gpgga_pub = rospy.get_param("~gpgga_pub",'/fmData/robot_gpgga')

		# setup publishers
		self.position_ok_pub = rospy.Publisher(self.topic_position_ok_pub, Int8, queue_size=0)
		self.gpgga = StringStamped()
		self.gpgga_pub = rospy.Publisher(self.topic_gpgga_pub, StringStamped, queue_size=0)

		# configure and open serial device
		ser_error = False
		try:
			self.ser_dev = serial.Serial(self.serial_device, self.serial_baudrate)
		except Exception as e:
			rospy.logerr(rospy.get_name() + ": Unable to open serial device: %s" % self.serial_device)
			ser_error = True

		if ser_error == False:
			self.slip = slip_protocol()
			self.crc = crc16()

			# instantiate utmconv class
			self.uc = utmconv()
			(self.utm_hem, self.utm_z, self.utm_l, self.utm_e, self.utm_n) = self.uc.geodetic_to_utm (self.lat_offset, self.lon_offset)

			# setup subscription topic callbacks
			rospy.Subscriber(self.topic_nmea_from_gt_pos_sub, nmea, self.on_nmea_msg)

			# call updater function
			self.r = rospy.Rate(self.update_rate)
			self.updater()

	def send_slip(self, lat, lon, alt):
		vDOP = chr(1)
		tDOP = chr(1)
		nDOP = chr(1)
		eDOP = chr(1)
		msg_as_bytes = pack('dddcccc', lat, lon, alt, vDOP, tDOP, nDOP, eDOP)
		crc_result = self.crc.calc(msg_as_bytes)
		crc_out = chr(crc_result >> 8) + chr(crc_result & 0x00FF)
		msg_out = self.slip.encode(msg_as_bytes + crc_out)
		self.ser_dev.write(msg_out)

	def send_nmea(self, time, lat, lon, alt):
		utc = datetime.datetime.utcfromtimestamp(time).strftime('%H%M%S.%f')[:-3]
		if lat != 0.0 or lon != 0.0:
			if lat > 0:
				lath = 'N'
			else:
				lath = 'S'
			if lon > 0:
				lonh = 'E'
			else:
				lonh = 'W'
			latd = int (lat)
			lond = int (lon)
			latm = abs(lat-latd)*60
			lonm = abs(lon-lond)*60
			latms = ('%.6f' % (latm % 1))[1:]
			lonms = ('%.6f' % (lonm % 1))[1:]
			lats = '%02d%02d%s' % (latd, int(latm), latms)
			lons = '%03d%02d%s' % (lond, int(lonm), lonms)

			msg_out = 'GPGGA,%s,%s,%c,%s,%c,1,06,1.0,%.3f,M,0.0,M,,*'  % (utc, lats, lath, lons, lonh, alt)
		else:
			msg_out = 'GPGGA,%s,,,,,0,0,,,M,,M,,*' % (utc)

		# calculate and append NMEA checksum
		cs = 0
		for i in range(len(msg_out)-3):
		  cs ^= ord(msg_out[i])		  
		msg_out = '$' + msg_out + format(cs, '02x')
		self.ser_dev.write(msg_out)

		self.last_nmea_sent = rospy.get_time()
		self.gpgga.header.stamp = rospy.Time.now()
		self.gpgga.data = msg_out
		self.gpgga_pub.publish (self.gpgga)

	def send_update (self, time, lat, lon, alt):
		if self.output_format == 'slip':
			self.send_slip(lat, lon, alt)
		elif self.output_format == 'nmea':
			self.send_nmea(time, lat, lon, alt)


	def on_nmea_msg(self, msg):
		if msg.type == 'GTPOS' and msg.data[1] == self.robot_id:
			self.last_heard = rospy.get_time()

			x_mm = float(msg.data[2])
			y_mm = float(msg.data[3])
			z_mm = float(msg.data[4])
			rel_x = x_mm/1000.0
			rel_y = y_mm/1000.0
			rel_z = z_mm/1000.0
			easting  = self.utm_e + rel_x
			northing = self.utm_n + rel_y
			alt = self.alt_offset + rel_z
			# convert back from UTM to geodetic
			(lat, lon) = self.uc.utm_to_geodetic (self.utm_hem, self.utm_z, easting, northing)
			self.send_update (self.last_heard, lat, lon, alt)

	def updater(self):
		while not rospy.is_shutdown():
			self.count += 1
		
			# if we haven't received a position update for 1 second, send an update with no info
			if self.last_nmea_sent + 1.0 < rospy.get_time():
				self.last_nmea_sent = rospy.get_time()
				self.send_update (self.last_nmea_sent, 0.0, 0.0, 0.0)

			# send position_ok
			if self.count % 20 == 0:
				if self.last_heard + 1.0 < rospy.get_time():
					self.position_ok = 0
				else:
					self.position_ok = 1
				self.position_ok_pub.publish(self.position_ok)

			# go back to sleep
			self.r.sleep()

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('gt_serial_server')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ROSnode()
    except rospy.ROSInterruptException:
		pass


