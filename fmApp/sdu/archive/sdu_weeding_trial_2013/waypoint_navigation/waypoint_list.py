#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation: Waypoint list
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
2013-06-07 KJ First version

Notice that the waypoint list must be in ROS_HOME which by default is ~/.ros

Supported waypoint list format:
	easting,northing,,id,mode,tolerance,speed,
Reference: https://docs.google.com/document/d/1nXmZ2Yz4_EzWaQ4GabGi4fah6phSRXDoe_4mvrjz-kA/edit#

"""

# imports
import csv

class waypoint_list():
	def __init__(self):
		self.wpts = []
		self.next = 0

	def load_from_csv_ne_format(self, filename):
		self.wpts = []
		file = open(filename, 'rb')
		lines = csv.reader(file, delimiter='\t')
		#for easting, northing, yaw, wptid, modestr, tolerance, speed, task in lines:
		modestr = 'MCTE'
		tolerance = 0.0
		speed = 0.0
		for easting, northing, wptid in lines:
			if modestr == 'STWP': # 'straight to waypoint'
				mode = 0
			elif modestr == 'MCTE': # 'minimize cross track error'
				mode = 1			
			self.wpts.append([float(easting), float(northing), wptid, mode, float(tolerance), float(speed)])
		file.close()
		self.next = 0

	def add (self, easting, northing, wptid, mode, tolerance, speed): 
		self.wpts.append([easting, northing, wptid, mode, tolerance, speed])

	def get_next (self):	
		if self.next < len(self.wpts):
			wpt = self.wpts[self.next]
			self.next += 1
		else:
			wpt = False
		return wpt

	def get_previous (self):
		prev_wpt = False
		wpt = False
		if self.next > 1:
			self.next -= 1
			wpt = self.wpts[self.next-1]
			if self.next > 1:
				prev_wpt = self.wpts[self.next-2]
		return (wpt, prev_wpt)

	def status (self):		
		return (len(self.wpts), self.next)

