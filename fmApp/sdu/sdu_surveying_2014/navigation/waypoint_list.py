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
Notice that the waypoint list must be in ROS_HOME which by default is ~/.ros

Supported waypoint list format:
	easting, northing, yaw, wptid, mode, tolerance, lin_spd, ang_spd, task, wait
Reference: https://docs.google.com/document/d/1nXmZ2Yz4_EzWaQ4GabGi4fah6phSRXDoe_4mvrjz-kA/edit#

2013-06-07 KJ First version
2013-11-14 KJ Changed the waypoint list format to:
			  [easting, northing, yaw, wptid, modestr, tolerance, lin_spd, ang_spd, wait, implement]
              Added support for flexible waypoint csv length

"""

# imports
#import csv

class waypoint_list():
	def __init__(self):
		self.IMPLEMENT_INVALID = -10000000.0
		self.wpts = []
		self.next = 0

	def load_from_csv_ne_format(self, filename):
		self.wpts = []
		lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
		wpt_num = 0
		for i in xrange(len(lines)): # for all lines
			if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
				data = lines[i].split (',') # split into comma separated list
				if len(data) >= 2 and data[0] != '' and data[1] != '':
					wpt_num += 1
					e = float (data[0])
					n = float (data[1])
					if len(data) >= 3 and data[2] != '':
						yaw = float(data[2])
					else:
						yaw = -1
					if len(data) >= 4 and data[3] != '':
						name = data[3]
					else:
						name = 'Wpt%d' % (wpt_num)
					mode = 1 # default is 'minimize cross track error'
					if  len(data) >= 5 and data[4] == 'STWP': # 'straight to waypoint' 
						mode = 0 
					if len(data) >= 6 and data[5] != '':  # waypoint reach tolerance
						tol = float(data[5])
					else:
						tol = 0.0 
					if  len(data) >= 7 and data[6] != '': # linear speed
						lin_spd = float(data[6])
					else:
						lin_spd = 0.0 
					if  len(data) >= 8 and data[7] != '': # angular speed
						ang_spd = float(data[7])
					else:
						ang_spd = 0.0
					if  len(data) >= 9 and data[8] != '': # wait after reaching wpt
						wait = float(data[8])
					else:
						wait = -1.0
					if  len(data) >= 10 and data[9] != '': # implement command
						implement = float(data[9])
					else:
						implement = self.IMPLEMENT_INVALID

					self.wpts.append([e, n, yaw, name, mode, tol, lin_spd, ang_spd, wait, implement])
				else:
					print 'Erroneous waypoint'
		self.next = 0

	#def add (self, easting, northing, yaw, wptid, mode, tolerance, lin_spd, ang_spd, implement, wait): 
	#	self.wpts.append([easting, northing, yaw, wptid, mode, tolerance, lin_spd, ang_spd, implement, wait])

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

