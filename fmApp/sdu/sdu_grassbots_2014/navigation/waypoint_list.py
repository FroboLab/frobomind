#!/usr/bin/env python
#/****************************************************************************
# Waypoint Navigation: Waypoint list
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
Supported waypoint list format:
	easting, northing, yaw, wptid, mode, tolerance, lin_spd, ang_spd, task, wait
Reference: https://docs.google.com/document/d/1nXmZ2Yz4_EzWaQ4GabGi4fah6phSRXDoe_4mvrjz-kA/edit#

2013-06-07 KJ First version
2013-11-14 KJ Changed the waypoint list format to:
			  [easting, northing, yaw, wptid, modestr, tolerance, lin_spd, ang_spd, wait, implement]
              Added support for flexible waypoint csv length
2014-09-10 KJ Changed from static CSV file load to dynamic ROS topic management
"""

#imports
from transverse_mercator_py.transverse_mercator import tranmerc

class waypoint_list():
	def __init__(self):
		self.ROUTEPT_MODE_PP = 0
		self.ROUTEPT_MODE_MCTE = 1
		self.ROUTEPT_INVALID_DATA = -1000000
		self.W_E = 0 
		self.W_N = 1
		self.W_HEADING = 2
		self.W_ID = 3
		self.W_NAV_MODE = 4
		self.W_LIN_VEL = 5
		self.W_ANG_VEL = 6
		self.W_PAUSE = 7
		self.W_TASK = 8
		self.delete_list()

	def append(self, e, n, heading, name, nav_mode, lin_vel, ang_vel, wait, task):
		self.list.append([e, n, heading, name, nav_mode, lin_vel, ang_vel, wait, task])
	
	def delete_list(self):
		self.list = []
		self.next = 0

	def load_from_csv_ne_format(self, filename):
		self.list = []
		file_ok = True
		try:
			lines = [line.rstrip('\n') for line in open(filename)] # read the file and strip \n
		except:
			file_ok = False
		if file_ok == True:
			wpt_num = 0
			for i in xrange(len(lines)): # for all lines
				if len(lines[i]) > 0 and lines[i][0] != '#': # if not a comment or empty line
					data = lines[i].split (',') # split into comma separated list
					position_available = False
					if len(data) >= 2 and data[0] != '' and data[1] != '':
						position_available = True
						e = float (data[0])
						n = float (data[1])
					else:
						e = self.ROUTEPT_INVALID_DATA
						n = self.ROUTEPT_INVALID_DATA
					if len(data)>=4 and data[2]!='' and data[3]!='':
						position_available = True
						lat = float(data[2])
						lon = float(data[3])
					else:
						lat = self.ROUTEPT_INVALID_DATA
						lon = self.ROUTEPT_INVALID_DATA

					if position_available == True:
						wpt_num += 1
						if len(data) > 4 and data[4] != '':
							heading = float(data[4])
						else:
							heading = self.ROUTEPT_INVALID_DATA
						if len(data)>5 and data[5] != '':
							name = data[5]
						else:
							name = 'Wpt%d' % (wpt_num)
						nav_mode = self.ROUTEPT_MODE_MCTE # default is 'minimize cross track error'
						if  len(data) > 6 and data[6] == 'STWP': # 'straight to waypoint' 
							nav_mode = self.ROUTEPT_MODE_PP 
						if  len(data) > 7 and data[7] != '': # linear velocity
							lin_vel = float(data[7])
						else:
							lin_vel = 0.0 
						if  len(data) > 8 and data[8] != '': # angular velocity
							ang_vel = float(data[8])
						else:
							ang_vel = 0.0
						if  len(data) > 8 and data[8] != '': # wait after reaching wpt
							pause = float(data[8])
						else:
							pause = self.ROUTEPT_INVALID_DATA
						if  len(data) > 9 and data[9] != '': # task
							task = float(data[9])
						else:
							task = self.ROUTEPT_INVALID_DATA

						self.append(e, n, heading, name, nav_mode, lin_vel, ang_vel, pause, task)
					else:
						print 'Erroneous waypoint'
			self.next = 0

	def get_first (self):	
		if len(self.list) > 0:
			wpt = self.list[0]
			self.next = 1
		else:
			wpt = False
		return wpt

	def get_next (self):	
		if self.next < len(self.list):
			wpt = self.list[self.next]
			self.next += 1
		else:
			wpt = False
		return wpt

	def get_previous (self):
		prev_wpt = False
		wpt = False
		if self.next > 1:
			self.next -= 1
			wpt = self.list[self.next-1]
			if self.next > 1:
				prev_wpt = self.list[self.next-2]
		return (wpt, prev_wpt)

	def status (self):		
		return (len(self.list), self.next)

