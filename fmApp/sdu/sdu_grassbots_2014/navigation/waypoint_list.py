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
from navigation_globals import *
from transverse_mercator_py.transverse_mercator import tranmerc

class waypoint_list():
	def __init__(self, wpt_def_nav_mode, wpt_def_lin_vel, wpt_def_ang_vel, wpt_def_pause, wpt_def_task):
		self.wpt_def_nav_mode = wpt_def_nav_mode
		self.wpt_def_lin_vel = wpt_def_lin_vel
		self.wpt_def_ang_vel = wpt_def_ang_vel
		self.wpt_def_pause = wpt_def_pause
		self.wpt_def_task = wpt_def_task
		self.delete_list()

	def append(self, e, n, heading, name, nav_mode, lin_vel, ang_vel, pause, task):
		if nav_mode == ROUTEPT_INVALID_DATA:
			nav_mode = self.wpt_def_nav_mode
		if lin_vel == ROUTEPT_INVALID_DATA or lin_vel < 0.001:
			lin_vel = self.wpt_def_lin_vel
		if ang_vel == ROUTEPT_INVALID_DATA or ang_vel < 0.001:
			ang_vel = self.wpt_def_ang_vel
		if pause == ROUTEPT_INVALID_DATA:
			pause = self.wpt_def_pause
		if task == ROUTEPT_INVALID_DATA:
			task = self.wpt_def_task
		self.list.append([e, n, heading, name, nav_mode, lin_vel, ang_vel, pause, task])
	
	def delete_list(self):
		self.list = []
		self.next = 0

	def load_from_csv_ne_format(self, filename):
		self.list = []
		file_ok = True
		try:
			lines = [line.rstrip() for line in open(filename)] # read the file and strip \n
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
						e = float (data[CSV_E])
						n = float (data[CSV_N])
					else:
						e = ROUTEPT_INVALID_DATA
						n = ROUTEPT_INVALID_DATA

					if position_available == True:
						wpt_num += 1
						if len(data) > CSV_HEADING and data[CSV_HEADING] != '':
							heading = float(data[CSV_HEADING])
						else:
							heading = ROUTEPT_INVALID_DATA
						if len(data)>CSV_ID and data[CSV_ID] != '':
							name = data[CSV_ID]
						else:
							name = 'Wpt%d' % (wpt_num)
						nav_mode = ROUTEPT_INVALID_DATA
						if  len(data) > CSV_NAV_MODE:
							if data[CSV_NAV_MODE] == 'PP': 
								nav_mode = ROUTEPT_NAV_MODE_PP
							elif data[CSV_NAV_MODE] == 'AB':
								nav_mode = ROUTEPT_NAV_MODE_AB
							else:
								nav_mode = self.wpt_def_nav_mode
						if  len(data) > CSV_LIN_VEL and data[CSV_LIN_VEL] != '': 
							lin_vel = float(data[CSV_LIN_VEL])
						else:
							lin_vel = self.wpt_def_lin_vel
						if  len(data) > CSV_ANG_VEL and data[CSV_ANG_VEL] != '':
							ang_vel = float(data[CSV_ANG_VEL])
						else:
							ang_vel = self.wpt_def_ang_vel
						if  len(data) > CSV_PAUSE and data[CSV_PAUSE] != '':
							pause = float(data[CSV_PAUSE])
						else:
							pause = self.wpt_def_pause
						if  len(data) > CSV_TASK and data[CSV_TASK] != '':
							task = float(data[CSV_TASK])
						else:
							task = self.wpt_def_task

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

	def get_number (self, wpt_num):	
		if wpt_num < len(self.list):
			wpt = self.list[wpt_num]
			self.next = wpt_num + 1
		else:
			wpt = False
		return wpt

	def status (self):		
		return (len(self.list), self.next)

