#!/usr/bin/env python
#/****************************************************************************
# Show route plan!
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
2014-09-19 KJ First version
"""

# imports
from pylab import *
from math import pi, sqrt

#*****************************************************************************
# USER CONFIGURABLE PARAMETERS BEGIN HERE

file_name = 'waypoints.txt'
plot_from = 0
plot_to = 10000000

list_nearby_waypoints = False
nearby_threshold = 2.0

plot_waypoint_positions = False
plot_robot_position = False
e_robot = 545981.07281
n_robot = 6256815.483139


# USER CONFIGURABLE PARAMETERS END HERE
#*****************************************************************************

# options
simdir_version = '2014-09-19'
show_plot = True
save_file = True

# csv file waypoint list structure
CSV_E = 0 
CSV_N = 1
CSV_HEADING = 2
CSV_ID = 3
CSV_NAV_MODE = 4
CSV_LIN_VEL = 5
CSV_ANG_VEL = 6
CSV_PAUSE = 7
CSV_TASK = 8
ROUTEPT_INVALID_DATA = -1000000
ROUTEPT_NAV_MODE_PP = 0 # pure pursuit
ROUTEPT_NAV_MODE_AB = 1 # AB line navigation

class waypoint_list():
	def __init__(self):
		self.list = []
		self.wpt_def_nav_mode = "PP"
		self.wpt_def_lin_vel = 1.0
		self.wpt_def_ang_vel = 1.0
		self.wpt_def_pause = 0.0
		self.wpt_def_task = 0

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



# find coordinate minimum and maximum
def enu_min_max(w):
	min_e = 100000000000000
	min_n = 100000000000000
	max_e = -100000000000000
	max_n = -100000000000000
	for i in xrange(len(w)):
		if min_e > w[i][0]:
			min_e = w[i][0]
		elif max_e < w[i][0]:
			max_e = w[i][0]
		if min_n > w[i][1]:
			min_n = w[i][1]
		elif max_n < w[i][1]:
			max_n = w[i][1]
	return (min_e, max_e, min_n, max_n)


wl = waypoint_list()
wl.load_from_csv_ne_format(file_name)
print 'Number of waypoints', len(wl.list)


if list_nearby_waypoints == True:
	for i in xrange(len(wl.list)):
		dist = sqrt((e_robot - wl.list[i][0])**2 + (e_robot - wl.list[i][0])**2)
		if dist <= nearby_threshold:
			print 'Nearby: %s %.1fm' % (wl.list[i][3], dist)


# plot the route plan
e_vals = []
n_vals = []
i = plot_from
if plot_to > len(wl.list):
	plot_to = len(wl.list)
while i < plot_to:
	e_vals.append (wl.list[i][0])
	n_vals.append (wl.list[i][1])
	i += 1
plot(e_vals, n_vals,'r')
if plot_waypoint_positions == True:
	plot(e_vals, n_vals,'ro')
if plot_robot_position == True:
	plot(e_robot, n_robot,'go')
(min_e, max_e, min_n, max_n) = enu_min_max(wl.list)
border = 1.0
title ('Route Plan')
xlabel('Easting [m]')
ylabel('Northing [m]')
axis('equal')
grid (True)
xlim(min_e-border, max_e+border)
ylim(min_n-border, max_n+border)
show()

